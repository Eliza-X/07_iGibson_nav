"""Independent navigation episodes on one unchanged map."""
import argparse
import copy
import csv
import json
import re
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import numpy as np
import yaml

from astar_planner import plan_path
from env_nonint_fixed_point import EnvironmentSetupError, execution_waypoints, main as run_episode
from plot_fixed_point_path import save_and_show

ROOT = Path(__file__).resolve().parent
FIELDS = ("id", "description", "planning_success", "execution_success", "status", "reason",
          "planned_distance_m", "path_length_m", "steps", "execution_max_step", "log_path", "video_path")


def load_suite(path, task_id=None):
    path = Path(path).resolve()
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) - {"base_config", "tasks"}:
        raise ValueError("Suite accepts only base_config and tasks")
    base_path = (path.parent / data.get("base_config", "fixed_point_path.yaml")).resolve()
    base = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    if not isinstance(base, dict) or base.get("scene") != "gibson" or base.get("build_graph") is not True:
        raise ValueError("Base config requires scene: gibson and build_graph: true")
    if base.get("floor_num", 0) != 0 or isinstance(base.get("floor_num", 0), bool):
        raise ValueError("Only floor 0 is supported")
    if base.get("task") != "point_nav_fixed":
        raise ValueError("Base task must be point_nav_fixed")
    tasks = data.get("tasks")
    if not isinstance(tasks, list) or not tasks:
        raise ValueError("tasks must be a nonempty list")
    seen, selected = set(), []
    for task in tasks:
        allowed = {"id", "description", "initial_pos", "initial_orn", "target_pos", "execution_max_step"}
        if not isinstance(task, dict) or set(task) - allowed:
            raise ValueError("Task fields cannot override map, planner or controller settings")
        identifier = task.get("id", "")
        if not isinstance(identifier, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", identifier):
            raise ValueError("Task id must be a safe ASCII identifier")
        if identifier.lower() in seen:
            raise ValueError("Task ids must be unique (case insensitive)")
        seen.add(identifier.lower())
        config = copy.deepcopy(base)
        for key in ("initial_pos", "initial_orn", "target_pos"):
            value = task.get(key)
            array = np.asarray(value, dtype=float)
            if array.shape != (3,) or not np.isfinite(array).all():
                raise ValueError(f"{identifier}: {key} must contain three finite numbers")
            config[key] = array.tolist()
        budget = task.get("execution_max_step", base.get("execution_max_step", 5000))
        if isinstance(budget, bool) or not isinstance(budget, int) or budget <= 0:
            raise ValueError("execution_max_step must be a positive integer")
        config.update(execution_max_step=budget, max_step=budget, floor_num=0)
        config["waypoint_resolution"] = config.get("trav_map_resolution", 0.1)
        if task_id is None or identifier == task_id:
            selected.append((task, config))
    if not selected:
        raise ValueError(f"Unknown task: {task_id}")
    return selected


def create_preflight_env(config):
    from igibson.envs.igibson_env import iGibsonEnv
    from igibson.utils.assets_utils import download_assets, download_demo_data
    download_assets()
    if config["scene_id"] == "Rs":
        download_demo_data()
    return iGibsonEnv(config_file=copy.deepcopy(config), mode="headless")


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, default=str), encoding="utf-8")


def save_summary(directory, rows, fatal_error=None):
    payload = {"tasks": rows, "fatal_error": fatal_error,
               "selected": len(rows), "planning_passed": sum(r["planning_success"] is True for r in rows),
               "executed": sum(r["execution_success"] is not None for r in rows),
               "succeeded": sum(r["execution_success"] is True for r in rows)}
    payload["success_rate"] = payload["succeeded"] / payload["executed"] if payload["executed"] else None
    write_json(directory / "summary.json", payload)
    with (directory / "summary.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return payload


def run_suite(tasks_path=ROOT / "navigation_tasks.yaml", task_id=None, output_root=ROOT / "outputs/batches",
              plan_only=False, headless=False, display_mode=None, short_exec=False):
    selected = load_suite(tasks_path, task_id)
    directory = Path(output_root).resolve() / (datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid4().hex[:8])
    directory.mkdir(parents=True, exist_ok=False)
    rows, configs = [], []
    for task, config in selected:
        folder = directory / task["id"]
        folder.mkdir()
        config["waypoint_log_dir"] = str(folder / "logs")
        config["video_output_dir"] = str(folder / "videos")
        config["execution_max_step"] = 10 if short_exec else config["execution_max_step"]
        config["max_step"] = config["execution_max_step"]
        if display_mode is not None:
            config["display_mode"] = display_mode
        (folder / "config.yaml").write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
        configs.append(config)
        rows.append(dict(id=task["id"], description=task.get("description", ""), planning_success=None,
                         execution_success=None, status="pending", reason="", execution_max_step=config["execution_max_step"]))
    save_summary(directory, rows)
    print(f"Batch output: {directory}")
    try:
        # Inspect every route before any robot reset/step. Never edit scene maps.
        env = create_preflight_env(configs[0])
        try:
            for row, config in zip(rows, configs):
                try:
                    floor_map, result = plan_path(env.scene, config)
                    execution_waypoints(env.scene, result, config)
                except ValueError as exc:
                    row.update(planning_success=False, status="planning_failed", reason=str(exc))
                else:
                    save_and_show(floor_map, result, directory / row["id"], show=False)
                    row.update(planning_success=True, status="planned", planned_distance_m=result["geodesic_distance_m"])
                save_summary(directory, rows)
        finally:
            env.close()
        if not plan_only:
            for row, config in zip(rows, configs):
                if not row["planning_success"]:
                    continue
                row.update(status="running")
                save_summary(directory, rows)
                try:
                    result = run_episode(config_data=config, headless=headless, display_mode=display_mode,
                                         short_exec=short_exec, wait_for_map=False)
                except (EnvironmentSetupError, ImportError):
                    raise
                except Exception as exc:
                    row.update(execution_success=False, status="execution_error", reason=f"{type(exc).__name__}: {exc}")
                else:
                    write_json(directory / row["id"] / "result.json", result)
                    row.update(execution_success=bool(result["accepted"]), status="completed", reason=result["reason"],
                               steps=result["steps"], path_length_m=result.get("info", {}).get("path_length"),
                               log_path=result.get("log_path"), video_path=result.get("video_path"))
                save_summary(directory, rows)
    except BaseException as exc:
        for row in rows:
            if row["status"] in ("pending", "planned", "running"):
                row.update(status="batch_aborted", reason=f"{type(exc).__name__}: {exc}")
        save_summary(directory, rows, f"{type(exc).__name__}: {exc}")
        raise
    report = save_summary(directory, rows)
    report["output_dir"] = str(directory)
    print(f"Planning passed: {report['planning_passed']}/{report['selected']}; executed: {report['executed']}; succeeded: {report['succeeded']}")
    return report


def cli(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tasks", type=Path, default=ROOT / "navigation_tasks.yaml")
    parser.add_argument("--task", help="Run a single task id; omit to run all")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "outputs/batches")
    parser.add_argument("--plan-only", action="store_true", help="Save route previews without reset or step")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--display-mode", choices=("classic", "video"))
    parser.add_argument("--short-exec", action="store_true", help="10 steps per task, startup check only")
    args = parser.parse_args(argv)
    return run_suite(args.tasks, args.task, args.output_dir, args.plan_only, args.headless, args.display_mode, args.short_exec)


if __name__ == "__main__":
    cli()
