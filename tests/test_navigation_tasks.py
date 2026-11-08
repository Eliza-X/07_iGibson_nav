import copy
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

import run_navigation_tasks as batch


def suite(tmp_path, count=3):
    base = yaml.safe_load((batch.ROOT / "fixed_point_path.yaml").read_text(encoding="utf-8"))
    (tmp_path / "base.yaml").write_text(yaml.safe_dump(base), encoding="utf-8")
    tasks = [dict(id=f"task_{i}", initial_pos=[i, 0, 0], initial_orn=[0, 0, 0], target_pos=[i+1, 1, 0]) for i in range(count)]
    path = tmp_path / "tasks.yaml"
    path.write_text(yaml.safe_dump(dict(base_config="base.yaml", tasks=tasks)), encoding="utf-8")
    return path


def fake_runtime(monkeypatch, fail_plan=None):
    events = []
    monkeypatch.setattr(batch, "create_preflight_env", lambda c: SimpleNamespace(scene=object(), close=lambda: events.append("close")))
    def plan(scene, config):
        index = int(config["initial_pos"][0])
        events.append(f"plan{index}")
        if index == fail_plan:
            raise ValueError("No traversable path")
        return None, dict(geodesic_distance_m=2.0)
    monkeypatch.setattr(batch, "plan_path", plan)
    monkeypatch.setattr(batch, "execution_waypoints", lambda *args: [])
    monkeypatch.setattr(batch, "save_and_show", lambda *args, **kwargs: None)
    return events


def test_preflight_all_before_execution_and_failure_isolation(tmp_path, monkeypatch):
    path = suite(tmp_path)
    events = fake_runtime(monkeypatch, fail_plan=1)
    configs = []
    def run(**kwargs):
        config = kwargs["config_data"]
        configs.append(copy.deepcopy(config))
        index = int(config["initial_pos"][0])
        events.append(f"run{index}")
        assert kwargs["wait_for_map"] is False
        if index == 0:
            raise RuntimeError("stuck")
        return dict(accepted=True, reason="success", steps=20, info={"path_length": 2.5})
    monkeypatch.setattr(batch, "run_episode", run)
    result = batch.run_suite(path, output_root=tmp_path / "out")
    assert events == ["plan0", "plan1", "plan2", "close", "run0", "run2"]
    assert result["selected"] == 3 and result["executed"] == 2 and result["succeeded"] == 1
    assert result["tasks"][1]["execution_success"] is None
    assert result["tasks"][2]["path_length_m"] == 2.5
    assert configs[0]["waypoint_log_dir"] != configs[1]["waypoint_log_dir"]
    assert (Path(result["output_dir"]) / "summary.csv").exists()


def test_plan_only_and_selection(tmp_path, monkeypatch):
    path = suite(tmp_path)
    events = fake_runtime(monkeypatch)
    monkeypatch.setattr(batch, "run_episode", lambda **kw: pytest.fail("Must not execute"))
    result = batch.run_suite(path, task_id="task_2", output_root=tmp_path, plan_only=True)
    assert events == ["plan2", "close"]
    assert result["executed"] == 0 and result["success_rate"] is None
    with pytest.raises(ValueError, match="Unknown task"):
        batch.load_suite(path, "missing")


@pytest.mark.parametrize("field,value", [("scene_id", "other"), ("trav_map_erosion", 0),
    ("planner_connectivity", 4), ("turn_direction", "left"), ("id", "../bad"),
    ("execution_max_step", True), ("initial_pos", [float("nan"), 0, 0])])
def test_reject_map_overrides_and_invalid_tasks(tmp_path, field, value):
    path = suite(tmp_path)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    data["tasks"][0][field] = value
    path.write_text(yaml.safe_dump(data), encoding="utf-8")
    with pytest.raises(ValueError):
        batch.load_suite(path)


def test_shared_failure_aborts_and_records(tmp_path, monkeypatch):
    path = suite(tmp_path)
    events = fake_runtime(monkeypatch)
    def run(**kw):
        events.append("failed_setup")
        raise batch.EnvironmentSetupError("scene load failed")
    monkeypatch.setattr(batch, "run_episode", run)
    with pytest.raises(batch.EnvironmentSetupError):
        batch.run_suite(path, output_root=tmp_path / "out")
    assert events.count("failed_setup") == 1
    import json
    report = json.loads(next((tmp_path / "out").glob("*/summary.json")).read_text(encoding="utf-8"))
    assert report["fatal_error"] and all(r["status"] == "batch_aborted" for r in report["tasks"])


def test_map_parameters_and_inputs_preserved(tmp_path):
    path = suite(tmp_path)
    base = yaml.safe_load((tmp_path / "base.yaml").read_text(encoding="utf-8"))
    configs = batch.load_suite(path)
    for _, config in configs:
        for key in ("scene", "scene_id", "floor_num", "trav_map_resolution", "trav_map_erosion", "planner_connectivity", "planner_allow_corner_cutting"):
            assert config[key] == base[key]
    configs[0][1]["robot"]["name"] = "changed"
    assert configs[1][1]["robot"]["name"] == base["robot"]["name"]
