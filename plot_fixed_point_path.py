"""Plan with iGibson's scene API and display a static map; never execute actions."""

import argparse
import json
import os
from pathlib import Path

import numpy as np
import yaml


def plan_path(scene, config):
    """Validate endpoints before the API can attach off-graph nodes."""
    floor = config.get("floor_num", 0)
    if isinstance(floor, bool) or not isinstance(floor, int):
        raise ValueError("floor_num must be an integer")
    maps = getattr(scene, "floor_map", [])
    if not scene.build_graph or not 0 <= floor < len(maps):
        raise ValueError("Navigation graph/map unavailable or floor_num out of range")
    # One returned point per graph node avoids drawing shortcuts across corners.
    if scene.waypoint_interval != 1:
        raise ValueError("Set waypoint_resolution equal to trav_map_resolution")
    endpoints = []
    for name in ("initial_pos", "target_pos"):
        point = np.asarray(config[name], dtype=float)
        if point.shape not in ((2,), (3,)) or not np.isfinite(point).all():
            raise ValueError(f"{name} must contain finite XY or XYZ coordinates")
        xy = point[:2]
        if not scene.has_node(floor, xy):
            raise ValueError(f"{name} {xy.tolist()} is outside the traversable graph on floor {floor}; choose a valid point")
        endpoints.append(xy)
    path, distance = scene.get_shortest_path(floor, *endpoints, entire_path=True)
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or not len(path) or not np.isfinite(path).all():
        raise ValueError("Scene returned an empty or invalid path")
    if not np.isfinite(distance) or distance < 0:
        raise ValueError("Scene returned an invalid geodesic distance")
    # Official world_to_map accepts a single XY point, not a batch.
    path_rc = [np.asarray(scene.world_to_map(xy)).tolist() for xy in path]
    return np.asarray(maps[floor]).copy(), {
        "scene_id": config["scene_id"], "floor_num": floor,
        "initial_pos": list(config["initial_pos"]), "target_pos": list(config["target_pos"]),
        "endpoint_map_rc": [np.asarray(scene.world_to_map(xy)).tolist() for xy in endpoints],
        "path_world_xy": path.tolist(), "path_map_rc": path_rc,
        "geodesic_distance_m": float(distance),
        "trav_map_resolution": float(scene.trav_map_resolution),
        "coordinate_note": "Map coordinates are [row, column]; endpoints are quantized to cells. No extra endpoint segments added.",
    }


def draw_path(floor_map, result):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(floor_map, cmap="gray", vmin=0, vmax=255, origin="upper", interpolation="nearest")
    route = np.asarray(result["path_map_rc"])
    endpoints = np.asarray(result["endpoint_map_rc"])
    ax.plot(route[:, 1], route[:, 0], color="dodgerblue", linewidth=1.8, label="Scene shortest path")
    for rc, color, marker, label in zip(endpoints, ["limegreen", "orangered"], ["o", "*"], ["Start (cell)", "Goal (cell)"]):
        ax.scatter(rc[1], rc[0], c=color, marker=marker, s=120, edgecolors="black", zorder=4, label=label)
    ax.set(title=f"{result['scene_id']} | floor {result['floor_num']} | {result['geodesic_distance_m']:.3f} m",
           xlabel="Map column (pixels)", ylabel="Map row (pixels)")
    ax.set_aspect("equal")
    ax.legend(loc="upper right")
    fig.tight_layout()
    return fig


def save_and_show(floor_map, result, output_dir, show=True):
    import matplotlib.pyplot as plt

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{result['scene_id']}_floor{result['floor_num']}_path"
    # A scene identifier must not turn the output basename into a path.
    stem = "".join(c if c.isalnum() or c in "_-" else "_" for c in stem)
    png = (output_dir / (stem + ".png")).resolve()
    data = output_dir / (stem + ".json")
    fig = draw_path(floor_map, result)
    try:
        fig.savefig(png, dpi=180)
        data.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Path image: {png}\nPath data: {data.resolve()}")
        if show:
            # Windows can display the saved image even when Matplotlib uses Agg.
            import matplotlib
            try:
                from matplotlib.backends import BackendFilter, backend_registry
                interactive_backends = backend_registry.list_builtin(BackendFilter.INTERACTIVE)
            except ImportError:  # Older Matplotlib versions used with iGibson.
                from matplotlib import rcsetup
                interactive_backends = rcsetup.interactive_bk
            interactive = str(matplotlib.get_backend()).lower() in {b.lower() for b in interactive_backends}
            if interactive:
                plt.show(block=True)
            elif os.name == "nt":
                os.startfile(str(png))
            else:
                raise RuntimeError(f"No interactive Matplotlib backend. Image saved to {png}; run on a desktop with Tk/Qt, or use --no-show")
    finally:
        plt.close(fig)
    return png


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("fixed_point_path.yaml"))
    parser.add_argument("--no-show", action="store_true", help="Save PNG/JSON without opening a window")
    args = parser.parse_args(argv)
    config_path = args.config.resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict) or config.get("scene") != "gibson" or config.get("build_graph") is not True:
        raise ValueError("This entry requires scene: gibson and build_graph: true")
    resolution = float(config.get("trav_map_resolution", 0.1))
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("trav_map_resolution must be positive and finite")
    config["waypoint_resolution"] = resolution
    try:
        from igibson.envs.igibson_env import iGibsonEnv
        from igibson.utils.assets_utils import download_assets, download_demo_data
    except ModuleNotFoundError as exc:
        raise RuntimeError("Run this script in an environment with iGibson and its dependencies installed") from exc
    download_assets()
    if config["scene_id"] == "Rs":
        download_demo_data()
    env = iGibsonEnv(config_file=config, mode="headless")
    try:
        # The loaded scene already owns the graph. No reset/step or task planning:
        # reset would plan before we have checked the configured endpoints.
        floor_map, result = plan_path(env.scene, config)
    finally:
        env.close()
    output_dir = Path(config.get("output_dir", "outputs/path_planning"))
    if not output_dir.is_absolute():
        output_dir = config_path.parent / output_dir
    return save_and_show(floor_map, result, output_dir, show=not args.no_show)


if __name__ == "__main__":
    main()
