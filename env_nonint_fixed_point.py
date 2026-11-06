"""
执行固定点的导航任务
yaml文件读取这个文件夹中的yaml文件
"""

import argparse
import logging
import os
from sys import platform

import numpy as np
import yaml

from action_policy import compute_action
from closed_loop_controller import run_closed_loop
from astar_planner import plan_path


def update_follow_camera(env, back=1.5, side=0.6, height=1.2, look_height=0.25):
    """Place ExternalView behind/left/above the robot (offsets in metres).

    Positive side is the robot's left; negative side selects its right.
    Following overrides manual camera movement on the next simulation step.
    """
    if env.mode == "headless":
        return
    viewer = getattr(getattr(env, "simulator", None), "viewer", None)
    if viewer is None:
        return
    robot = env.robots[0]
    position = np.asarray(robot.get_position(), dtype=float)
    yaw = float(robot.get_rpy()[2])
    forward = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    left = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
    camera = position - back * forward + side * left
    camera[2] += height
    direction = position + np.array([0.0, 0.0, look_height]) - camera
    length = np.linalg.norm(direction)
    if not np.isfinite(length) or length <= 0:
        raise ValueError("Follow camera must have a finite, nonzero viewing direction")
    direction /= length
    viewer.px, viewer.py, viewer.pz = map(float, camera)
    viewer.view_direction = direction
    viewer.theta = float(np.arctan2(direction[1], direction[0]))
    viewer.phi = float(np.arctan2(direction[2], np.linalg.norm(direction[:2])))


def execution_waypoints(scene, result, config):
    """Keep graph nodes; allow endpoint connectors only inside one free cell.

    This is a navigation-map check, not a robot-footprint collision guarantee.
    """
    points = np.asarray(result["path_world_xy"], dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Planner must return a nonempty finite XY path")
    floor = config["floor_num"]
    for name, node in (("initial_pos", points[0]), ("target_pos", points[-1])):
        endpoint = np.asarray(config[name], dtype=float)[:2]
        if (not scene.has_node(floor, endpoint)
                or not scene.has_node(floor, node)
                or not np.array_equal(scene.world_to_map(endpoint), scene.world_to_map(node))
                or np.linalg.norm(endpoint - node) > np.sqrt(2) * scene.trav_map_resolution + 1e-8):
            raise ValueError(f"{name} cannot be safely connected within its traversable map cell")
    waypoints = points.tolist()
    target = list(config["target_pos"][:2])
    if not np.array_equal(points[-1], target):
        waypoints.append(target)
    return waypoints


def resolve_display_mode(config, override=None):
    """Explicit argument wins over YAML; configs without a mode use classic."""
    mode = override if override is not None else config.get("display_mode", "classic")
    if mode not in ("video", "classic"):
        raise ValueError("display_mode must be video or classic")
    return mode


# 程序主函数
def main(selection="user", headless=False, short_exec=False, follow_camera=True, show_route_map=True,
         show_map_waypoints=True, display_mode=None):
    """
    Plan and execute one fixed-point TurtleBot episode in the same Gibson scene.
    headless disables the GUI; short_exec limits the environment and controller to 10 steps.
    follow_camera keeps ExternalView behind the robot; disable for manual viewing.
    show_route_map opens a live map and keeps the final trajectory until closed.
    show_map_waypoints toggles waypoint dots on that map only.
    headless disables both camera following and the route-map window.
    display_mode overrides YAML: video records four views; classic uses original windows.
    In video mode headless suppresses preview but still records the video.
    """
    print("*" * 80 + "\nDescription:" + main.__doc__ + "*" * 80)
    # 读取此脚本所在目录中的yaml文件
    config_filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fixed_point_path.yaml")
    print("实际读取的配置文件：", config_filename)
    with open(config_filename, "r", encoding="utf-8") as config_file:
        config_data = yaml.safe_load(config_file)
    if not isinstance(config_data, dict) or config_data.get("scene") != "gibson" or config_data.get("build_graph") is not True:
        raise ValueError("Execution requires scene: gibson and build_graph: true")
    floor = config_data.get("floor_num", 0)
    if isinstance(floor, bool) or not isinstance(floor, int) or floor != 0:
        raise ValueError("The fixed-point execution task currently supports floor_num: 0 only")
    if config_data.get("task") != "point_nav_fixed":
        raise ValueError("Execution requires task: point_nav_fixed")
    resolution = float(config_data.get("trav_map_resolution", 0.1))
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("trav_map_resolution must be finite and positive")
    config_data["floor_num"] = floor
    config_data["waypoint_resolution"] = resolution
    config_data.setdefault("waypoint_tolerance", min(0.05, resolution / 2))
    config_data.setdefault("turn_direction", "left")
    config_data.setdefault("waypoint_log_dir", "logs")
    config_data["visible_waypoints"] = not headless
    config_data["visible_target"] = not headless
    config_data["visible_path"] = False
    # Planning's 500-step budget is too short for dense, slow waypoint execution.
    config_data["max_step"] = 10 if short_exec else int(config_data.get("execution_max_step", 5000))
    if config_data["max_step"] <= 0:
        raise ValueError("execution_max_step must be positive")
    mode = resolve_display_mode(config_data, display_mode)
    quad_enabled = mode == "video"
    # Derive the recorder's internal flag solely from the selected mode.
    config_data["display_mode"] = mode
    config_data["record_video"] = quad_enabled
    print(f"Display mode: {mode} | recording: {quad_enabled} | preview: {not headless}")
    if quad_enabled:
        config_data["image_width"] = int(config_data.get("video_panel_width", 640))
        config_data["image_height"] = int(config_data.get("video_panel_height", 480))

    from igibson.envs.igibson_env import iGibsonEnv
    from igibson.utils.assets_utils import download_assets, download_demo_data

    download_assets()
    if config_data.get("scene") == "gibson" and config_data.get("scene_id") == "Rs":
        download_demo_data()
    # Reduce texture scale for Mac.
    if platform == "darwin":
        config_data["texture_scale"] = 0.5

    # Shadows and PBR do not make much sense for a Gibson static mesh
    config_data["enable_shadow"] = False
    config_data["enable_pbr"] = False

    # Quad mode renders its own three cameras and owns one preview window.
    env = iGibsonEnv(config_file=config_data,
                     mode="gui_interactive" if not headless and not quad_enabled else "headless")

    route_map = None
    recorder = None
    # 闭环控制器
    try:
        # Validate and plan before reset: task reset itself may query the graph.
        floor_map, result = plan_path(env.scene, config_data)
        if (env.task.floor_num != floor
                or not np.allclose(env.task.initial_pos, config_data["initial_pos"], atol=1e-8, rtol=0)
                or not np.allclose(env.task.target_pos, config_data["target_pos"], atol=1e-8, rtol=0)):
            raise ValueError("Loaded task floor/start/goal differs from the planning configuration")
        controller_config = dict(config_data)
        controller_config["waypoints"] = execution_waypoints(env.scene, result, config_data)
        print(f"Planned distance: {result['geodesic_distance_m']:.3f} m; "
              f"graph nodes: {len(result['path_world_xy'])}; "
              f"execution waypoints: {len(controller_config['waypoints'])}; "
              f"step budget: {controller_config['max_step']}")
        if quad_enabled:
            try:
                from quad_view_recorder import QuadViewRecorder
            except ModuleNotFoundError as exc:
                if exc.name == "cv2":
                    raise RuntimeError("Quad view requires OpenCV: install opencv-python in the iGibson environment") from exc
                raise
            recorder = QuadViewRecorder(
                env, floor_map, result, controller_config["waypoints"], config_data,
                preview=not headless, show_waypoints=show_map_waypoints,
            )
        elif show_route_map and not headless:
            from live_path_visualization import LivePathVisualization
            route_map = LivePathVisualization(
                env.scene, floor_map, result, waypoints=controller_config["waypoints"],
                show_waypoints=show_map_waypoints,
            )

        def update_observers(current_env):
            if recorder is not None:
                recorder.update(current_env)
                return
            if follow_camera and not headless:
                update_follow_camera(current_env)
            if route_map is not None:
                route_map.update(current_env)

        # run_closed_loop owns the only reset and reads the actual pose each step.
        summary = run_closed_loop(
            env=env, config_data=controller_config, action_fn=compute_action,
            observation_callback=update_observers,
        )
        if route_map is not None:
            route_map.finish(env, summary)
        if recorder is not None:
            recorder.finish(env, summary)
            summary["video_path"] = str(recorder.video_path) if recorder.video_path else None
    except BaseException:
        if route_map is not None:
            route_map.close()
        raise
    finally:
        try:
            if recorder is not None:
                recorder.close()
        finally:
            env.close()
    if route_map is not None:
        try:
            route_map.wait_until_closed()
        finally:
            route_map.close()
    return summary

def cli(argv=None):
    parser = argparse.ArgumentParser(description="Run navigation with classic windows or four-view video recording.")
    parser.add_argument("--display-mode", choices=("video", "classic"), default=None,
                        help="Override display_mode in fixed_point_path.yaml")
    parser.add_argument("--headless", action="store_true", help="Disable windows; video mode still records")
    parser.add_argument("--short-exec", action="store_true", help="Limit execution to 10 steps")
    parser.add_argument("--no-follow-camera", action="store_true", help="Disable camera following in classic mode")
    parser.add_argument("--no-route-map", action="store_true", help="Hide the separate map in classic mode")
    parser.add_argument("--no-map-waypoints", action="store_true", help="Hide waypoint dots on the map")
    args = parser.parse_args(argv)
    return main(display_mode=args.display_mode, headless=args.headless, short_exec=args.short_exec,
                follow_camera=not args.no_follow_camera, show_route_map=not args.no_route_map,
                show_map_waypoints=not args.no_map_waypoints)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    cli()
