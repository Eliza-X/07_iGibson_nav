"""Closed-loop waypoint execution; the environment owns task termination."""

import inspect
import json
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import numpy as np

from waypoint_tracker import WaypointTracker, compute_target_error
from waypoint_visualization import WaypointVisualization


def read_robot_pose(env):
    position = env.robots[0].get_position()
    return [float(position[0]), float(position[1]), float(env.robots[0].get_rpy()[2])]


def run_closed_loop(env, config_data, action_fn, observation_callback=None):
    """Run one episode; action_fn takes distance, angle, tolerance, turn direction.

    JSONL rows pair pre-action pose/error/action with post-action pose/events.
    The caller owns env.close(). Task targets and rewards remain unchanged.
    Optional observation_callback(env) runs after reset and each step, including
    the terminal step; it may update viewer state for the next viewer refresh.
    """
    env.reset()
    if observation_callback is not None:
        observation_callback(env)
    if env.action_space.shape != (2,):
        raise ValueError("TurtleBot requires a two-dimensional continuous action space")
    dist_tol = float(env.task.dist_tol)
    tracker = WaypointTracker.from_config(config_data, env.task.target_pos, dist_tol)
    max_steps = int(config_data.get("max_step", 500))
    if max_steps <= 0:
        raise ValueError("max_step must be positive")
    turn_direction = str(config_data.get("turn_direction", "left")).strip().lower()
    if turn_direction not in ("left", "right"):
        raise ValueError("turn_direction must be left or right")

    print("Task:", type(env.task).__name__, "source:", inspect.getfile(type(env.task)))
    print("Final target:", env.task.target_pos, "waypoints:", tracker.waypoints.tolist())
    log_dir = Path(config_data.get("waypoint_log_dir", "logs"))
    if not log_dir.is_absolute():
        log_dir = Path(__file__).resolve().parent / log_dir
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / ("waypoints_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                          + "_" + uuid4().hex + ".jsonl")
    visual = WaypointVisualization(env, tracker, config_data.get("visible_waypoints", False))
    summary = None
    try:
        with log_path.open("x", encoding="utf-8") as log:
            def record(data):
                log.write(json.dumps(data, ensure_ascii=False, default=str) + "\n")
                log.flush()

            record({"type": "start", "waypoints": tracker.waypoints.tolist(),
                    "target_pos": np.asarray(env.task.target_pos).tolist(),
                    "dist_tol": dist_tol, "waypoint_tolerance": tracker.waypoint_tolerance,
                    "turn_direction": turn_direction})
            for step in range(1, max_steps + 1):
                pose = read_robot_pose(env)
                reached_before = tracker.update(pose)
                visual.update(tracker)
                index = None if tracker.completed else tracker.index
                target = tracker.current_waypoint
                distance, angle = (0.0, 0.0) if tracker.completed else compute_target_error(pose, target)
                action = (np.zeros(2) if tracker.completed else
                          action_fn(distance, angle, tracker.current_tolerance, turn_direction))
                action = np.clip(action, env.action_space.low, env.action_space.high).astype(np.float32)
                _, reward, done, info = env.step(action)
                if observation_callback is not None:
                    observation_callback(env)
                post_pose = read_robot_pose(env)
                reached_after = tracker.update(post_pose)
                visual.update(tracker)
                record({"type": "step", "step": step, "pose_before": pose,
                        "waypoint_index": index, "waypoint": None if target is None else target.tolist(),
                        "distance": distance, "angle_error": angle, "action": action.tolist(),
                        "pose_after": post_pose, "reached_before": reached_before,
                        "reached_after": reached_after, "sequence_completed": tracker.completed,
                        "reward": float(reward), "done": bool(done), "info": info})
                if step % 10 == 0 or reached_before or reached_after or done:
                    print(f"step={step} waypoint={index} xy={post_pose[:2]} "
                          f"distance_before={distance:.3f} angle_before={np.degrees(angle):.1f} "
                          f"action={action} reached={reached_before + reached_after}")
                if done or step == max_steps:
                    environment_success = bool(info.get("success", False)) if done else False
                    reason = ("success" if environment_success and tracker.completed else
                              "early_final_goal" if environment_success else
                              "environment_done" if done else "controller_step_limit")
                    summary = {"type": "summary", "steps": step, "reason": reason,
                               "environment_success": environment_success,
                               "sequence_completed": tracker.completed,
                               "accepted": environment_success and tracker.completed,
                               "completed_waypoints": tracker.index, "final_pose": post_pose,
                               "info": info, "log_path": str(log_path)}
                    record(summary)
                    print("Episode result:", summary)
                    break
    finally:
        visual.hide()
    return summary
