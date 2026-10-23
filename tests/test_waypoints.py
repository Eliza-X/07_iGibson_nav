import json
from types import SimpleNamespace

import numpy as np
import pytest

from action_policy import compute_action
from closed_loop_controller import run_closed_loop
from waypoint_tracker import WaypointTracker, compute_target_error


@pytest.mark.parametrize("points", [[], None, [[1]], [[1, 2, 3]], [[np.nan, 0]], [[np.inf, 0]], [[2, 0]]])
def test_invalid_points(points):
    with pytest.raises(ValueError):
        WaypointTracker(points, [1, 0, 0], 0.15, 0.1)


@pytest.mark.parametrize("tol", [0, -1, np.inf, np.nan])
def test_invalid_tolerance(tol):
    with pytest.raises(ValueError):
        WaypointTracker([[1, 0]], [1, 0], tol, 0.1)


def test_duplicates_reset_and_completion():
    tracker = WaypointTracker([[0, 0], [0, 0], [1, 0]], [1, 0], 0.15, 0.1)
    assert tracker.update([0, 0, 0]) == [0, 1]
    assert tracker.current_tolerance == 0.1
    assert tracker.update([1, 0, 0]) == [2]
    assert tracker.completed and tracker.current_waypoint is None
    assert tracker.update([1, 0, 0]) == []
    tracker.reset()
    assert tracker.index == 0 and not tracker.completed


def test_strict_boundary_and_fallback():
    tracker = WaypointTracker.from_config({}, [0, 0, 0], 0.1)
    assert tracker.update([0.1, 0, 0]) == []
    assert tracker.update([0.099, 0, 0]) == [0]


@pytest.mark.parametrize("yaw,target,expected", [
    (0, [0, 1], np.pi / 2), (0, [0, -1], -np.pi / 2),
    (np.pi - 0.01, [-1, -0.01], 0.01 + np.arctan(0.01)),
    (-np.pi + 0.01, [-1, 0.01], -0.01 - np.arctan(0.01)),
])
def test_bearing_wrap(yaw, target, expected):
    distance, angle = compute_target_error([0, 0, yaw], target)
    assert distance == pytest.approx(np.linalg.norm(target))
    assert angle == pytest.approx(expected)


def test_policy_preserves_fixed_turn_and_stop():
    assert compute_action(1, -1, 0.1, "left")[1] > 0
    assert compute_action(1, 1, 0.1, "right")[1] < 0
    np.testing.assert_array_equal(compute_action(0.01, 1, 0.1, "left"), [0, 0])
    np.testing.assert_allclose(compute_action(1, 0, 0.1, "left"), [0.2, 0])


class FakeTask:
    def __init__(self, target):
        self.target_pos = np.array([*target, 0.0])
        self.dist_tol = 0.1


class FakeEnv:
    mode = "headless"

    def __init__(self, target, positions=None, failure=False):
        self.task = FakeTask(target)
        self.positions = positions
        self.failure = failure
        self.action_space = SimpleNamespace(shape=(2,), low=np.array([-1, -1]), high=np.array([1, 1]))
        self.robots = [self]
        self.reset()

    def reset(self):
        self.pose = np.zeros(3)
        self.steps = 0
        return {}

    def get_position(self):
        return np.array([*self.pose[:2], 0])

    def get_rpy(self):
        return [0, 0, self.pose[2]]

    def step(self, action):
        if self.positions is not None:
            self.pose = np.array(self.positions[min(self.steps, len(self.positions) - 1)], dtype=float)
        else:
            # Ideal kinematics only: not iGibson's normalized action mapping.
            self.pose[:2] += float(action[0]) * 0.1 * np.array([np.cos(self.pose[2]), np.sin(self.pose[2])])
            self.pose[2] += float(action[1]) * 0.1
        self.steps += 1
        success = np.linalg.norm(self.pose[:2] - self.task.target_pos[:2]) < self.task.dist_tol
        return {}, 0, bool(success or self.failure), {"success": bool(success)}


def run(env, tmp_path, points=None, action_fn=compute_action, **extra):
    config = {"max_step": 5000, "waypoint_log_dir": str(tmp_path), **extra}
    if points is not None:
        config["waypoints"] = points
    result = run_closed_loop(env, config, action_fn)
    rows = [json.loads(line) for line in open(result["log_path"], encoding="utf-8")]
    return result, rows


def test_switch_uses_new_target_and_terminal_step_updates_tracker(tmp_path):
    calls = []

    def policy(distance, angle, tolerance, direction):
        calls.append((distance, angle, tolerance))
        return [0, 0]

    env = FakeEnv([1, 1], [[1, 0, 0], [1, 1, 0]])
    result, rows = run(env, tmp_path, [[1, 0], [1, 1]], policy)
    assert calls == pytest.approx([(1, 0, 0.15), (1, np.pi / 2, 0.1)])
    assert result["accepted"] and result["completed_waypoints"] == 2
    assert rows[2]["reached_after"] == [1] and rows[2]["done"]
    np.testing.assert_array_equal(env.task.target_pos, [1, 1, 0])


def test_early_goal_is_not_accepted(tmp_path):
    result, _ = run(FakeEnv([1, 1], [[1, 1, 0]]), tmp_path, [[1, 0], [1, 1]])
    assert result["reason"] == "early_final_goal"
    assert not result["accepted"] and not result["sequence_completed"]


def test_failure_and_step_limit(tmp_path):
    result, _ = run(FakeEnv([1, 0], [[0, 0, 0]], failure=True), tmp_path)
    assert result["reason"] == "environment_done" and not result["accepted"]
    result, _ = run(FakeEnv([1, 0], [[0, 0, 0]]), tmp_path, max_step=2)
    assert result["reason"] == "controller_step_limit"


def test_start_already_complete_sends_zero_action(tmp_path):
    def unexpected_policy(*args):
        raise AssertionError("completed tracker must not invoke policy")
    result, rows = run(FakeEnv([0, 0]), tmp_path, action_fn=unexpected_policy)
    assert result["accepted"] and rows[1]["action"] == [0, 0]


@pytest.mark.parametrize("points", [None, [[0.5, 0], [1, 0]], [[1, 0], [1, 1]],
                                     [[1, 0], [1, 1], [2, 1]]])
@pytest.mark.parametrize("direction", ["left", "right"])
def test_ideal_kinematic_routes(tmp_path, points, direction):
    target = [1, 0] if points is None else points[-1]
    result, rows = run(FakeEnv(target), tmp_path, points, turn_direction=direction)
    assert result["accepted"]
    reached = [i for row in rows if row["type"] == "step"
               for i in row["reached_before"] + row["reached_after"]]
    assert reached == list(range(1 if points is None else len(points)))
