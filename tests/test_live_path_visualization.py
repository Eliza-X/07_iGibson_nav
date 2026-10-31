from types import SimpleNamespace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
import pytest

import live_path_visualization as live


class Scene:
    def world_to_map(self, xy):
        # Deliberately flip world Y to catch incorrect heading/map assumptions.
        return np.array([10 - xy[1], xy[0]])


def make_map():
    return live.LivePathVisualization(Scene(), np.full((12, 12), 255), {
        "path_map_rc": [[10, 0], [8, 2]], "endpoint_map_rc": [[10, 0], [8, 2]],
        "scene_id": "test", "floor_num": 0, "geodesic_distance_m": 3.0,
    })


@pytest.fixture
def desktop_canvas(monkeypatch):
    # Exercise artists/event calls without requiring a desktop in CI.
    monkeypatch.setattr(FigureCanvasAgg, "required_interactive_framework", "mock")
    monkeypatch.setattr(plt, "show", lambda **kwargs: None)


def test_noninteractive_backend_explains_fix_and_closes_figure():
    before = plt.get_fignums()
    with pytest.raises(RuntimeError, match="show_route_map=False"):
        make_map()
    assert plt.get_fignums() == before


def test_actual_pose_history_throttle_and_final_frame(desktop_canvas, monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(live, "monotonic", lambda: clock[0])
    position = [1., 2., 0.]
    robot = SimpleNamespace(get_position=lambda: position, get_rpy=lambda: [0, 0, np.pi / 2])
    env = SimpleNamespace(robots=[robot])
    view = make_map()
    try:
        arrows = []
        monkeypatch.setattr(view.heading, "set_positions", lambda a, b: arrows.append((a, b)))
        view.update(env)
        np.testing.assert_allclose(view.robot.get_data(), [[1], [8]])
        np.testing.assert_allclose(arrows[-1], [[1, 8], [1, 7.6]])
        position[:] = [2., 3., 0.]
        clock[0] = 0.01
        view.update(env)
        assert len(view.history) == 2  # record samples even between redraws
        np.testing.assert_allclose(view.robot.get_data(), [[1], [8]])
        view.finish(env, {"accepted": False, "reason": "controller_step_limit", "steps": 2})
        np.testing.assert_allclose(view.robot.get_data(), [[2], [7]])
        np.testing.assert_allclose(view.trail.get_data(), [[1, 2], [8, 7]])
        assert "NOT SUCCESSFUL" in view.ax.get_title()
        assert "controller_step_limit" in view.ax.get_title()
    finally:
        view.close()


def test_closed_window_stops_reading_pose_and_does_not_wait(desktop_canvas, monkeypatch):
    view = make_map()
    view.close()
    monkeypatch.setattr(plt, "show", lambda **kwargs: pytest.fail("Closed map reopened"))
    view.update(SimpleNamespace())
    view.wait_until_closed()
