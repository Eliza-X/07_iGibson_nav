from types import SimpleNamespace

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
import pytest

import live_path_visualization as live


class Scene:
    trav_map_size = 12
    trav_map_resolution = 1.0

    def world_to_map(self, xy):
        return (np.asarray(xy)[::-1] / self.trav_map_resolution + self.trav_map_size / 2).astype(int)


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
        np.testing.assert_allclose(view.robot.get_data(), [[7], [8]])
        np.testing.assert_allclose(arrows[-1], [[7, 8], [7, 8.4]])
        position[:] = [2., 3., 0.]
        clock[0] = 0.01
        view.update(env)
        assert len(view.history) == 2  # record samples even between redraws
        np.testing.assert_allclose(view.robot.get_data(), [[7], [8]])
        view.finish(env, {"accepted": False, "reason": "controller_step_limit", "steps": 2})
        np.testing.assert_allclose(view.robot.get_data(), [[8], [9]])
        np.testing.assert_allclose(view.trail.get_data(), [[7, 8], [8, 9]])
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


@pytest.mark.parametrize("size,resolution", [(100, 0.1), (40, 0.25)])
def test_float_map_aligns_with_integer_cells(size, resolution):
    scene = Scene()
    scene.trav_map_size = size
    scene.trav_map_resolution = resolution
    np.testing.assert_allclose(live.world_to_map_float(scene, [0, 0]), [size / 2, size / 2])
    # Exact binary-representable offsets avoid integer truncation roundoff.
    point = [resolution * 5, -resolution * 5]
    np.testing.assert_allclose(live.world_to_map_float(scene, point), scene.world_to_map(point))


def test_diagonal_subcell_motion_is_preserved(desktop_canvas):
    view = make_map()
    position = np.array([0.01, 0.03, 0.0])
    robot = SimpleNamespace(get_position=lambda: position, get_rpy=lambda: [0, 0, np.pi / 4])
    env = SimpleNamespace(robots=[robot])
    samples = [[0.01, 0.03], [0.04, 0.06], [0.08, 0.10]]
    try:
        # These positions all fall in one integer cell in the test scene.
        assert len({tuple(view.scene.world_to_map(xy)) for xy in samples}) == 1
        for xy in samples:
            position[:2] = xy
            view.update(env, force=True)
        np.testing.assert_allclose(view.history, samples)
        np.testing.assert_allclose(view.trail.get_data(), [[6.01, 6.04, 6.08], [6.03, 6.06, 6.10]])
        np.testing.assert_allclose(view.robot.get_data(), [[6.08], [6.10]])
        delta = np.diff(np.asarray(view.trail.get_data()), axis=1)
        np.testing.assert_allclose(delta[0], delta[1])
    finally:
        view.close()
