import json

import matplotlib
matplotlib.use("Agg")
import numpy as np
import pytest

from plot_fixed_point_path import draw_path, plan_path, save_and_show


class Scene:
    build_graph = True
    waypoint_interval = 1
    trav_map_resolution = 1.0
    floor_map = [np.full((8, 8), 255)]

    def __init__(self):
        self.calls = []

    def world_to_map(self, xy):
        return np.asarray(xy)[::-1].astype(int)

    def has_node(self, floor, xy):
        return bool(np.all(np.asarray(xy) >= 0) and np.all(np.asarray(xy) < 8))

    def get_shortest_path(self, floor, source, target, entire_path=False):
        self.calls.append((floor, source.copy(), target.copy(), entire_path))
        return np.array([[1, 2], [2, 2], [2, 3]]), 2.0


def config():
    return dict(scene_id="test", floor_num=0, initial_pos=[1, 2, 0], target_pos=[2, 3, 0])


def test_scene_call_and_plot_axes():
    scene = Scene()
    floor_map, result = plan_path(scene, config())
    assert scene.calls[0][3] is True
    np.testing.assert_equal(scene.calls[0][1], [1, 2])
    assert result["path_map_rc"] == [[2, 1], [2, 2], [3, 2]]
    fig = draw_path(floor_map, result)
    try:
        line = fig.axes[0].lines[0]
        np.testing.assert_equal(line.get_xdata(), [1, 2, 2])
        np.testing.assert_equal(line.get_ydata(), [2, 2, 3])
        assert fig.axes[0].images[0].origin == "upper"
    finally:
        import matplotlib.pyplot as plt
        plt.close(fig)


@pytest.mark.parametrize("change", [{"initial_pos": [-1, 2]}, {"target_pos": [np.nan, 1]},
                                      {"floor_num": -1}, {"floor_num": True}])
def test_bad_inputs_do_not_call_planner(change):
    scene = Scene()
    with pytest.raises(ValueError):
        plan_path(scene, {**config(), **change})
    assert not scene.calls


def test_dense_path_required():
    scene = Scene()
    scene.waypoint_interval = 2
    with pytest.raises(ValueError, match="waypoint_resolution"):
        plan_path(scene, config())


def test_save_without_window(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    monkeypatch.setattr(plt, "show", lambda **kw: pytest.fail("Unexpected window"))
    floor_map, result = plan_path(Scene(), config())
    png = save_and_show(floor_map, result, tmp_path, show=False)
    assert png.read_bytes().startswith(b"\x89PNG")
    assert json.loads(png.with_suffix(".json").read_text())["geodesic_distance_m"] == 2.0


def test_default_opens_blocking_window_after_save(tmp_path, monkeypatch):
    import matplotlib.pyplot as plt
    monkeypatch.setattr(matplotlib, "get_backend", lambda: "TkAgg")
    calls = []
    def show(**kwargs):
        assert (tmp_path / "test_floor0_path.png").exists()
        assert (tmp_path / "test_floor0_path.json").exists()
        calls.append(kwargs)
    monkeypatch.setattr(plt, "show", show)
    floor_map, result = plan_path(Scene(), config())
    save_and_show(floor_map, result, tmp_path)
    assert calls == [{"block": True}]
