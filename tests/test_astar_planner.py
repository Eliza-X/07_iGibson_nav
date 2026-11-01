import numpy as np
import pytest

from astar_planner import astar_grid, plan_path
from env_nonint_fixed_point import execution_waypoints
from waypoint_tracker import WaypointTracker


@pytest.mark.parametrize("connectivity,expected", [(4, 7), (8, 3 * np.sqrt(2) + 1)])
def test_open_grid_optimal_cost_and_dense_nodes(connectivity, expected):
    path, cost = astar_grid(np.ones((5, 6), bool), (0, 0), (3, 4), connectivity)
    assert cost == pytest.approx(expected)
    assert path[0] == (0, 0) and path[-1] == (3, 4)
    steps = np.abs(np.diff(path, axis=0))
    assert (steps.max(axis=1) == 1).all()
    if connectivity == 4:
        assert (steps.sum(axis=1) == 1).all()


def test_wall_detour_and_unreachable():
    free = np.ones((5, 5), bool)
    free[:4, 2] = False
    path, cost = astar_grid(free, (0, 0), (0, 4), 4)
    assert cost == 12
    assert all(free[node] for node in path)
    free[4, 2] = False
    with pytest.raises(ValueError, match="No traversable path"):
        astar_grid(free, (0, 0), (0, 4))


def test_corner_cutting_switch():
    free = np.eye(2, dtype=bool)
    with pytest.raises(ValueError, match="No traversable path"):
        astar_grid(free, (0, 0), (1, 1))
    path, cost = astar_grid(free, (0, 0), (1, 1), allow_corner_cutting=True)
    assert path == [(0, 0), (1, 1)] and cost == pytest.approx(np.sqrt(2))
    # Even one blocked side prevents a diagonal shortcut.
    free[0, 1] = True
    assert astar_grid(free, (0, 0), (1, 1))[1] == 2


def test_same_cell_and_invalid_endpoints():
    assert astar_grid(np.ones((2, 2)), (0, 0), (0, 0)) == ([(0, 0)], 0)
    for start in [(-1, 0), (2, 0)]:
        with pytest.raises(ValueError, match="outside"):
            astar_grid(np.ones((2, 2)), start, (0, 0))
    with pytest.raises(ValueError, match="outside"):
        astar_grid(np.zeros((2, 2)), (0, 0), (0, 0))


@pytest.mark.parametrize("kwargs", [{"connectivity": 6}, {"connectivity": True},
                                   {"allow_corner_cutting": "false"}])
def test_invalid_options(kwargs):
    with pytest.raises(ValueError):
        astar_grid(np.ones((2, 2)), (0, 0), (1, 1), **kwargs)


def test_metric_output_precise_endpoint_and_tracker_compatibility():
    class Scene:
        floor_map = [np.full((5, 5), 255)]
        trav_map_resolution = 0.1

        def world_to_map(self, xy):
            return np.floor(np.asarray(xy)[::-1] / 0.1 + 1e-9).astype(int)

        def map_to_world(self, rc):
            return np.asarray(rc)[::-1] * 0.1

        def has_node(self, floor, xy):
            rc = self.world_to_map(xy)
            return bool(np.all(rc >= 0) and np.all(rc < 5))

        def get_shortest_path(self, *args, **kwargs):
            pytest.fail("New planner must not call scene shortest path")

    scene = Scene()
    config = dict(scene_id="test", floor_num=0, initial_pos=[0.01, 0.01, 0], target_pos=[0.32, 0.32, 0])
    floor_map, result = plan_path(scene, config)
    assert result["geodesic_distance_m"] == pytest.approx(0.3 * np.sqrt(2))
    assert result["path_map_rc"] == [[0, 0], [1, 1], [2, 2], [3, 3]]
    points = execution_waypoints(scene, result, config)
    assert points[-1] == [0.32, 0.32]
    tracker = WaypointTracker(points, config["target_pos"], 0.05, 0.1)
    for xy in points:
        tracker.update([*xy, 0])
    assert tracker.completed
    floor_map[:] = 0
    assert (scene.floor_map[0] == 255).all()
