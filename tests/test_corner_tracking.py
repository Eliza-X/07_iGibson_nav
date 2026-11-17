"""Regression for the real 2026-11-17 corner-cutting failure."""
from pathlib import Path
import numpy as np
from waypoint_tracker import WaypointTracker, corner_indices
from interactive_nav.push_planning import BoxMap
from interactive_nav.push_navigation import PushNavigation
from run_push_navigation import load_config


def test_recorded_failure_keeps_corner_instead_of_cutting_to_d():
    c = load_config(Path(__file__).parents[1]/'push_navigation.yaml')
    pose = (.8121239765941538, .5544526427326019, .952093974098062)
    path = [list(pose[:2]), [.8, .55], [.8, .6], [.85, .6], [.9, .6]]
    nav = BoxMap(c, [1.3039495773678718, -.02270069159825088], -.10059298727681794)
    old = WaypointTracker(path, path[-1], .06, .01)
    old.update(pose)
    assert np.allclose(old.current_waypoint, [.9, .6])
    assert not nav.segment_clear(pose[:2], old.current_waypoint)
    t = WaypointTracker(path, path[-1], .06, .01, corner_indices(path))
    # A connector corner may require a one-cycle pause, then B is retained.
    for _ in range(2):
        t.update(pose, nav.segment_clear)
    assert np.allclose(t.current_waypoint, [.8, .6])
    assert nav.segment_clear(pose[:2], t.current_waypoint)
    assert t.index in t.critical_indices


def test_corner_completion_stops_same_cycle_skipping():
    path = [[0, 0], [1, 0], [1, .05], [1, 1]]
    t = WaypointTracker(path, path[-1], .06, .1, corner_indices(path))
    t.index = 1
    assert t.update((1, 0, 0), lambda a, b: True) == [1]
    assert t.corner_pause and t.index == 2


def test_unsafe_switch_refines_and_does_not_stop_at_normal_tolerance():
    path = [[0, 0], [1, 0], [1, 1]]
    t = WaypointTracker(path, path[-1], .06, .1, {1})
    t.index = 1
    t.update((.98, 0, 0), lambda a, b: b[1] == 0)
    assert t.index == 1 and t.refining
    assert t.current_tolerance == .005
    assert not t.blocked_at_precision
    t.update((.999, 0, 0), lambda a, b: b[1] == 0)
    assert t.blocked_at_precision


def test_near_duplicate_connectors_and_straight_segments():
    assert corner_indices([[0, 0], [1e-8, 0], [1, 0], [1, 1]]) == {2}
    path = [[0, 0], [.05, 0], [.1, 0], [.2, 0]]
    t = WaypointTracker(path, path[-1], .06, .01, corner_indices(path))
    t.update((.049, 0, 0), lambda a, b: True)
    assert t.index == 3 and not t.corner_pause


def test_navigation_controller_moves_toward_logged_safe_corner():
    c = load_config(Path(__file__).parents[1]/'push_navigation.yaml')
    t = PushNavigation(c)
    t.state = 'NAVIGATE'
    s = dict(robot_position=[.8121239765941538, .5544526427326019, .01],
             robot_rpy=[0, 0, .952093974098062], box_position=[1.3039495773678718, -.02270069159825088, .175],
             box_rpy=[0, 0, -.10059298727681794], robot_speed=0., box_speed=0.,
             robot_angular_speed=0., box_angular_speed=0., contacts=[], unexpected_contacts=[], box_wall_contacts=[])
    nav = BoxMap(c, s['box_position'], s['box_rpy'][2])
    t.tracker = t.make_tracker(nav.path(s['robot_position'], c['goal']), c['goal'])
    for _ in range(2):
        action = t.tick(s)
    assert t.state == 'NAVIGATE' and t.replans == 0
    assert np.allclose(t.tracker.current_waypoint, [.8, .6])
    assert np.any(action != 0)
    assert t.tracking['obstruction'] is None
