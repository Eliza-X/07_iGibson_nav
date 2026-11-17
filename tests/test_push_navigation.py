from pathlib import Path
import numpy as np
import pytest
from run_push_navigation import load_config
from interactive_nav.push_planning import BoxMap, check_layout
from interactive_nav.push_navigation import PushNavigation


def config():
    return load_config(Path(__file__).parents[1]/'push_navigation.yaml')


def observation():
    return dict(robot_position=[-2.5, 0., .1], robot_rpy=[0., 0., 0.],
                box_position=[0., 0., .175], box_rpy=[0., 0., 0.],
                robot_speed=0., box_speed=0., robot_angular_speed=0., box_angular_speed=0.,
                contacts=[], unexpected_contacts=[], box_wall_contacts=[])


def test_layout_requires_interaction_and_retains_box_obstacle():
    c = config()
    assert check_layout(c)['initially_blocked']
    with pytest.raises(ValueError):
        BoxMap(c, [0, 0], 0).path(c['start'], c['goal'])
    nav = BoxMap(c, [1.3, 0], 0)
    path = nav.path(c['release_point'], c['goal'])
    assert not nav.segment_clear([1.3, 0], [1.3, 0])
    assert max(abs(p[1]) for p in path) > .5
    assert all(nav.segment_clear(a, b) for a, b in zip(path, path[1:]))


def test_entire_control_sequence_requires_release_and_goal_stop():
    c = config(); t = PushNavigation(c); s = observation()
    for _ in range(15):
        t.tick(s)
    assert t.state == 'APPROACH'
    for _ in range(100):
        t.tick(s)
        if t.tracker and not t.tracker.completed:
            s['robot_position'][:2] = list(t.tracker.current_waypoint)
        if t.state == 'INTERACT':
            break
    assert t.state == 'INTERACT'
    s['robot_position'][0] = -.4
    s['contacts'] = [{}]
    t.tick(s)
    for _ in range(14):
        s['box_position'][0] += .1
        s['robot_position'][0] += .1
        t.tick(s)
    for _ in range(15):
        t.tick(s)
        if t.state == 'RELEASE':
            break
    assert t.state == 'RELEASE'
    assert t.tick(s)[0] < 0
    s['contacts'] = []
    s['robot_position'][0] = c['release_point'][0]
    for _ in range(20):
        t.tick(s)
        if t.state == 'NAVIGATE':
            break
    assert t.state == 'NAVIGATE' and t.path_available
    for _ in range(300):
        t.tick(s)
        if t.tracker and not t.tracker.completed:
            s['robot_position'][:2] = list(t.tracker.current_waypoint)
        if t.state == 'FINAL_STOP':
            break
    assert t.state == 'FINAL_STOP'
    s['robot_speed'] = .1
    for _ in range(10):
        t.tick(s)
    assert t.state == 'FINAL_STOP'
    s['robot_speed'] = 0.
    for _ in range(10):
        t.tick(s)
    assert t.state == 'SUCCESS' and t.replans == 1


@pytest.mark.parametrize('kind', ['unexpected_contacts', 'box_wall_contacts'])
def test_wall_collision_stops_even_during_interaction(kind):
    t = PushNavigation(config()); s = observation()
    t.state = 'INTERACT'; s[kind] = [{}]
    assert not t.tick(s).any()
    assert t.reason == 'wall_or_unexpected_collision'


def test_box_contact_not_allowed_in_navigation():
    t = PushNavigation(config()); s = observation()
    t.state = 'NAVIGATE'; s['contacts'] = [{}]
    assert not t.tick(s).any()
    assert t.reason == 'unexpected_box_contact'


def test_no_route_after_push_and_replan_limit():
    t = PushNavigation(config()); s = observation()
    t.state = 'REPLAN'
    for _ in range(6):
        t.tick(s)
    assert t.reason == 'no_path_after_push'
    t = PushNavigation(config()); t.state = 'REPLAN'; t.replans = t.c['max_replans']
    for _ in range(6):
        t.tick(s)
    assert t.reason == 'replan_limit'


def test_release_cannot_ignore_wall():
    t = PushNavigation(config()); s = observation()
    t.state = 'RELEASE'; s['robot_position'][:2] = [.2, 1.]
    assert not t.tick(s).any()
    assert t.reason == 'unsafe_release'
