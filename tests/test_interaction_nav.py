import math

import numpy as np
import pytest

from interaction_map import InteractionMap
from interaction_scene import ROOT
from interaction_task import InteractionTask
from run_interaction_nav import load_config, obstacle_contacts


class Door:
    def __init__(self, angle=0):
        self.angle = angle
        self.target = angle
        self.calls = 0

    def open_door(self):
        self.target = math.pi / 2
        self.calls += 1


def setup(scenario='closed'):
    c = load_config(ROOT / 'interaction_nav.yaml', scenario)
    m = InteractionMap(c['map_resolution'], c['robot_radius'], c['safety_margin'], c['obstacles'])
    door = Door(math.pi / 2 if scenario == 'open' else 0)
    return c, m, door, InteractionTask(c, m, door)


def test_closed_disconnected_open_connected_and_full_width_clearance():
    c, m, _, _ = setup()
    start, goal = c['robot_position'][:2], c['target_position'][:2]
    with pytest.raises(ValueError, match='No traversable path'):
        m.plan(start, goal)
    m.update(math.pi / 2)
    path = m.plan(start, goal)
    assert path[-1] == goal
    assert all(m.segment_clear(a, b) for a, b in zip([start] + path, path))
    # Offset obstacle leaves x=0 center ray free but blocks the robot's width.
    narrow = InteractionMap(obstacles=[{'center': [0.22, 0.4], 'size': [0.1, 0.2]}])
    narrow.update(math.pi / 2)
    assert not narrow.segment_clear(c['door_wait_point'], c['door_exit_point'])
    for xy in (start, goal, [0, 0]):
        np.testing.assert_allclose(m.map_to_world(m.world_to_map(xy)), xy, atol=m.resolution / 2)


@pytest.mark.parametrize('scenario,expected,requests', [
    ('closed', 'goal_reached', 1), ('open', 'goal_reached', 0),
    ('jammed', 'door_open_timeout', 1), ('blocked', 'passage_blocked', 1)])
def test_complete_task_with_kinematic_robot(scenario, expected, requests):
    # Logical closed-loop test; deliberately not an iGibson physics acceptance test.
    c, m, door, task = setup(scenario)
    pose = np.array([*c['robot_position'][:2], c['robot_yaw']], float)
    speed = yaw_rate = 0.0
    dt = 0.1
    for _ in range(5000):
        action = task.tick(pose, speed, yaw_rate, dt)
        if door.calls:
            assert np.linalg.norm(pose[:2] - c['door_wait_point']) < c['interaction_distance'] or task.state in ('CROSSING', 'TO_GOAL', 'FINAL_STOP', 'SUCCESS')
        if task.state in ('WAIT_FOR_DOOR', 'STOPPING'):
            np.testing.assert_array_equal(action, [0, 0])
        if scenario != 'jammed':
            door.angle = min(door.target, door.angle + c['door_speed'] * dt)
        speed, yaw_rate = action
        pose[2] += yaw_rate * dt
        pose[:2] += speed * np.array([np.cos(pose[2]), np.sin(pose[2])]) * dt
        if task.done:
            break
    assert task.reason == expected
    assert task.requests == door.calls == requests
    assert task.state == ('SUCCESS' if expected == 'goal_reached' else 'FAILED')
    if task.state == 'SUCCESS':
        assert np.linalg.norm(pose[:2] - c['target_position'][:2]) < c['goal_tolerance']
        assert pose[1] - c['robot_radius'] > 0.1


def test_lost_passage_stops_immediately():
    c, m, door, task = setup('open')
    pose = np.array([*c['door_wait_point'], math.pi / 2])
    task.navigate(pose, c['door_exit_point'], 'CROSSING')
    door.angle = 0
    np.testing.assert_array_equal(task.tick(pose, 0.1, 0, 0.1), [0, 0])
    assert task.reason == 'passage_lost_during_crossing'


def test_no_interaction_until_measured_stop():
    c, m, door, task = setup()
    task.transition('STOPPING')
    pose = [*c['door_wait_point'], 0]
    for _ in range(10):
        task.tick(pose, 0.2, 0, 0.1)
    assert door.calls == 0
    for _ in range(5):
        task.tick(pose, 0, 0, 0.1)
    assert door.calls == 1


def test_floor_support_excluded_but_wall_and_door_contacts_counted():
    def contact(other, link, z, normal):
        return (0, 5, other, 1, link, (0, 0, z), (0, 0, z), normal, -0.001)

    class Physics:
        def getContactPoints(self, **kwargs):
            return [contact(1, -1, 0, (0, 0, 1)), contact(1, -1, 0.2, (1, 0, 0)),
                    contact(1, 0, 0.2, (0, 1, 0)), contact(5, 2, 0.2, (0, 0, 1)),
                    contact(9, -1, 0, (0, 0, 1))]

    assert obstacle_contacts(Physics(), [5], 1, [9]) == {(5, 1, 1, -1), (5, 1, 1, 0)}


def test_post_step_collision_overrides_success():
    _, _, _, task = setup()
    task.transition('SUCCESS')
    task.fail('obstacle_collision')
    assert task.state == 'FAILED'
    assert task.reason == 'obstacle_collision'
