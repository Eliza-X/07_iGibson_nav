"""Simulation-time task state machine; no renderer or physics imports."""

import numpy as np

from action_policy import compute_action
from waypoint_tracker import WaypointTracker, compute_target_error


class InteractionTask:
    def __init__(self, config, navigation_map, door):
        self.config, self.map, self.door = config, navigation_map, door
        self.map.update(door.angle)
        self.state, self.reason = 'SETTLING', None
        self.elapsed = self.entered = self.wait_time = 0.0
        self.still_time = 0.0
        self.requests = 0
        self.tracker = None
        self.route = []
        self.passage = False
        self.transitions = []

    @property
    def done(self):
        return self.state in ('SUCCESS', 'FAILED')

    def transition(self, state):
        self.state, self.entered = state, self.elapsed
        self.transitions.append({'time': self.elapsed, 'state': state})

    def fail(self, reason):
        # Post-step collision checks take precedence over a pre-step success.
        if self.state != 'FAILED':
            self.reason = reason
            self.transition('FAILED')

    def navigate(self, pose, goal, state):
        try:
            self.route = self.map.plan(pose[:2], goal)
            self.tracker = WaypointTracker(self.route, goal,
                                           self.config['waypoint_tolerance'], self.config['goal_tolerance'])
            self.transition(state)
        except ValueError as exc:
            self.fail('planning_failed: ' + str(exc))

    def tick(self, pose, speed, yaw_rate, dt):
        zero = np.zeros(2, dtype=np.float32)
        if self.done:
            return zero
        self.elapsed += dt
        self.map.update(self.door.angle)
        c = self.config
        near, far = c['door_wait_point'], c['door_exit_point']
        angle_ready = self.door.angle >= c['passage_angle']
        self.passage = angle_ready and self.map.segment_clear(near, far)
        if self.elapsed >= c['task_timeout']:
            self.fail('task_timeout')
            return zero
        if self.state == 'SETTLING':
            if self.elapsed >= c['settle_time']:
                self.navigate(pose, near, 'APPROACH')
            return zero
        if self.state == 'STOPPING':
            if np.linalg.norm(np.asarray(pose[:2]) - near) > c['interaction_distance']:
                self.fail('outside_interaction_range')
            elif speed <= c['stop_speed'] and abs(yaw_rate) <= c['stop_yaw_rate']:
                self.still_time += dt
                if self.still_time >= c['stop_hold_time']:
                    if not angle_ready:
                        self.door.open_door()
                        self.requests += 1
                    self.transition('WAIT_FOR_DOOR')
            else:
                self.still_time = 0.0
            if self.elapsed - self.entered > c['stop_timeout']:
                self.fail('stop_timeout')
            return zero
        if self.state == 'WAIT_FOR_DOOR':
            self.wait_time += dt
            if self.passage:
                self.navigate(pose, far, 'CROSSING')
            elif self.elapsed - self.entered >= c['door_timeout']:
                self.fail('passage_blocked' if angle_ready else 'door_open_timeout')
            return zero
        if self.state == 'FINAL_STOP':
            if np.linalg.norm(np.asarray(pose[:2]) - c['target_position'][:2]) >= c['goal_tolerance']:
                self.navigate(pose, c['target_position'][:2], 'TO_GOAL')
            elif speed <= c['stop_speed'] and abs(yaw_rate) <= c['stop_yaw_rate']:
                self.still_time += dt
                if self.still_time >= c['stop_hold_time']:
                    self.reason = 'goal_reached'
                    self.transition('SUCCESS')
            else:
                self.still_time = 0.0
            return zero
        if self.elapsed - self.entered >= c['navigation_timeout']:
            self.fail('navigation_timeout')
            return zero
        if self.state == 'CROSSING' and not self.passage:
            self.fail('passage_lost_during_crossing')
            return zero
        self.tracker.update(pose)
        if self.tracker.completed:
            if self.state == 'APPROACH':
                self.still_time = 0.0
                self.transition('STOPPING')
            elif self.state == 'CROSSING':
                if pose[1] - c['robot_radius'] <= 0.1:
                    self.fail('robot_not_fully_across')
                else:
                    self.navigate(pose, c['target_position'][:2], 'TO_GOAL')
            else:
                self.still_time = 0.0
                self.transition('FINAL_STOP')
            return zero
        target = self.tracker.current_waypoint
        if not self.map.segment_clear(pose[:2], target):
            self.fail('route_obstructed')
            return zero
        distance, angle = compute_target_error(pose, target)
        action = compute_action(distance, angle, self.tracker.current_tolerance, 'shortest')
        action[0] = min(action[0], c['crossing_action_limit'] if self.state == 'CROSSING' else c['forward_action_limit'])
        action[1] *= c['angular_action_sign']
        return action
