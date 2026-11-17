"""One box: navigate, physically push, release, replan, navigate and stop."""
import math
import numpy as np
from action_policy import compute_action
from waypoint_tracker import WaypointTracker, compute_target_error
from .push_probe import PushController
from .push_planning import BoxMap


class PushNavigation:
    def __init__(self, c):
        self.c = c
        self.state, self.reason = 'SETTLE', ''
        self.elapsed = self.entered = self.still = self.progress = 0.
        self.reference = None
        self.tracker = None
        self.push = None
        self.replans = 0
        self.path_available = None
        self.plans = []

    def transition(self, state):
        self.state, self.entered, self.still = state, self.elapsed, 0.
        self.tracker = None

    def fail(self, reason):
        self.reason = reason
        self.transition('FAILED')

    def diagnostics(self):
        return dict(path_available=self.path_available, replans=self.replans, plans=self.plans)

    def tick(self, s):
        c = self.c
        self.elapsed += c['dt']
        zero = np.zeros(2)
        if self.state in ('FAILED', 'SUCCESS'):
            return zero
        pose = (*s['robot_position'][:2], s['robot_rpy'][2])
        yaw = math.atan2(math.sin(pose[2]), math.cos(pose[2]))
        if self.reference is not None:
            self.progress = s['box_position'][0]-self.reference[0]
        if s['unexpected_contacts'] or s.get('box_wall_contacts'):
            self.fail('wall_or_unexpected_collision')
        elif max(abs(v) for v in s['box_rpy'][:2]+s['robot_rpy'][:2]) > c['max_tilt']:
            self.fail('tilted')
        elif self.elapsed > c['max_seconds']:
            self.fail('total_timeout')
        elif abs(pose[0]) > 4 or abs(pose[1]) > 3:
            self.fail('out_of_bounds')
        elif s['contacts'] and self.state not in ('INTERACT', 'RELEASE', 'RELEASE_STOP'):
            self.fail('unexpected_box_contact')
        if self.state == 'FAILED':
            return zero
        stable = (max(s['robot_speed'], s['box_speed']) < c['stop_speed'] and
                  max(s['robot_angular_speed'], s['box_angular_speed']) < c['stop_angular_speed'])
        self.still = self.still+c['dt'] if stable else 0.
        nav = BoxMap(c, s['box_position'], s['box_rpy'][2])
        if self.state == 'SETTLE':
            if self.elapsed >= c['settle_seconds'] and self.still >= c['stable_seconds']:
                self.reference = (*s['box_position'][:2], s['box_rpy'][2])
                try:
                    nav.path(pose, c['goal'])
                except ValueError:
                    self.transition('APPROACH')
                else:
                    self.fail('initial_route_not_blocked')
            elif self.elapsed > c['settle_timeout']:
                self.fail('settle_timeout')
            return zero
        if self.state in ('APPROACH', 'NAVIGATE'):
            target = c['prepush'] if self.state == 'APPROACH' else c['goal']
            if self.elapsed-self.entered > c['navigation_timeout']:
                self.fail('navigation_timeout')
                return zero
            if self.tracker is None:
                try:
                    path = nav.path(pose, target)
                except ValueError:
                    self.path_available = False
                    self.fail('no_path')
                    return zero
                self.plans.append(dict(stage=self.state, time=self.elapsed, path=path))
                self.tracker = WaypointTracker(path, target, c['waypoint_tolerance'], c['goal_tolerance'])
            self.tracker.update(pose)
            if self.tracker.completed:
                self.transition('ALIGN' if self.state == 'APPROACH' else 'FINAL_STOP')
                return zero
            if not nav.segment_clear(pose[:2], self.tracker.current_waypoint):
                if self.state == 'NAVIGATE' and self.replans < c['max_replans']:
                    self.transition('REPLAN')
                else:
                    self.fail('route_obstructed')
                return zero
            distance, angle = compute_target_error(pose, self.tracker.current_waypoint)
            action = compute_action(distance, angle, self.tracker.current_tolerance, 'shortest')
            action[1] *= c['angular_action_sign']
            return action
        if self.state == 'ALIGN':
            if self.elapsed-self.entered > c['stop_timeout']:
                self.fail('align_timeout')
            elif abs(yaw) > c['align_tolerance']:
                return np.array([0., np.clip(-.8*yaw*c['angular_action_sign'], -.2, .2)])
            elif self.still >= c['stable_seconds']:
                self.push = PushController(c)
                self.push.reference = self.reference
                self.push.transition('APPROACH')
                self.transition('INTERACT')
            return zero
        if self.state == 'INTERACT':
            action = self.push.tick(s)
            if self.push.state == 'FAILED':
                self.fail(self.push.reason)
                return zero
            if self.push.state == 'SUCCESS':
                self.transition('RELEASE')
                return zero
            return action
        if self.state == 'RELEASE':
            target = np.array([c['release_point'][0], pose[1]])
            wall_map = BoxMap(c, s['box_position'], s['box_rpy'][2], include_box=False)
            # The only exception to box inflation is straight withdrawal from
            # existing contact. Walls remain inflated throughout this motion.
            if abs(yaw) > .12 or not wall_map.segment_clear(pose[:2], target):
                self.fail('unsafe_release')
            elif pose[0] <= target[0]:
                if s['contacts'] or not nav.segment_clear(pose[:2], pose[:2]):
                    self.fail('release_not_clear')
                else:
                    self.transition('RELEASE_STOP')
            elif self.elapsed-self.entered > c['release_timeout']:
                self.fail('release_timeout')
            else:
                return np.array([-c['release_action'], 0.])
            return zero
        if self.state in ('RELEASE_STOP', 'REPLAN', 'FINAL_STOP'):
            if self.elapsed-self.entered > c['stop_timeout']:
                self.fail('stop_timeout')
                return zero
            if self.still < c['stable_seconds']:
                return zero
            if self.state == 'RELEASE_STOP':
                self.transition('REPLAN')
            elif self.state == 'REPLAN':
                if self.replans >= c['max_replans']:
                    self.fail('replan_limit')
                    return zero
                try:
                    path = nav.path(pose, c['goal'])
                except ValueError:
                    self.path_available = False
                    self.fail('no_path_after_push')
                    return zero
                self.replans += 1
                self.path_available = True
                self.plans.append(dict(stage='REPLAN', time=self.elapsed, path=path))
                self.transition('NAVIGATE')
                self.tracker = WaypointTracker(path, c['goal'], c['waypoint_tolerance'], c['goal_tolerance'])
            elif np.linalg.norm(np.array(pose[:2])-c['goal']) < c['goal_tolerance']:
                self.reason = 'success'
                self.transition('SUCCESS')
            else:
                self.transition('REPLAN')
            return zero
        self.fail('unknown_state')
        return zero
