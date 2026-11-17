"""One box: navigate, physically push, release, replan, navigate and stop."""
import math
import numpy as np
from action_policy import compute_action
from waypoint_tracker import WaypointTracker, compute_target_error, corner_indices
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
        self.tracking = None
        self.refine_index = None
        self.refine_since = 0.

    def make_tracker(self, path, target):
        c = self.c
        return WaypointTracker(path, target, c['waypoint_tolerance'], c['goal_tolerance'],
                               critical_indices=corner_indices(path),
                               corner_tolerance=c['corner_tolerance'], fine_tolerance=c['fine_tolerance'])

    def blocked(self, reason):
        if self.state == 'NAVIGATE' and self.replans < self.c['max_replans']:
            self.transition('REPLAN')
        else:
            self.fail(reason)

    def transition(self, state):
        self.state, self.entered, self.still = state, self.elapsed, 0.
        self.tracker = None
        self.refine_index = None

    def fail(self, reason):
        self.reason = reason
        self.transition('FAILED')

    def diagnostics(self):
        return dict(path_available=self.path_available, replans=self.replans, plans=self.plans)

    def tick(self, s):
        c = self.c
        self.tracking = None
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
                self.tracker = self.make_tracker(path, target)
            reached = self.tracker.update(pose, nav.segment_clear)
            if self.tracker.completed:
                self.transition('ALIGN' if self.state == 'APPROACH' else 'FINAL_STOP')
                return zero
            tracker = self.tracker
            obstruction = nav.first_obstruction(pose[:2], tracker.current_waypoint)
            candidate_obstruction = (nav.first_obstruction(pose[:2], tracker.blocked_target)
                                     if tracker.blocked_target is not None else None)
            distance, angle = compute_target_error(pose, tracker.current_waypoint)
            self.tracking = dict(index=tracker.index, target=tracker.current_waypoint.tolist(),
                                 critical=tracker.index in tracker.critical_indices, distance=distance,
                                 reached=reached, refining=tracker.refining, corner_pause=tracker.corner_pause,
                                 obstruction=obstruction, candidate_obstruction=candidate_obstruction,
                                 blocked_target=tracker.blocked_target.tolist() if tracker.blocked_target is not None else None)
            if tracker.refining:
                if self.refine_index != tracker.index:
                    self.refine_index, self.refine_since = tracker.index, self.elapsed
            else:
                self.refine_index = None
            if obstruction or tracker.blocked_at_precision or (tracker.refining and self.elapsed-self.refine_since > c['refine_timeout']):
                self.tracking['decision'] = 'stop_and_replan_or_fail'
                self.blocked('route_obstructed' if obstruction else 'corner_refinement_blocked')
                return zero
            if tracker.corner_pause:
                self.tracking['decision'] = 'corner_pause'
                return zero
            self.tracking['decision'] = 'refine' if tracker.refining else 'follow'
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
                self.tracker = self.make_tracker(path, c['goal'])
            elif np.linalg.norm(np.array(pose[:2])-c['goal']) < c['goal_tolerance']:
                self.reason = 'success'
                self.transition('SUCCESS')
            else:
                self.transition('REPLAN')
            return zero
        self.fail('unknown_state')
        return zero
