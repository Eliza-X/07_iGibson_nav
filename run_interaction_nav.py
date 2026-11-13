"""Autonomous known-map door navigation. Run in the working iGibson environment."""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import tempfile
from uuid import uuid4

import numpy as np
import yaml

from interaction_map import InteractionMap
from interaction_scene import ROOT, load_config as load_scene_config, run
from interaction_task import InteractionTask


def load_config(path, scenario=None):
    path = Path(path).resolve()
    with path.open(encoding='utf-8') as stream:
        nav = yaml.safe_load(stream)
    config = load_scene_config(path.parent / nav['scene_config'])
    config.update(nav)
    if scenario is not None:
        config['scenario'] = scenario
    if config['scenario'] not in ('closed', 'open', 'jammed', 'blocked'):
        raise ValueError('Unknown scenario')
    positive = ('map_resolution', 'robot_radius', 'safety_margin', 'interaction_distance',
                'passage_angle', 'waypoint_tolerance', 'goal_tolerance', 'forward_action_limit',
                'crossing_action_limit', 'settle_time', 'stop_speed', 'stop_yaw_rate',
                'stop_hold_time', 'stop_timeout', 'door_timeout', 'navigation_timeout', 'task_timeout')
    for name in positive:
        if isinstance(config[name], bool) or not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError(name + ' must be finite and positive')
    if not config['passage_angle'] <= config['door_open_angle']:
        raise ValueError('Passage angle exceeds door target')
    if config['angular_action_sign'] not in (-1, 1):
        raise ValueError('angular_action_sign must be -1 or 1')
    if max(config['forward_action_limit'], config['crossing_action_limit']) > 1:
        raise ValueError('Action limits are normalized and must not exceed 1')
    for name in ('door_wait_point', 'door_exit_point'):
        value = np.asarray(config[name], float)
        if value.shape != (2,) or not np.isfinite(value).all():
            raise ValueError(name + ' must be finite XY')
    if config['goal_tolerance'] >= config['interaction_distance']:
        raise ValueError('Goal tolerance must be smaller than interaction distance')
    clearance = config['robot_radius'] + config['safety_margin'] + config['goal_tolerance'] + 0.1
    if config['door_wait_point'][1] >= -clearance or config['door_exit_point'][1] <= clearance:
        raise ValueError('Door waypoints must clear the partition with the entire robot')
    if config['scenario'] == 'blocked':
        config['obstacles'] = list(config['obstacles']) + [{'center': [0.0, 0.35], 'size': [0.9, 0.4]}]
    config['log_dir'] = str((path.parent / config['log_dir']).resolve())
    # Validate obstacle geometry before starting the simulator.
    InteractionMap(config['map_resolution'], config['robot_radius'], config['safety_margin'], config['obstacles'])
    return config


def obstacle_contacts(physics, robot_bodies, room_body, floor_bodies=()):
    """Unique external contacts, excluding floor support and robot self contact."""
    contacts = set()
    for body in robot_bodies:
        for point in physics.getContactPoints(bodyA=body):
            other, link = point[2], point[4]
            if other in robot_bodies:
                continue
            floor_support = (other in floor_bodies or (other == room_body and link == -1 and
                             point[6][2] <= 0.025 and point[7][2] > 0.9)
                             )
            if not floor_support and point[8] <= 0:
                contacts.add((body, point[3], other, link))
    return contacts


class NavigationEpisode:
    def __init__(self, config):
        self.config = config
        self.task = self.log = self.temp = None
        self.trace = []
        self.path_length = 0.0
        self.collision_events = 0
        self.last_contacts = set()
        self.previous_angle = None
        self.last_motion_time = 0.0
        self.door_motion = 'IDLE'
        self.summary = None

    def record(self, item):
        self.log.write(json.dumps(item, ensure_ascii=False) + '\n')
        self.log.flush()

    def begin(self, sim, robot, door, physics):
        self.sim, self.robot, self.door, self.p = sim, robot, door, physics
        c = self.config
        from igibson.objects.articulated_object import ArticulatedObject
        self.temp = tempfile.TemporaryDirectory(prefix='interaction_obstacles_')
        self.objects = []
        for i, obstacle in enumerate(c['obstacles']):
            x, y = obstacle['center']
            sx, sy = obstacle['size']
            path = Path(self.temp.name) / f'obstacle_{i}.urdf'
            path.write_text(f'''<robot name="obstacle_{i}"><link name="base">
<inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial>
<visual><origin xyz="{x} {y} 0.3"/><geometry><box size="{sx} {sy} 0.6"/></geometry>
<material name="red"><color rgba="0.8 0.1 0.1 1"/></material></visual>
<collision><origin xyz="{x} {y} 0.3"/><geometry><box size="{sx} {sy} 0.6"/></geometry></collision>
</link></robot>''', encoding='utf-8')
            obj = ArticulatedObject(str(path))
            sim.import_object(obj)
            self.objects.append(obj)
        if c['scenario'] == 'open':
            physics.resetJointState(door.body, door.joint, c['door_open_angle'], targetVelocity=0)
            door.open_door()  # Initial fixture; not a task interaction request.
        sim.sync(force_sync=True)
        self.map = InteractionMap(c['map_resolution'], c['robot_radius'], c['safety_margin'], c['obstacles'])
        self.task = InteractionTask(c, self.map, door)
        folder = Path(c['log_dir'])
        folder.mkdir(parents=True, exist_ok=True)
        self.log_path = folder / (datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid4().hex[:8] + '.jsonl')
        self.log = self.log_path.open('x', encoding='utf-8')
        self.record({'type': 'start', 'config': c, 'action_units': 'normalized differential drive commands'})
        print('Interaction log:', self.log_path)

    def pose(self):
        return [*map(float, self.robot.get_position()[:2]), float(self.robot.get_rpy()[2])]

    def before_step(self):
        self.pre_pose = self.pose()
        self.pre_state = self.task.state
        speed = float(np.linalg.norm(self.robot.get_linear_velocity()[:2]))
        yaw_rate = float(self.robot.get_angular_velocity()[2])
        self.action = self.task.tick(self.pre_pose, speed, yaw_rate, self.sim.render_timestep)
        if self.config['scenario'] == 'jammed':
            # Deliberate actuator failure fixture: hold closed despite requests.
            self.p.resetJointState(self.door.body, self.door.joint, 0, targetVelocity=0)
            self.p.setJointMotorControl2(self.door.body, self.door.joint, self.p.POSITION_CONTROL,
                                        targetPosition=0, force=self.config['door_force'])
        self.robot.apply_action(self.action)

    def after_step(self):
        pose = self.pose()
        self.path_length += float(np.linalg.norm(np.array(pose[:2]) - self.pre_pose[:2]))
        self.trace.append(pose[:2])
        contacts = obstacle_contacts(self.p, self.robot.get_body_ids(), self.door.body,
                                     self.sim.scene.floor_body_ids)
        self.collision_events += len(contacts - self.last_contacts)
        self.last_contacts = contacts
        if contacts:
            self.task.fail('obstacle_collision')
        if self.task.done:
            self.robot.apply_action(np.zeros(2))
        angle = self.door.angle
        if self.previous_angle is None or abs(angle - self.previous_angle) > 0.0001:
            self.last_motion_time = self.task.elapsed
        self.previous_angle = angle
        self.door_motion = ('AT_TARGET' if abs(angle - self.door.target) < 0.03 else
                            'STALLED' if self.task.elapsed - self.last_motion_time > 2 else 'MOVING_OR_PENDING')
        self.record({'type': 'step', 'time': self.task.elapsed, 'state_before': self.pre_state,
                     'state': self.task.state, 'pose_before': self.pre_pose, 'pose': pose,
                     'action': self.action.tolist(), 'door_angle': angle,
                     'door_motion': self.door_motion, 'passage_ready': self.task.passage,
                     'collision_contacts': [list(v) for v in sorted(contacts)]})
        if self.task.state != self.pre_state:
            print(f'{self.task.elapsed:.2f}s: {self.pre_state} -> {self.task.state}', self.task.reason or '')

    def labels(self):
        return [f'Task: {self.task.state} | t={self.task.elapsed:.1f}s',
                f'Door: {self.door_motion} | angle={math.degrees(self.door.angle):.1f} / {math.degrees(self.door.target):.1f} deg',
                f'Passage: {"READY" if self.task.passage else "NOT_READY"} | requests={self.task.requests}',
                f'Result: {self.task.reason or "running"}']

    def draw(self, frame, cv2):
        canvas = np.repeat((self.map.free.astype(np.uint8) * 220)[..., None], 3, axis=2)
        def pixel(xy):
            return tuple(reversed(self.map.world_to_map(xy)))
        if len(self.task.route) > 1:
            cv2.polylines(canvas, [np.array([pixel(p) for p in self.task.route], np.int32)], False, (255, 100, 0), 1)
        if len(self.trace) > 1:
            cv2.polylines(canvas, [np.array([pixel(p) for p in self.trace], np.int32)], False, (0, 180, 0), 1)
        cv2.circle(canvas, pixel(self.config['target_position'][:2]), 3, (0, 0, 255), -1)
        if self.trace:
            cv2.circle(canvas, pixel(self.trace[-1]), 3, (0, 120, 255), -1)
        width = min(320, frame.shape[1] // 2)
        height = int(width * canvas.shape[0] / canvas.shape[1])
        height = min(height, frame.shape[0] // 2)
        canvas = cv2.resize(np.flipud(canvas), (width, height), interpolation=cv2.INTER_NEAREST)
        frame[-height:, -width:] = canvas

    def finish(self):
        try:
            if self.task is not None and self.log is not None:
                if not self.task.done:
                    self.task.fail('interrupted_or_step_limit')
                self.robot.apply_action(np.zeros(2))
                self.summary = {'type': 'summary', 'success': self.task.state == 'SUCCESS',
                                'reason': self.task.reason, 'simulation_time': self.task.elapsed,
                                'path_length_m': self.path_length, 'open_requests': self.task.requests,
                                'door_wait_time': self.task.wait_time, 'collision_events': self.collision_events,
                                'transitions': self.task.transitions, 'final_pose': self.pose()}
                self.record(self.summary)
                self.log_path.with_suffix('.summary.json').write_text(
                    json.dumps(self.summary, ensure_ascii=False, indent=2), encoding='utf-8')
                print('Result:', self.summary)
        finally:
            if self.log is not None:
                self.log.close()
            if self.temp is not None:
                self.temp.cleanup()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'interaction_nav.yaml')
    parser.add_argument('--scenario', choices=('closed', 'open', 'jammed', 'blocked'))
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--steps', type=int, help='Optional smoke-test step limit')
    args = parser.parse_args(argv)
    if args.steps is not None and args.steps <= 0:
        parser.error('--steps must be positive')
    config = load_config(args.config, args.scenario)
    episode = NavigationEpisode(config)
    try:
        run(config, args.steps, args.headless, episode=episode)
    except ModuleNotFoundError as exc:
        raise SystemExit(f'Missing dependency: {exc.name}. Use the working iGibson environment.') from exc
    return episode.summary


if __name__ == '__main__':
    main()
