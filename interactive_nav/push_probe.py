"""Straight pushing with measured progress, contact and settling feedback."""
import json
import math
import time
from pathlib import Path
import numpy as np


def validate_config(c):
    for key, value in c.items():
        if key == 'box_size':
            if len(value) != 3 or any(not math.isfinite(v) or v <= 0 for v in value):
                raise ValueError('box_size must contain three positive dimensions')
        elif key == 'angular_action_sign':
            if value not in (-1, 1):
                raise ValueError('angular_action_sign must be -1 or 1')
        elif not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be positive and finite')
    if max(c['approach_action'], c['push_action']) > 1:
        raise ValueError('Normalized actions must not exceed 1')


class PushController:
    def __init__(self, config):
        self.c = config
        self.state = 'SETTLE'
        self.reason = ''
        self.elapsed = self.entered = self.still = self.lost = 0.
        self.reference = None
        self.progress = self.best = self.last_progress = 0.

    def transition(self, state):
        self.state = state
        self.entered = self.elapsed
        self.still = 0.

    def fail(self, reason):
        self.reason = reason
        self.transition('FAILED')

    def tick(self, s):
        c = self.c
        self.elapsed += c['dt']
        zero = np.zeros(2)
        if self.state in ('FAILED', 'SUCCESS'):
            return zero
        if s['unexpected_contacts']:
            self.fail('unexpected_collision')
        elif max(abs(v) for v in s['box_rpy'][:2]+s['robot_rpy'][:2]) > c['max_tilt']:
            self.fail('tilted')
        elif self.elapsed >= c['max_seconds']:
            self.fail('total_timeout')
        if self.state == 'FAILED':
            return zero
        stable = max(s['robot_speed'], s['box_speed']) < c['stop_speed'] and max(s['robot_angular_speed'], s['box_angular_speed']) < c['stop_angular_speed']
        self.still = self.still+c['dt'] if stable else 0.
        if self.reference is not None:
            self.progress = s['box_position'][0]-self.reference[0]
            yaw = math.atan2(math.sin(s['box_rpy'][2]-self.reference[2]), math.cos(s['box_rpy'][2]-self.reference[2]))
            if abs(s['box_position'][1]-self.reference[1]) > c['max_lateral'] or abs(yaw) > c['max_yaw']:
                self.fail('box_drift')
                return zero
        if self.state == 'SETTLE':
            if self.elapsed >= c['settle_seconds'] and self.still >= c['stable_seconds']:
                self.reference = (*s['box_position'][:2], s['box_rpy'][2])
                self.transition('APPROACH')
            elif self.elapsed > c['settle_timeout']:
                self.fail('settle_timeout')
            return zero
        if self.state == 'APPROACH':
            if s['contacts']:
                self.transition('PUSH')
                self.last_progress = self.elapsed
            elif self.elapsed-self.entered > c['approach_timeout']:
                self.fail('no_contact')
        elif self.state == 'PUSH':
            self.lost = 0. if s['contacts'] else self.lost+c['dt']
            if self.progress >= c['push_distance']:
                self.transition('STOP')
            elif self.lost >= c['contact_loss_seconds']:
                self.fail('contact_lost')
            elif self.elapsed-self.entered > c['push_timeout']:
                self.fail('push_timeout')
            elif self.progress >= self.best+c['progress_epsilon']:
                self.best = self.progress
                self.last_progress = self.elapsed
            elif self.elapsed-self.last_progress >= c['stall_seconds']:
                self.fail('stalled')
        elif self.state == 'STOP':
            if self.still >= c['stable_seconds']:
                self.transition('VERIFY')
            elif self.elapsed-self.entered > c['stop_timeout']:
                self.fail('stop_timeout')
        elif self.state == 'VERIFY':
            if not stable:
                self.transition('STOP')
            elif self.progress >= c['push_distance']:
                self.reason = 'success'
                self.transition('SUCCESS')
            else:
                self.fail('insufficient_final_displacement')
        if self.state not in ('APPROACH', 'PUSH'):
            return zero
        yaw = math.atan2(math.sin(s['robot_rpy'][2]), math.cos(s['robot_rpy'][2]))
        return np.array([c['approach_action'] if self.state == 'APPROACH' else c['push_action'],
                         np.clip(-c['heading_gain']*yaw*c['angular_action_sign'], -.1, .1)])


def box_urdf(c):
    x, y, z = c['box_size']; m = c['box_mass']
    return f'''<robot name="push_box"><link name="box">
    <inertial><mass value="{m}"/><inertia ixx="{m*(y*y+z*z)/12}" iyy="{m*(x*x+z*z)/12}" izz="{m*(x*x+y*y)/12}" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><geometry><box size="{x} {y} {z}"/></geometry><material name="orange"><color rgba="0.9 0.45 0.1 1"/></material></visual>
    <collision><geometry><box size="{x} {y} {z}"/></geometry></collision>
    </link></robot>'''


def run_probe(c, mode, output, playback_speed=0.5):
    import pybullet as p
    from igibson.envs.igibson_env import iGibsonEnv
    from igibson.objects.articulated_object import ArticulatedObject
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    native = dict(scene='empty', robot=dict(name='Turtlebot', action_type='continuous', action_normalize=True),
                  output=['task_obs'], task_obs_dim=0, initial_pos_z_offset=.1,
                  image_width=1280, image_height=800)
    # Own the display so native viewer camera updates cannot override our view.
    env = iGibsonEnv(config_file=native, mode='headless', action_timestep=c['dt'], automatic_reset=False)
    view = None
    try:
        env.reset()
        robot = env.robots[0]
        env.land(robot, [0, 0, .1], [0, 0, 0])
        bodies = robot.get_body_ids()
        bounds = [p.getAABB(b, link) for b in bodies for link in range(-1, p.getNumJoints(b))]
        floor_ids = {p.getBodyUniqueId(i) for i in range(p.getNumBodies())}-set(bodies)
        path = output/'box.urdf'; path.write_text(box_urdf(c), encoding='utf-8')
        obj = ArticulatedObject(str(path)); env.simulator.import_object(obj)
        box = obj.get_body_ids()[0]
        p.resetBasePositionAndOrientation(box, [max(a[1][0] for a in bounds)+c['initial_gap']+c['box_size'][0]/2, 0, c['box_size'][2]/2+.01], [0, 0, 0, 1])
        p.changeDynamics(box, -1, lateralFriction=c['box_friction'], restitution=0)
        if p.getDynamicsInfo(box, -1)[0] <= 0:
            raise RuntimeError('Box must be a dynamic body')
        (output/'geometry.json').write_text(json.dumps(dict(robot_aabbs=bounds, box_aabb=p.getAABB(box), floor_ids=list(floor_ids)), indent=2), encoding='utf-8')

        def observe():
            pos, quat = p.getBasePositionAndOrientation(box)
            linear, angular = p.getBaseVelocity(box)
            contacts, unexpected = [], []
            for body in bodies:
                for point in p.getContactPoints(bodyA=body):
                    if point[8] > 0 or point[2] in bodies:
                        continue
                    entry = dict(robot_link=point[3], other_body=point[2], other_link=point[4], normal_force=point[9])
                    if point[2] == box:
                        contacts.append(entry)
                    elif point[2] not in floor_ids:
                        unexpected.append(entry)
            return dict(box_position=list(pos), box_rpy=list(p.getEulerFromQuaternion(quat)),
                        robot_position=list(map(float, robot.get_position())), robot_rpy=list(map(float, robot.get_rpy())),
                        robot_speed=float(np.linalg.norm(robot.get_linear_velocity())),
                        robot_angular_speed=float(np.linalg.norm(robot.get_angular_velocity())),
                        box_speed=float(np.linalg.norm(linear)), box_angular_speed=float(np.linalg.norm(angular)),
                        contacts=contacts, unexpected_contacts=unexpected)

        task = PushController(c)
        if mode != 'headless':
            from .push_view import PushView
            view = PushView(env, observe(), c['push_distance'], playback_speed)
            view.show(task, observe())
            view.wait()
        with (output/'episode.jsonl').open('w', encoding='utf-8') as log:
            while task.state not in ('FAILED', 'SUCCESS'):
                if view:
                    view.wait()
                    if view.closed:
                        task.fail('user_closed')
                        break
                started = time.perf_counter()
                before = observe()
                action = task.tick(before)
                env.step(np.clip(action, env.action_space.low, env.action_space.high).astype(np.float32))
                log.write(json.dumps(dict(time=task.elapsed, state=task.state, action=action.tolist(), observation=before))+'\n')
                if view:
                    view.show(task, observe())
                    if task.state not in ('FAILED', 'SUCCESS'):
                        view.wait(started+c['dt']/playback_speed)
            result = dict(success=task.state == 'SUCCESS', reason=task.reason, simulated_seconds=task.elapsed,
                          displacement=task.progress, final_observation=observe())
            log.write(json.dumps(dict(type='summary', **result))+'\n')
        (output/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        if view and not view.closed:
            print(f"Result: {task.reason}. Close the window to continue. Logs: {output}")
            view.paused = False
            view.show(task, result['final_observation'])
            view.wait(final=True)
        return result
    finally:
        if view:
            view.close()
        env.close()
