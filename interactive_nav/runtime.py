"""Native iGibson adapter. The runner alone advances the environment."""
import json
import math
from pathlib import Path
import numpy as np
from action_policy import compute_action
from waypoint_tracker import WaypointTracker, compute_target_error
from .planning import START, GOAL, CONTACT, Snapshot, choose_plan
from .scene import DoorScene

def create_env(config, mode, output):
    from igibson.envs.igibson_env import iGibsonEnv
    from igibson.tasks.task_base import BaseTask
    class DoorTask(BaseTask):
        def reset_scene(self, env):
            env.door_scene.set_open(config['initially_open'])
        def reset_agent(self, env):
            env.land(env.robots[0], [*START,.1], [0,0,0])
        def reset_variables(self, env):
            self.path_length = 0.; self.previous = np.array(START)
        def get_task_obs(self, env):
            return np.array(GOAL)-env.robots[0].get_position()[:2]
        def step(self, env):
            position = env.robots[0].get_position()[:2]
            self.path_length += float(np.linalg.norm(position-self.previous)); self.previous = position.copy()
        def get_termination(self, env, collision_links=None, action=None, info=None):
            info = {} if info is None else info
            position = env.robots[0].get_position()
            success = np.linalg.norm(position[:2]-GOAL) < config['goal_tolerance']
            reason = ('success' if success else 'collision' if collision_links else
                      'out_of_bounds' if abs(position[0])>4 or abs(position[1])>3 or position[2]<-.2 else
                      'timeout' if env.current_step >= config['max_steps'] else '')
            info.update(success=bool(success),reason=reason,path_length=self.path_length)
            return bool(reason),info
    class DoorEnv(iGibsonEnv):
        def load_task_setup(self):
            super().load_task_setup()  # Config omits task: initializes standard settings and DummyTask.
            self.task = DoorTask(self)
    native = dict(scene='empty', robot=dict(name='Turtlebot', action_type='continuous',action_normalize=True),
                  output=['task_obs'], task_obs_dim=2, initial_pos_z_offset=.1,
                  collision_ignore_link_a_ids=[1,2,3,4],
                  image_width=config.get('render_width',1280),image_height=config.get('render_height',960))
    from viewer_windows import resizable_viewer_windows, attach_viewer_display
    with resizable_viewer_windows(mode == 'gui_interactive'):
        env = DoorEnv(config_file=native,mode=mode,action_timestep=config['dt'],automatic_reset=False)
    try:
        if mode == 'gui_interactive':
            attach_viewer_display(env)
        env.door_scene = DoorScene(env,Path(output)/'assets')
    except BaseException:
        env.close(); raise
    return env

def observe(env):
    robot = env.robots[0]
    return Snapshot((*map(float,robot.get_position()[:2]),float(robot.get_rpy()[2])),
                    env.door_scene.angle,env.door_scene.version)

def run_episode(env, config, output):
    env.reset()
    failed_door=False; tracker=None; plan=None; waiting=0; attempts=0; replans=0
    checkpoint=None; checkpoint_step=0; length=0.; previous=np.array(observe(env).pose[:2])
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    with (output/'episode.jsonl').open('w',encoding='utf-8') as log:
        def record(row):
            log.write(json.dumps(row)+'\n'); log.flush()
        for step in range(config['max_steps']):
            state=observe(env)
            if tracker is None and waiting == 0:
                if replans >= config['max_replans']:
                    reason='replan_limit'; break
                try:
                    plan,candidates=choose_plan(state,config,failed_door)
                except ValueError:
                    reason='no_feasible_plan'; break
                replans+=1
                record(dict(type='decision',step=step,selected=plan.candidate_id,
                            candidates=[dict(id=p.candidate_id,seconds=p.estimated_seconds) for p in candidates]))
                tracker=WaypointTracker(plan.waypoints,plan.waypoints[-1],.08,config['goal_tolerance'])
                checkpoint=None; checkpoint_step=step
            action=np.zeros(2)
            if waiting:
                waiting-=1
                if waiting == 0:
                    attempts+=1
                    safe=np.linalg.norm(np.array(state.pose[:2])-CONTACT)<config['goal_tolerance']
                    if safe and not config['fail_open']:
                        env.door_scene.set_open(True)
                    failed_door=not env.door_scene.verified_open()
                    record(dict(type='open_result',step=step,success=not failed_door))
                    tracker=None
            else:
                if state.map_version != plan.map_version:
                    tracker=None; continue
                tracker.update(state.pose)
                if tracker.completed:
                    if plan.candidate_id == 'open_door':
                        waiting=max(1,math.ceil(config['open_seconds']/config['dt']))
                    else:
                        tracker=None
                else:
                    distance,angle=compute_target_error(state.pose,tracker.current_waypoint)
                    action=compute_action(distance,angle,tracker.current_tolerance,'shortest')
                    action[1]*=config['angular_action_sign']
                    # Include rotation as progress; turning in place is not a stall.
                    if checkpoint is None:
                        checkpoint=state.pose
                    if step-checkpoint_step >= config['stall_steps']:
                        moved=np.linalg.norm(np.array(state.pose[:2])-checkpoint[:2])
                        turned=abs(math.atan2(math.sin(state.pose[2]-checkpoint[2]),math.cos(state.pose[2]-checkpoint[2])))
                        if moved < .03 and turned < .05:
                            record(dict(type='stalled',step=step))
                            if plan.candidate_id == 'open_door': failed_door=True
                            tracker=None; action=np.zeros(2)
                        checkpoint=state.pose; checkpoint_step=step
            action=np.clip(action,env.action_space.low,env.action_space.high).astype(np.float32)
            _,reward,done,info=env.step(action)
            post=observe(env); xy=np.array(post.pose[:2]); length+=float(np.linalg.norm(xy-previous)); previous=xy
            record(dict(type='step',step=step,pose=post.pose,door_angle=post.door_angle,action=action.tolist()))
            if done:
                reason=info.get('reason','environment_done'); break
        else:
            reason='step_limit'
        result=dict(success=reason=='success',reason=reason,steps=step+1,
                    simulated_seconds=(step+1)*config['dt'],path_length=length,open_attempts=attempts,replans=replans)
        record(dict(type='summary',**result))
    return result
