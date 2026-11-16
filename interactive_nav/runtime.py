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
            env.door_scene.reset(config['initially_open'])
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
            success = False  # Traversal state machine owns final stop and success.
            reason = ('collision' if collision_links else
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
        env.door_scene = DoorScene(env,Path(output)/'assets',config)
    except BaseException:
        env.close(); raise
    return env

def observe(env):
    robot = env.robots[0]
    return Snapshot((*map(float,robot.get_position()[:2]),float(robot.get_rpy()[2])),
                    env.door_scene.angle,env.door_scene.version)

from .execution import run_episode
