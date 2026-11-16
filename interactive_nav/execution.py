"""Deterministic door traversal. Cost selection is deliberately disabled."""
import json
from pathlib import Path
import numpy as np
from action_policy import compute_action
from waypoint_tracker import WaypointTracker, compute_target_error
from .planning import NavigationMap, CONTACT, EXIT, GOAL

class DoorTraversal:
    def __init__(self,config,door):
        self.config,self.door=config,door
        self.state='APPROACH'; self.reason=''; self.elapsed=0.; self.entered=0.
        self.still=0.; self.requests=0; self.tracker=None
    def transition(self,state):
        self.state=state; self.entered=self.elapsed; self.still=0.; self.tracker=None
    def fail(self,reason):
        self.reason=reason; self.transition('FAILED')
    def tick(self,pose,speed,yaw_rate,dt):
        zero=np.zeros(2); self.elapsed+=dt; c=self.config
        if self.state in ('FAILED','SUCCESS'): return zero
        nav=NavigationMap(self.door.angle,c['resolution'],c['clearance_radius'])
        ready=abs(self.door.angle)>=c['passage_angle'] and nav.segment_clear(CONTACT,EXIT)
        stationary=speed<=c['stop_speed'] and abs(yaw_rate)<=c['stop_yaw_rate']
        if self.state=='STOPPING':
            if np.linalg.norm(np.asarray(pose[:2])-CONTACT)>c['interaction_distance']:
                self.fail('outside_interaction_range'); return zero
            self.still=self.still+dt if stationary else 0.
            if self.still>=c['stop_hold_time']:
                if not ready:
                    if not c['fail_open']: self.door.request(True)
                    self.requests+=1
                self.transition('WAIT_FOR_DOOR')
            elif self.elapsed-self.entered>c['stop_timeout']: self.fail('stop_timeout')
            return zero
        if self.state=='WAIT_FOR_DOOR':
            if ready: self.transition('CROSSING')
            elif self.elapsed-self.entered>c['door_timeout']: self.fail('door_open_timeout_or_blocked')
            return zero
        if self.state=='FINAL_STOP':
            if np.linalg.norm(np.asarray(pose[:2])-GOAL)>=c['goal_tolerance']:
                self.transition('TO_GOAL'); return zero
            self.still=self.still+dt if stationary else 0.
            if self.still>=c['stop_hold_time']:
                self.reason='success'; self.transition('SUCCESS')
            return zero
        if self.elapsed-self.entered>c['navigation_timeout']:
            self.fail('navigation_timeout'); return zero
        if self.state=='CROSSING' and not ready:
            self.fail('passage_lost'); return zero
        target=CONTACT if self.state=='APPROACH' else EXIT if self.state=='CROSSING' else GOAL
        if self.tracker is None:
            try:
                path=nav.path(pose,target)
                self.tracker=WaypointTracker(path,target,.08,c['goal_tolerance'])
            except ValueError:
                self.fail('no_path'); return zero
        self.tracker.update(pose)
        if self.tracker.completed:
            self.transition({'APPROACH':'STOPPING','CROSSING':'TO_GOAL','TO_GOAL':'FINAL_STOP'}[self.state])
            return zero
        if not nav.segment_clear(pose[:2],self.tracker.current_waypoint):
            self.fail('route_obstructed'); return zero
        distance,angle=compute_target_error(pose,self.tracker.current_waypoint)
        action=compute_action(distance,angle,self.tracker.current_tolerance,'shortest')
        if self.state=='CROSSING': action[0]=min(action[0],c['crossing_action_limit'])
        action[1]*=c['angular_action_sign']
        return action

def run_episode(env,config,output):
    from .runtime import observe
    env.reset(); task=DoorTraversal(config,env.door_scene)
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    length=0.; previous=np.array(observe(env).pose[:2]); steps=0
    with (output/'episode.jsonl').open('w',encoding='utf-8') as log:
        def record(data): log.write(json.dumps(data)+'\n'); log.flush()
        record(dict(type='start',policy='fixed_door_traversal',cost_selection=False))
        for _ in range(config['max_steps']):
            before=task.state; state=observe(env); robot=env.robots[0]
            action=task.tick(state.pose,float(np.linalg.norm(robot.get_linear_velocity()[:2])),
                             float(robot.get_angular_velocity()[2]),config['dt'])
            _,_,done,info=env.step(np.clip(action,env.action_space.low,env.action_space.high).astype(np.float32))
            steps+=1; post=observe(env); xy=np.array(post.pose[:2]); length+=float(np.linalg.norm(xy-previous)); previous=xy
            # Check actual contact against ALL robot links, independent of task filtering.
            p=env.door_scene.p
            contacts=[contact for body in robot.get_body_ids()
                      for contact in p.getContactPoints(bodyA=body,bodyB=env.door_scene.body)
                      if contact[8]<=0]
            if contacts: task.fail('door_collision')
            elif done and not info.get('success',False): task.fail(info.get('reason','environment_done'))
            record(dict(type='step',step=steps,state=task.state,previous_state=before,pose=post.pose,
                        action=action.tolist(),door=env.door_scene.diagnostics(),door_contacts=len(contacts)))
            if task.state in ('SUCCESS','FAILED'): break
        else: task.fail('step_limit')
        result=dict(success=task.state=='SUCCESS',reason=task.reason,steps=steps,
                    simulated_seconds=steps*config['dt'],path_length=length,open_attempts=task.requests)
        record(dict(type='summary',**result))
    return result
