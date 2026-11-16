"""Planning invariants and closed-loop orchestration without renderer dependencies."""
from pathlib import Path
import numpy as np
import yaml
from interactive_nav.planning import NavigationMap, Snapshot, START, GOAL, CONTACT, OPEN_ANGLE, choose_plan
from interactive_nav.runtime import run_episode

def config():
    return yaml.safe_load((Path(__file__).parents[1]/'door_bypass.yaml').read_text())

def test_cost_changes_strategy():
    cfg=config(); state=Snapshot((*START,0),0,0)
    assert choose_plan(state,cfg)[0].candidate_id=='open_door'
    cfg['open_seconds']=100
    assert choose_plan(state,cfg)[0].candidate_id=='navigate_goal'

def test_maps_and_failed_door():
    cfg=config(); closed=NavigationMap(); opened=NavigationMap(OPEN_ANGLE)
    assert not closed.free[closed.to_grid((0,0))]
    assert opened.free[opened.to_grid((0,0))]
    assert max(p[1] for p in closed.path(START,GOAL))>1.6
    assert max(abs(p[1]) for p in opened.path(START,GOAL))<.2
    assert choose_plan(Snapshot((*START,0),0,0),cfg,True)[0].candidate_id=='navigate_goal'
    assert len(choose_plan(Snapshot((*START,0),OPEN_ANGLE,1),cfg)[1])==1
    assert np.allclose(opened.to_world(opened.to_grid(CONTACT)),CONTACT)

class FakeEnv:
    """Kinematic test double: tests sequencing, not iGibson physics or control scaling."""
    def __init__(self,cfg):
        self.cfg=cfg; self.resets=0; self.steps=0; self.pose=np.array([*START,0.])
        self.robots=[self]; self.door_scene=self; self.angle=0.; self.version=0
        self.action_space=type('Space',(),dict(low=np.array([-1.,-1.]),high=np.array([1.,1.])))()
    def get_position(self): return np.array([*self.pose[:2],0.])
    def get_rpy(self): return [0,0,self.pose[2]]
    def reset(self): self.resets+=1
    def set_open(self,opened): self.angle=OPEN_ANGLE if opened else 0.; self.version+=1
    def verified_open(self): return abs(self.angle-OPEN_ANGLE)<.05
    def step(self,action):
        self.steps+=1
        self.pose[2]+=action[1]*self.cfg['angular_action_sign']*self.cfg['dt']
        self.pose[:2]+=action[0]*self.cfg['dt']*np.array([np.cos(self.pose[2]),np.sin(self.pose[2])])
        success=np.linalg.norm(self.pose[:2]-GOAL)<self.cfg['goal_tolerance']
        return {},0.,success,dict(reason='success' if success else '',success=success)

def test_runner_open_and_failure_recovery(tmp_path):
    for fail in (False,True):
        cfg=config(); cfg['fail_open']=fail
        env=FakeEnv(cfg)
        result=run_episode(env,cfg,tmp_path/str(fail))
        assert result['success']
        assert env.resets==1
        assert result['open_attempts']==1
        assert result['replans']==2
        assert env.verified_open() != fail
