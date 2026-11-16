"""Planning invariants and closed-loop orchestration without renderer dependencies."""
from pathlib import Path
import numpy as np
import yaml
from interactive_nav.planning import NavigationMap, Snapshot, START, GOAL, CONTACT, OPEN_ANGLE, choose_plan
from interactive_nav.runtime import run_episode
from interactive_nav.execution import DoorTraversal
from interactive_nav.door import DoorController, door_urdf
from types import SimpleNamespace
import xml.etree.ElementTree as ET

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

def test_motor_command_wakes_without_teleport():
    calls=[]
    physics=SimpleNamespace(getNumJoints=lambda body:1,getJointInfo=lambda body,i:(0,b'door_hinge'),
        ACTIVATION_STATE_WAKE_UP=4,POSITION_CONTROL=2,
        changeDynamics=lambda *a,**kw:calls.append(('wake',a,kw)),
        setJointMotorControl2=lambda *a,**kw:calls.append(('motor',a,kw)))
    door=DoorController(physics,7,config()); door.request(True)
    assert [c[0] for c in calls]==['wake','wake','motor']
    assert calls[-1][2]['maxVelocity']==.5
    root=ET.fromstring(door_urdf())
    assert root.find("link[@name='door_leaf']/collision") is not None
    assert root.find("joint[@name='door_hinge']").get('type')=='revolute'

def test_wait_for_physical_open_and_passage_loss():
    cfg=config(); requests=[]
    door=SimpleNamespace(angle=0.,request=lambda opened:requests.append(opened))
    task=DoorTraversal(cfg,door); task.transition('STOPPING')
    for _ in range(4): task.tick((*CONTACT,0),0,0,.1)
    assert requests==[True] and task.state=='WAIT_FOR_DOOR'
    assert np.all(task.tick((*CONTACT,0),0,0,.1)==0)
    door.angle=OPEN_ANGLE
    task.tick((*CONTACT,0),0,0,.1)
    assert task.state=='CROSSING'
    door.angle=0.
    task.tick((*CONTACT,0),0,0,.1)
    assert task.reason=='passage_lost'

def test_jammed_door_stops_without_bypass():
    cfg=config(); cfg['fail_open']=True
    door=SimpleNamespace(angle=0.)
    task=DoorTraversal(cfg,door); task.transition('STOPPING')
    for _ in range(140):
        assert np.all(task.tick((*CONTACT,0),0,0,.1)==0)
    assert task.state=='FAILED' and task.requests==1

def test_closed_door_blocks_centerline():
    from interactive_nav.planning import EXIT
    assert not NavigationMap().segment_clear(CONTACT,EXIT)
    assert NavigationMap(OPEN_ANGLE).segment_clear(CONTACT,EXIT)

def test_complete_traversal_requires_final_stop():
    cfg=config(); door=SimpleNamespace(angle=OPEN_ANGLE)
    task=DoorTraversal(cfg,door)
    for _ in range(6): task.tick((*CONTACT,0),0,0,.1)
    assert task.state=='CROSSING'
    pose=(*CONTACT,0)
    for _ in range(200):
        task.tick(pose,0,0,.1)
        if task.tracker and not task.tracker.completed:
            pose=(*task.tracker.current_waypoint,0)
        if task.state=='FINAL_STOP': break
    assert task.state=='FINAL_STOP'
    task.tick((*GOAL,0),1.,0,.1)
    assert task.state=='FINAL_STOP'
    for _ in range(4): task.tick((*GOAL,0),0,0,.1)
    assert task.state=='SUCCESS' and task.requests==0
