from pathlib import Path
import yaml
from interactive_nav.push_probe import PushController, box_urdf, validate_config
import xml.etree.ElementTree as ET


def setup():
    c = yaml.safe_load((Path(__file__).parents[1]/'push_probe.yaml').read_text())
    s = dict(box_position=[1., 0., .175], box_rpy=[0., 0., 0.], robot_rpy=[0., 0., 0.],
             robot_speed=0., box_speed=0., robot_angular_speed=0., box_angular_speed=0.,
             contacts=[], unexpected_contacts=[])
    t = PushController(c)
    for _ in range(15):
        t.tick(s)
    assert t.state == 'APPROACH'
    return t, s


def test_success_requires_box_motion_and_final_stop():
    t, s = setup()
    s['contacts'] = [{'robot_link': -1}]
    t.tick(s)
    assert t.state == 'PUSH'
    s['box_position'][0] += .51
    s['box_speed'] = .1
    assert not t.tick(s).any()
    assert t.state == 'STOP'
    for _ in range(10):
        t.tick(s)
    assert t.state == 'STOP'
    s['box_speed'] = 0.
    for _ in range(10):
        t.tick(s)
    assert t.state == 'SUCCESS'


def test_contact_without_displacement_stalls():
    t, s = setup()
    s['contacts'] = [{}]
    for _ in range(50):
        t.tick(s)
    assert t.reason == 'stalled'


def test_contact_loss_and_lateral_drift():
    t, s = setup()
    s['contacts'] = [{}]; t.tick(s)
    s['contacts'] = []
    for _ in range(6):
        t.tick(s)
    assert t.reason == 'contact_lost'
    t, s = setup()
    s['box_position'][1] = .2
    assert not t.tick(s).any()
    assert t.reason == 'box_drift'


def test_unexpected_collision_stops():
    t, s = setup()
    s['unexpected_contacts'] = [{}]
    assert not t.tick(s).any()
    assert t.reason == 'unexpected_collision'


def test_dynamic_box_has_physical_inertia():
    t, _ = setup()
    validate_config(t.c)
    root = ET.fromstring(box_urdf(t.c))
    assert float(root.find('link/inertial/mass').get('value')) > 0
    assert float(root.find('link/inertial/inertia').get('ixx')) > 0
    assert root.find('link/collision') is not None
