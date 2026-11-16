"""Motorized door geometry, rotated from feature/interaction into this layout."""
import math
import xml.etree.ElementTree as ET

HINGE = (0., -.67, .03)
LEAF_CENTER = (0., .67, 1.1)
LEAF_SIZE = (.06, 1.34, 2.2)
OPEN_ANGLE = -math.pi/2

def door_urdf():
    root=ET.Element('robot',name='automatic_door')
    base=ET.SubElement(root,'link',name='building')
    inertia=ET.SubElement(base,'inertial')
    ET.SubElement(inertia,'mass',value='0')
    ET.SubElement(inertia,'inertia',ixx='0',iyy='0',izz='0',ixy='0',ixz='0',iyz='0')
    leaf=ET.SubElement(root,'link',name='door_leaf')
    inertia=ET.SubElement(leaf,'inertial')
    ET.SubElement(inertia,'origin',xyz='0 .67 1.1')
    ET.SubElement(inertia,'mass',value='8')
    ET.SubElement(inertia,'inertia',ixx='4.424',iyy='3.228',izz='1.199',ixy='0',ixz='0',iyz='0')
    for name,xyz,size,color,collision in (
        ('door',LEAF_CENTER,LEAF_SIZE,'0.78 0.35 0.12 1',True),
        ('stripe',(-.034,1.18,1.1),(.01,.05,1.8),'0.95 0.85 0.3 1',False)):
        for tag in ('visual','collision') if collision else ('visual',):
            item=ET.SubElement(leaf,tag,name=name)
            ET.SubElement(item,'origin',xyz=' '.join(map(str,xyz)))
            geometry=ET.SubElement(item,'geometry')
            ET.SubElement(geometry,'box',size=' '.join(map(str,size)))
            if tag=='visual':
                material=ET.SubElement(item,'material',name=name)
                ET.SubElement(material,'color',rgba=color)
    joint=ET.SubElement(root,'joint',name='door_hinge',type='revolute')
    ET.SubElement(joint,'parent',link='building'); ET.SubElement(joint,'child',link='door_leaf')
    ET.SubElement(joint,'origin',xyz=' '.join(map(str,HINGE)))
    ET.SubElement(joint,'axis',xyz='0 0 1')
    ET.SubElement(joint,'limit',lower=str(OPEN_ANGLE),upper='0',effort='40',velocity='.5')
    ET.SubElement(joint,'dynamics',damping='.4',friction='.1')
    return ET.tostring(root,encoding='unicode')

class DoorController:
    def __init__(self,physics,body,config):
        self.p,self.body,self.config=physics,body,config
        self.joint=next(i for i in range(physics.getNumJoints(body))
                        if physics.getJointInfo(body,i)[1]==b'door_hinge')
        self.target=0.; self.version=0
    @property
    def angle(self): return float(self.p.getJointState(self.body,self.joint)[0])
    def request(self,opened):
        self.target=OPEN_ANGLE if opened else 0.
        for link in range(-1,self.p.getNumJoints(self.body)):
            self.p.changeDynamics(self.body,link,activationState=self.p.ACTIVATION_STATE_WAKE_UP)
        self.p.setJointMotorControl2(self.body,self.joint,self.p.POSITION_CONTROL,
            targetPosition=self.target,force=self.config['door_force'],maxVelocity=self.config['door_speed'])
        self.version+=1
    def reset(self,opened=False):
        self.p.resetJointState(self.body,self.joint,OPEN_ANGLE if opened else 0.,targetVelocity=0)
        self.request(opened)
    @property
    def status(self):
        if abs(self.angle-self.target)>.03:
            return 'OPENING / BLOCKED' if self.target else 'CLOSING / BLOCKED'
        return 'OPEN' if self.target else 'CLOSED'
    def diagnostics(self):
        link=self.p.getLinkState(self.body,self.joint,computeForwardKinematics=True)
        return dict(angle=self.angle,status=self.status,link_position=list(link[4]),
                    link_orientation=list(link[5]),aabb=self.p.getAABB(self.body,self.joint))
