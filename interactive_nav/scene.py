"""Create renderer-visible walls and a software-actuated URDF door."""
from pathlib import Path
import math
from .planning import WALLS, OPEN_ANGLE

def box_link(name, center, size, mass=0):
    xyz = ' '.join(map(str, center)); dimensions = ' '.join(map(str, size))
    return f'''<link name="{name}"><inertial><mass value="{mass}"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="{xyz}"/><geometry><box size="{dimensions}"/></geometry><material name="grey"><color rgba="0.5 0.6 0.7 1"/></material></visual>
    <collision><origin xyz="{xyz}"/><geometry><box size="{dimensions}"/></geometry></collision></link>'''

class DoorScene:
    def __init__(self, env, directory):
        import pybullet as p
        from igibson.objects.articulated_object import ArticulatedObject
        self.p = p
        directory = Path(directory); directory.mkdir(parents=True,exist_ok=True)
        self.objects = []
        for i,(x0,y0,x1,y1) in enumerate(WALLS):
            path = directory / f'wall_{i}.urdf'
            path.write_text('<robot name="wall">'+box_link('base',((x0+x1)/2,(y0+y1)/2,.6),(x1-x0,y1-y0,1.2))+'</robot>',encoding='utf-8')
            obj = ArticulatedObject(str(path)); env.simulator.import_object(obj); self.objects.append(obj)
        path = directory/'door.urdf'
        path.write_text('<robot name="door"><link name="base"><inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial></link>'+box_link('leaf',(0,.7,.6),(.08,1.4,1.2),1)+
            '<joint name="hinge" type="revolute"><parent link="base"/><child link="leaf"/><origin xyz="0 -0.7 0"/><axis xyz="0 0 1"/><limit lower="-1.571" upper="0" effort="1000" velocity="1"/></joint></robot>',encoding='utf-8')
        self.door = ArticulatedObject(str(path),merge_fixed_links=False)
        env.simulator.import_object(self.door)
        self.body = self.door.get_body_ids()[0]
        self.version = 0

    @property
    def angle(self):
        return float(self.p.getJointState(self.body,0)[0])

    def set_open(self, opened):
        target = OPEN_ANGLE if opened else 0.
        self.p.resetJointState(self.body,0,target)
        self.p.setJointMotorControl2(self.body,0,self.p.POSITION_CONTROL,targetPosition=target,force=1000)
        self.version += 1

    def verified_open(self):
        return abs(self.angle-OPEN_ANGLE) < .05
