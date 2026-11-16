"""Create renderer-visible walls and a software-actuated URDF door."""
from pathlib import Path
import math
from .planning import WALLS, OPEN_ANGLE
from .door import DoorController, door_urdf

def box_link(name, center, size, mass=0):
    xyz = ' '.join(map(str, center)); dimensions = ' '.join(map(str, size))
    return f'''<link name="{name}"><inertial><mass value="{mass}"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
    <visual><origin xyz="{xyz}"/><geometry><box size="{dimensions}"/></geometry><material name="grey"><color rgba="0.5 0.6 0.7 1"/></material></visual>
    <collision><origin xyz="{xyz}"/><geometry><box size="{dimensions}"/></geometry></collision></link>'''

class DoorScene(DoorController):
    def __init__(self, env, directory, config):
        import pybullet as p
        from igibson.objects.articulated_object import ArticulatedObject
        self.p = p
        directory = Path(directory); directory.mkdir(parents=True,exist_ok=True)
        self.objects = []
        for i,(x0,y0,x1,y1) in enumerate(WALLS):
            path = directory / f'wall_{i}.urdf'
            path.write_text('<robot name="wall">'+box_link('base',((x0+x1)/2,(y0+y1)/2,1.25),(x1-x0,y1-y0,2.5))+'</robot>',encoding='utf-8')
            obj = ArticulatedObject(str(path)); env.simulator.import_object(obj); self.objects.append(obj)
        path = directory/'door.urdf'
        path.write_text(door_urdf(),encoding='utf-8')
        self.door = ArticulatedObject(str(path),merge_fixed_links=False)
        env.simulator.import_object(self.door)
        self.body = self.door.get_body_ids()[0]
        super().__init__(p,self.body,config)
        lintel=directory/'lintel.urdf'
        lintel.write_text('<robot name="lintel">'+box_link('base',(0,0,2.4),(.2,1.4,.2))+'</robot>',encoding='utf-8')
        obj=ArticulatedObject(str(lintel)); env.simulator.import_object(obj); self.objects.append(obj)
