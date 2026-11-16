"""Simulator-independent geometry, state contracts and cost planning.

Coordinates are metres in world XY; arrays use row=y, column=x.
"""
from dataclasses import dataclass
import math
import numpy as np
from astar_planner import astar_grid
from .door import HINGE, LEAF_CENTER, LEAF_SIZE, OPEN_ANGLE

# Shared by the physical scene and rasterizer: xmin, ymin, xmax, ymax.
WALLS = ((-4.1,-3.1,4.1,-2.9), (-4.1,2.9,4.1,3.1),
         (-4.1,-3,-3.9,3), (3.9,-3,4.1,3),
         (-.1,-3,.1,-.7), (-.1,.7,.1,1.6))
START = (-2.5, 0.)
GOAL = (2.5, 0.)
CONTACT = (-.8, 0.)
EXIT = (1.8, 0.)

@dataclass(frozen=True)
class Snapshot:
    pose: tuple
    door_angle: float
    map_version: int

@dataclass
class Plan:
    candidate_id: str
    map_version: int
    waypoints: list
    estimated_seconds: float

class NavigationMap:
    def __init__(self, angle=0., resolution=.1, radius=.25):
        if resolution <= 0 or radius <= 0:
            raise ValueError('Resolution and clearance radius must be positive')
        self.resolution = resolution
        self.radius = radius
        self.origin = np.array([-4., -3.])
        xs = np.arange(-4., 4. + resolution/2, resolution)
        ys = np.arange(-3., 3. + resolution/2, resolution)
        x, y = np.meshgrid(xs, ys)
        occupied = np.zeros(x.shape, dtype=bool)
        # Cell diagonal allowance makes rasterization conservative.
        margin = radius + resolution / math.sqrt(2)
        for x0,y0,x1,y1 in WALLS:
            occupied |= (x >= x0-margin)&(x <= x1+margin)&(y >= y0-margin)&(y <= y1+margin)
        # Door local rectangle: hinge (0,-.7), width .08, length 1.4.
        dx, dy = x-HINGE[0], y-HINGE[1]
        local_x = math.cos(angle)*dx + math.sin(angle)*dy
        local_y = -math.sin(angle)*dx + math.cos(angle)*dy
        occupied |= (abs(local_x-LEAF_CENTER[0]) <= LEAF_SIZE[0]/2+margin)&(abs(local_y-LEAF_CENTER[1]) <= LEAF_SIZE[1]/2+margin)
        self.angle=angle
        self.free = ~occupied

    def segment_clear(self,start,end):
        points=np.linspace(start,end,max(2,int(np.ceil(np.linalg.norm(np.asarray(end)-start)/(self.resolution/2)))+1))
        for point in points:
            row,col=self.to_grid(point)
            if not (0<=row<self.free.shape[0] and 0<=col<self.free.shape[1] and self.free[row,col]):
                return False
        return True

    def to_grid(self, xy):
        col, row = np.rint((np.asarray(xy[:2])-self.origin)/self.resolution).astype(int)
        return int(row), int(col)

    def to_world(self, rc):
        return (self.origin + np.array([rc[1], rc[0]])*self.resolution).tolist()

    def path(self, start, goal):
        nodes, _ = astar_grid(self.free, self.to_grid(start), self.to_grid(goal), 8, False)
        points = [self.to_world(n) for n in nodes]
        # Connections stay inside conservative free endpoint cells.
        points = [list(start[:2])] + points + [list(goal[:2])]
        return [p for i,p in enumerate(points) if i == 0 or np.linalg.norm(np.array(p)-points[i-1]) > 1e-8]

def travel_seconds(points, yaw=0., speed=.12, angular_speed=.5):
    total = 0.
    for a,b in zip(points, points[1:]):
        delta = np.asarray(b)-a
        heading = math.atan2(delta[1],delta[0])
        total += np.linalg.norm(delta)/speed
        total += abs(math.atan2(math.sin(heading-yaw), math.cos(heading-yaw)))/angular_speed
        yaw = heading
    return float(total)

def choose_plan(state, config, failed_door=False):
    kwargs = dict(resolution=config['resolution'], radius=config['clearance_radius'])
    current = NavigationMap(state.door_angle, **kwargs)
    choices = []
    def estimate(points, yaw):
        return travel_seconds(points, yaw, config['estimated_speed'], config['estimated_angular_speed'])
    try:
        path = current.path(state.pose, GOAL)
        choices.append(Plan('navigate_goal', state.map_version, path, estimate(path,state.pose[2])))
    except ValueError:
        pass
    if not failed_door and abs(state.door_angle-OPEN_ANGLE) > .05:
        try:
            approach = current.path(state.pose, CONTACT)
            onward = NavigationMap(OPEN_ANGLE, **kwargs).path(CONTACT, GOAL)
            cost = estimate(approach,state.pose[2]) + config['open_seconds'] + estimate(onward,0.)
            choices.append(Plan('open_door', state.map_version, approach, cost))
        except ValueError:
            pass
    if not choices:
        raise ValueError('No feasible candidate')
    # Stable tie: navigate without interaction.
    return min(choices, key=lambda p:p.estimated_seconds), choices
