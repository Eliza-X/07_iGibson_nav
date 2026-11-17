"""Box-aware occupancy using the existing A* and grid coordinate contract."""
import math
import numpy as np
from .planning import NavigationMap


def walls(c):
    gap = c['doorway_width']/2
    return ((-4.1, -3.1, 4.1, -2.9), (-4.1, 2.9, 4.1, 3.1),
            (-4.1, -3, -3.9, 3), (3.9, -3, 4.1, 3),
            (-.1, -3, .1, -gap), (-.1, gap, .1, 3))


class BoxMap(NavigationMap):
    def __init__(self, c, position, yaw, include_box=True):
        self.resolution = c['resolution']
        self.radius = c['clearance_radius']
        self.origin = np.array([-4., -3.])
        xs = np.arange(-4., 4.+self.resolution/2, self.resolution)
        ys = np.arange(-3., 3.+self.resolution/2, self.resolution)
        x, y = np.meshgrid(xs, ys)
        margin = self.radius+self.resolution/math.sqrt(2)
        occupied = np.zeros(x.shape, dtype=bool)
        for x0, y0, x1, y1 in walls(c):
            occupied |= (x >= x0-margin)&(x <= x1+margin)&(y >= y0-margin)&(y <= y1+margin)
        if include_box:
            dx, dy = x-position[0], y-position[1]
            bx = math.cos(yaw)*dx+math.sin(yaw)*dy
            by = -math.sin(yaw)*dx+math.cos(yaw)*dy
            occupied |= (abs(bx) <= c['box_size'][0]/2+margin)&(abs(by) <= c['box_size'][1]/2+margin)
        self.free = ~occupied

    def path(self, start, goal):
        path = super().path(start, goal)
        if not all(self.segment_clear(a, b) for a, b in zip(path, path[1:])):
            raise ValueError('Path endpoint connection is obstructed')
        return path

    def first_obstruction(self, start, end):
        start, end = np.asarray(start[:2]), np.asarray(end[:2])
        count = max(2, int(np.ceil(np.linalg.norm(end-start)/(self.resolution/2)))+1)
        for point in np.linspace(start, end, count):
            row, col = self.to_grid(point)
            if not (0 <= row < self.free.shape[0] and 0 <= col < self.free.shape[1] and self.free[row, col]):
                return dict(point=point.tolist(), cell=[row, col])
        return None


def check_layout(c):
    nav = BoxMap(c, c['box_position'], 0.)
    try:
        nav.path(c['start'], c['goal'])
    except ValueError:
        pass
    else:
        raise ValueError('Initial box does not block the route')
    approach = nav.path(c['start'], c['prepush'])
    moved = [c['box_position'][0]+c['push_distance'], c['box_position'][1]]
    onward = BoxMap(c, moved, 0.).path(c['release_point'], c['goal'])
    return dict(initially_blocked=True, approach=approach, predicted_onward=onward)
