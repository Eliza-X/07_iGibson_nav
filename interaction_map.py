"""Known geometry map, derived from the same URDF as the physical scene."""

import math
import xml.etree.ElementTree as ET

import numpy as np

from astar_planner import astar_grid
from interaction_scene import build_room_urdf


class InteractionMap:
    def __init__(self, resolution=0.05, radius=0.22, margin=0.06, obstacles=()):
        if not all(math.isfinite(v) and v > 0 for v in (resolution, radius, margin)):
            raise ValueError("Map resolution, robot radius and margin must be positive")
        self.resolution = resolution
        self.radius = radius
        self.padding = radius + margin + resolution / math.sqrt(2)
        self.origin = np.array([-4.2, -3.2])
        self.shape = tuple(np.ceil(np.array([6.4, 8.4]) / resolution).astype(int))
        rows, cols = np.indices(self.shape)
        self.xy = self.origin + np.stack([cols, rows], axis=-1) * resolution
        root = ET.fromstring(build_room_urdf())
        self.static = []
        for element in root.findall("link[@name='building']/collision"):
            xyz = np.fromstring(element.find('origin').get('xyz'), sep=' ')
            size = np.fromstring(element.find('geometry/box').get('size'), sep=' ')
            if xyz[2] + size[2] / 2 <= 0.01 or xyz[2] - size[2] / 2 > 0.6:
                continue  # Floor and overhead lintel are not planar obstacles.
            self.static.append((xyz[:2], size[:2], 0.0))
        hinge = root.find("joint[@name='door_hinge']/origin")
        self.hinge = np.fromstring(hinge.get('xyz'), sep=' ')[:2]
        leaf = root.find("link[@name='door_leaf']/collision")
        self.leaf_offset = np.fromstring(leaf.find('origin').get('xyz'), sep=' ')[:2]
        self.leaf_size = np.fromstring(leaf.find('geometry/box').get('size'), sep=' ')[:2]
        for item in obstacles:
            center, size = np.asarray(item['center'], float), np.asarray(item['size'], float)
            if center.shape != (2,) or size.shape != (2,) or not np.isfinite([center, size]).all() or np.any(size <= 0):
                raise ValueError('Obstacle center/size must be finite XY; size must be positive')
            self.static.append((center, size, 0.0))
        self.update(0.0)

    def update(self, angle):
        if getattr(self, '_angle', None) == angle:
            return
        self._angle = angle
        c, s = math.cos(angle), math.sin(angle)
        center = self.hinge + np.array([[c, -s], [s, c]]) @ self.leaf_offset
        self.boxes = self.static + [(center, self.leaf_size, angle)]
        self.free = self.clear(self.xy)

    def clear(self, points):
        points = np.asarray(points, float)
        valid = np.isfinite(points).all(axis=-1)
        valid &= (points >= self.origin).all(axis=-1)
        valid &= (points <= self.origin + (np.array(self.shape[::-1]) - 1) * self.resolution).all(axis=-1)
        for center, size, angle in self.boxes:
            c, s = math.cos(angle), math.sin(angle)
            local = (points - center) @ np.array([[c, -s], [s, c]])
            outside = np.maximum(np.abs(local) - np.asarray(size) / 2, 0)
            valid &= np.linalg.norm(outside, axis=-1) > self.padding
        return valid

    def segment_clear(self, start, end):
        # Sampling gaps are covered by the extra half-cell diagonal padding.
        count = max(2, int(np.ceil(np.linalg.norm(np.asarray(end) - start) / (self.resolution / 2))) + 1)
        return bool(self.clear(np.linspace(start, end, count)).all())

    def world_to_map(self, xy):
        return tuple(np.rint((np.asarray(xy) - self.origin) / self.resolution).astype(int)[::-1])

    def map_to_world(self, rc):
        return self.origin + np.asarray(rc)[::-1] * self.resolution

    def plan(self, start, goal):
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        if not self.clear([start, goal]).all():
            raise ValueError('Path endpoint lacks robot clearance')
        nodes, _ = astar_grid(self.free, self.world_to_map(start), self.world_to_map(goal))
        points = [start] + [self.map_to_world(rc) for rc in nodes] + [goal]
        if not all(self.segment_clear(a, b) for a, b in zip(points, points[1:])):
            raise ValueError('Grid path or endpoint connector lacks clearance')
        # Visibility shortcuts are checked with the same inflated geometry.
        result, index = [], 0
        while index < len(points) - 1:
            candidate = len(points) - 1
            while candidate > index + 1 and not self.segment_clear(points[index], points[candidate]):
                candidate -= 1
            result.append(points[candidate].tolist())
            index = candidate
        return result
