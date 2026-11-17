"""Ordered planar targets, independent of iGibson and task rewards."""

import numpy as np


def compute_target_error(robot_pose, waypoint):
    """Return XY distance and signed bearing error from (x, y, yaw)."""
    x, y, yaw = robot_pose
    dx, dy = np.asarray(waypoint) - [x, y]
    angle = np.arctan2(dy, dx) - yaw
    return float(np.hypot(dx, dy)), float(np.arctan2(np.sin(angle), np.cos(angle)))


def corner_indices(points, epsilon=1e-5):
    """Ignore duplicate/tiny connectors; preserve meaningful direction changes."""
    points = np.asarray(points, dtype=float)
    kept = [0]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i]-points[kept[-1]]) > epsilon:
            kept.append(i)
    corners = set()
    for a, b, c in zip(kept, kept[1:], kept[2:]):
        incoming, outgoing = points[b]-points[a], points[c]-points[b]
        cosine = np.dot(incoming, outgoing)/(np.linalg.norm(incoming)*np.linalg.norm(outgoing))
        if cosine < 1-1e-6:
            corners.add(b)
    return corners


class WaypointTracker:
    def __init__(self, waypoints, target_pos, waypoint_tolerance, dist_tol,
                 critical_indices=None, corner_tolerance=.025, fine_tolerance=.005):
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or not len(points):
            raise ValueError("waypoints must be a nonempty list of [x, y] points")
        if not np.isfinite(points).all():
            raise ValueError("waypoints must contain finite coordinates")
        target = np.asarray(target_pos, dtype=float)
        if target.shape not in ((2,), (3,)) or not np.isfinite(target).all():
            raise ValueError("target_pos must contain finite XY or XYZ coordinates")
        if not np.allclose(points[-1], target[:2], atol=1e-8, rtol=0):
            raise ValueError("last waypoint must match target_pos XY")
        self.waypoint_tolerance = float(waypoint_tolerance)
        self.dist_tol = float(dist_tol)
        if any(not np.isfinite(t) or t <= 0 for t in (self.waypoint_tolerance, self.dist_tol)):
            raise ValueError("waypoint_tolerance and dist_tol must be finite and positive")
        self.waypoints = points.copy()
        self.critical_indices = set(critical_indices or ())
        if any(not isinstance(i, (int, np.integer)) or not 0 <= i < len(points)-1 for i in self.critical_indices):
            raise ValueError('Critical indices must reference non-final waypoints')
        self.corner_tolerance, self.fine_tolerance = corner_tolerance, fine_tolerance
        if not 0 < fine_tolerance < corner_tolerance <= self.waypoint_tolerance:
            if self.critical_indices:
                raise ValueError('Require 0 < fine < corner <= waypoint tolerance')
        self.reset()

    @classmethod
    def from_config(cls, config, target_pos, dist_tol):
        points = config.get("waypoints", [list(target_pos[:2])])
        return cls(points, target_pos, config.get("waypoint_tolerance", 0.15), dist_tol)

    def reset(self):
        self.index = 0
        self.refining = self.corner_pause = self.blocked_at_precision = False
        self.blocked_target = None

    @property
    def completed(self):
        return self.index == len(self.waypoints)

    @property
    def current_waypoint(self):
        return None if self.completed else self.waypoints[self.index].copy()

    @property
    def current_tolerance(self):
        if self.refining:
            return self.fine_tolerance
        if self.index >= len(self.waypoints)-1:
            return self.dist_tol
        return self.corner_tolerance if self.index in self.critical_indices else self.waypoint_tolerance

    def update(self, robot_pose, segment_clear=None):
        """Consume consecutive reached points; return zero-based indices.

        Strict comparison matches the existing controller's stopping rule.
        """
        reached = []
        self.refining = self.corner_pause = self.blocked_at_precision = False
        self.blocked_target = None
        while not self.completed:
            distance, _ = compute_target_error(robot_pose, self.current_waypoint)
            if distance >= self.current_tolerance:
                break
            if self.index+1 < len(self.waypoints) and segment_clear is not None:
                candidate = self.waypoints[self.index+1]
                if not segment_clear(robot_pose[:2], candidate):
                    self.blocked_target = candidate.copy()
                    self.refining = True
                    self.blocked_at_precision = distance < self.fine_tolerance
                    break
            critical = self.index in self.critical_indices
            reached.append(self.index)
            self.index += 1
            if critical:
                self.corner_pause = True
                break
        return reached
