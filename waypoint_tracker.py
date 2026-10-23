"""Ordered planar targets, independent of iGibson and task rewards."""

import numpy as np


def compute_target_error(robot_pose, waypoint):
    """Return XY distance and signed bearing error from (x, y, yaw)."""
    x, y, yaw = robot_pose
    dx, dy = np.asarray(waypoint) - [x, y]
    angle = np.arctan2(dy, dx) - yaw
    return float(np.hypot(dx, dy)), float(np.arctan2(np.sin(angle), np.cos(angle)))


class WaypointTracker:
    def __init__(self, waypoints, target_pos, waypoint_tolerance, dist_tol):
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
        self.reset()

    @classmethod
    def from_config(cls, config, target_pos, dist_tol):
        points = config.get("waypoints", [list(target_pos[:2])])
        return cls(points, target_pos, config.get("waypoint_tolerance", 0.15), dist_tol)

    def reset(self):
        self.index = 0

    @property
    def completed(self):
        return self.index == len(self.waypoints)

    @property
    def current_waypoint(self):
        return None if self.completed else self.waypoints[self.index].copy()

    @property
    def current_tolerance(self):
        return self.dist_tol if self.index >= len(self.waypoints) - 1 else self.waypoint_tolerance

    def update(self, robot_pose):
        """Consume consecutive reached points; return zero-based indices.

        Strict comparison matches the existing controller's stopping rule.
        """
        reached = []
        while not self.completed:
            distance, _ = compute_target_error(robot_pose, self.current_waypoint)
            if distance >= self.current_tolerance:
                break
            reached.append(self.index)
            self.index += 1
        return reached
