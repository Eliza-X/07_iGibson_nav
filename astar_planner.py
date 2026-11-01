"""A* on the scene's loaded traversability map; no scene graph search."""

import heapq
from itertools import count
from math import sqrt

import numpy as np


def astar_grid(free, start, goal, connectivity=8, allow_corner_cutting=False):
    """Return all (row, column) nodes and cost in cells on a boolean grid."""
    if isinstance(connectivity, bool) or connectivity not in (4, 8):
        raise ValueError("planner_connectivity must be 4 or 8")
    if not isinstance(allow_corner_cutting, bool):
        raise ValueError("planner_allow_corner_cutting must be a boolean")
    free = np.asarray(free, dtype=bool)
    if free.ndim != 2 or not free.size:
        raise ValueError("Traversability map must be a nonempty 2D grid")
    start, goal = tuple(start), tuple(goal)

    def valid(node):
        r, c = node
        return 0 <= r < free.shape[0] and 0 <= c < free.shape[1] and free[r, c]

    for name, node in (("start", start), ("goal", goal)):
        if len(node) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, np.integer)) for v in node):
            raise ValueError(f"{name} must be an integer row/column pair")
        if not valid(node):
            raise ValueError(f"{name} {node} is outside the traversable map")

    def heuristic(node):
        dr, dc = abs(node[0] - goal[0]), abs(node[1] - goal[1])
        return dr + dc if connectivity == 4 else max(dr, dc) + (sqrt(2) - 1) * min(dr, dc)

    moves = [(-1, 0), (0, -1), (0, 1), (1, 0)]
    if connectivity == 8:
        moves += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
    serial = count()
    queue = [(heuristic(start), next(serial), 0.0, start)]
    costs, parents = {start: 0.0}, {}
    while queue:
        _, _, cost, node = heapq.heappop(queue)
        if cost > costs[node]:
            continue
        if node == goal:
            path = [node]
            while node in parents:
                node = parents[node]
                path.append(node)
            return path[::-1], cost
        r, c = node
        for dr, dc in moves:
            neighbor = (r + dr, c + dc)
            if not valid(neighbor):
                continue
            diagonal = dr != 0 and dc != 0
            if diagonal and not allow_corner_cutting and not (valid((r + dr, c)) and valid((r, c + dc))):
                continue
            candidate = cost + (sqrt(2) if diagonal else 1.0)
            if candidate < costs.get(neighbor, float("inf")):
                costs[neighbor] = candidate
                parents[neighbor] = node
                heapq.heappush(queue, (candidate + heuristic(neighbor), next(serial), candidate, neighbor))
    raise ValueError("No traversable path between start and goal under the selected neighborhood rules")


def plan_path(scene, config):
    """Return (floor_map, result) compatible with plotting and waypoint execution.

    Map values of 255 are traversable. The loaded map may already be eroded and
    filtered by iGibson. Endpoint cells are preserved, with no extra connectors.
    """
    floor = config.get("floor_num", 0)
    maps = getattr(scene, "floor_map", [])
    if isinstance(floor, bool) or not isinstance(floor, int) or not 0 <= floor < len(maps):
        raise ValueError("floor_num must identify an available floor map")
    floor_map = np.asarray(maps[floor]).copy()
    if floor_map.ndim != 2 or not floor_map.size or not np.isfinite(floor_map).all():
        raise ValueError("Scene must supply a finite, nonempty 2D floor map")
    resolution = float(scene.trav_map_resolution)
    if not np.isfinite(resolution) or resolution <= 0:
        raise ValueError("trav_map_resolution must be finite and positive")
    endpoints = []
    for name in ("initial_pos", "target_pos"):
        point = np.asarray(config[name], dtype=float)
        if point.shape not in ((2,), (3,)) or not np.isfinite(point).all():
            raise ValueError(f"{name} must contain finite XY or XYZ coordinates")
        rc = np.asarray(scene.world_to_map(point[:2]))
        if rc.shape != (2,) or not np.isfinite(rc).all() or not np.equal(rc, np.floor(rc)).all():
            raise ValueError(f"{name} must map to an integer row/column pair")
        endpoints.append(tuple(int(v) for v in rc))
    connectivity = config.get("planner_connectivity", 8)
    corner_cutting = config.get("planner_allow_corner_cutting", False)
    path, cost = astar_grid(floor_map == 255, *endpoints, connectivity, corner_cutting)
    world = np.asarray([scene.map_to_world(np.asarray(rc)) for rc in path], dtype=float)
    if world.shape != (len(path), 2) or not np.isfinite(world).all():
        raise ValueError("Scene returned invalid world coordinates")
    return floor_map, {
        "scene_id": config["scene_id"], "floor_num": floor,
        "initial_pos": list(config["initial_pos"]), "target_pos": list(config["target_pos"]),
        "endpoint_map_rc": [list(rc) for rc in endpoints],
        "path_world_xy": world.tolist(), "path_map_rc": [list(rc) for rc in path],
        "geodesic_distance_m": float(cost * resolution),
        "trav_map_resolution": resolution,
        "planner": "astar_grid", "planner_connectivity": connectivity,
        "planner_allow_corner_cutting": corner_cutting,
        "coordinate_note": "Map coordinates are [row, column]; endpoints are quantized to cells. "
                           "No extra endpoint segments added. geodesic_distance_m is the grid path length.",
    }
