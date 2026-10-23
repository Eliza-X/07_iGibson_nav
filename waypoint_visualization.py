"""Manual waypoint markers, independent of scene shortest-path visualization."""


class WaypointVisualization:
    def __init__(self, env, tracker, enabled):
        self.markers = []
        self.active = None
        if not enabled or env.mode != "gui_interactive":
            return
        import pybullet as p
        from igibson.objects.visual_marker import VisualMarker

        self.height = float(env.task.target_pos[2])
        for index, xy in enumerate(tracker.waypoints):
            marker = VisualMarker(
                visual_shape=p.GEOM_CYLINDER, rgba_color=[0, 1, 0, 0.35],
                radius=tracker.dist_tol if index == len(tracker.waypoints) - 1 else tracker.waypoint_tolerance,
                length=0.04, initial_offset=[0, 0, 0.02],
            )
            env.simulator.import_object(marker)
            marker.set_position([float(xy[0]), float(xy[1]), self.height])
            self.markers.append(marker)
        self.active = VisualMarker(
            visual_shape=p.GEOM_CYLINDER, rgba_color=[1, 0.7, 0, 0.7],
            radius=0.06, length=0.25, initial_offset=[0, 0, 0.125],
        )
        env.simulator.import_object(self.active)
        self.update(tracker)

    def update(self, tracker):
        # Completed markers disappear; pending stay green, active is gold.
        for index, marker in enumerate(self.markers):
            for instance in marker.renderer_instances:
                instance.hidden = index < tracker.index
        if self.active is not None:
            for instance in self.active.renderer_instances:
                instance.hidden = tracker.completed
            if not tracker.completed:
                x, y = tracker.current_waypoint
                self.active.set_position([float(x), float(y), self.height])

    def hide(self):
        for marker in self.markers + ([self.active] if self.active is not None else []):
            for instance in marker.renderer_instances:
                instance.hidden = True
