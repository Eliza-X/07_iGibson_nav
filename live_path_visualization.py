"""Nonblocking live map of the planned path and measured robot trajectory."""

from time import monotonic

import numpy as np

from plot_fixed_point_path import draw_path


def world_to_map_float(scene, world_xy):
    """iGibson world XY to continuous map RC, for display only.

    Accept a single point or an array of points. Unlike world_to_map, this
    preserves sub-cell positions; planning still uses integer map indices.
    """
    return (np.asarray(world_xy, dtype=float)[..., ::-1]
            / scene.trav_map_resolution + scene.trav_map_size / 2.0)


class LivePathVisualization:
    def __init__(self, scene, floor_map, result, refresh_hz=8, *, waypoints=None, show_waypoints=True,
                 display=True):
        import matplotlib.pyplot as plt
        from matplotlib.patches import FancyArrowPatch

        if not np.isfinite(refresh_hz) or refresh_hz <= 0:
            raise ValueError("refresh_hz must be finite and positive")
        self.plt = plt
        self.display = display
        self.scene = scene
        self.interval = 1.0 / refresh_hz
        self.last_draw = -float("inf")
        self.history = []  # Measured world XY positions, without quantization.
        self.closed = False
        if display:
            self.fig = draw_path(floor_map, result)
        else:
            from matplotlib.figure import Figure
            from matplotlib.backends.backend_agg import FigureCanvasAgg
            self.fig = Figure(figsize=(6, 6), dpi=100)
            FigureCanvasAgg(self.fig)
            draw_path(floor_map, result, fig=self.fig)
        # Agg/inline can save images but cannot provide a live desktop window.
        if display and self.fig.canvas.required_interactive_framework is None:
            plt.close(self.fig)
            raise RuntimeError(
                "Live route map requires a desktop Matplotlib backend (TkAgg or QtAgg). "
                "Install/enable Tk or Qt, or run main(show_route_map=False)."
            )
        if display:
            self.fig.canvas.manager.set_window_title("Live route map")
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.ax = self.fig.axes[0]
        self.base_title = self.ax.get_title()
        if show_waypoints and waypoints is not None and len(waypoints):
            waypoint_rc = np.asarray([scene.world_to_map(np.asarray(xy)) for xy in waypoints])
            self.ax.scatter(waypoint_rc[:, 1], waypoint_rc[:, 0], s=12,
                            color="purple", edgecolors="none", label="Waypoints", zorder=3)
        self.trail, = self.ax.plot([], [], color="darkorange", linewidth=1.5,
                                   label="Actual trajectory", zorder=5)
        self.robot, = self.ax.plot([], [], "o", color="darkorange", markeredgecolor="black",
                                   markersize=8, label="Robot", zorder=7)
        self.heading = FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>",
                                      mutation_scale=15, color="darkorange", zorder=6)
        self.ax.add_patch(self.heading)
        self.heading.set_visible(False)
        self.ax.legend(loc="upper right")
        self.ax.set_autoscale_on(False)
        if not display:
            # Updates add a second title line; reserve space in the video canvas.
            self.fig.subplots_adjust(top=0.88)
        if display:
            plt.show(block=False)

    def _on_close(self, event):
        self.closed = True

    def update(self, env, force=False):
        if self.closed or (self.display and not self.plt.fignum_exists(self.fig.number)):
            self.closed = True
            return
        position = np.asarray(env.robots[0].get_position(), dtype=float)[:2]
        yaw = float(env.robots[0].get_rpy()[2])
        # Keep sub-cell positions for both the robot and its heading arrow.
        rc = world_to_map_float(self.scene, position)
        ahead = position + 0.4 * np.array([np.cos(yaw), np.sin(yaw)])
        ahead_rc = world_to_map_float(self.scene, ahead)
        if not self.history or not np.array_equal(position, self.history[-1]):
            self.history.append(position.copy())
        now = monotonic()
        if not force and now - self.last_draw < self.interval:
            return
        self.last_draw = now
        history = world_to_map_float(self.scene, self.history)
        self.trail.set_data(history[:, 1], history[:, 0])
        self.robot.set_data([rc[1]], [rc[0]])
        self.heading.set_positions(rc[::-1], ahead_rc[::-1])
        self.heading.set_visible(True)
        self.ax.set_title(f"{self.base_title}\nRunning | x={position[0]:.2f}, y={position[1]:.2f} m")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def finish(self, env, summary):
        self.update(env, force=True)
        if not self.closed:
            status = "SUCCESS" if summary.get("accepted") else "NOT SUCCESSFUL"
            self.ax.set_title(f"{self.base_title}\n{status} | {summary.get('reason')} | "
                              f"{summary.get('steps')} steps")
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def wait_until_closed(self):
        """Keep the final trajectory visible after the simulation is released."""
        if self.display and not self.closed and self.plt.fignum_exists(self.fig.number):
            self.plt.show(block=True)

    def frame_rgb(self):
        """Render the latest map state to an independent uint8 RGB image."""
        self.fig.canvas.draw()
        return np.asarray(self.fig.canvas.buffer_rgba())[:, :, :3].copy()

    def close(self):
        self.closed = True
        if self.display:
            self.plt.close(self.fig)
        else:
            self.fig.clear()
