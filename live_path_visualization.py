"""Nonblocking live map of the planned path and measured robot trajectory."""

from time import monotonic

import numpy as np

from plot_fixed_point_path import draw_path


class LivePathVisualization:
    def __init__(self, scene, floor_map, result, refresh_hz=8):
        import matplotlib.pyplot as plt
        from matplotlib.patches import FancyArrowPatch

        if not np.isfinite(refresh_hz) or refresh_hz <= 0:
            raise ValueError("refresh_hz must be finite and positive")
        self.plt = plt
        self.scene = scene
        self.interval = 1.0 / refresh_hz
        self.last_draw = -float("inf")
        self.history = []
        self.closed = False
        self.fig = draw_path(floor_map, result)
        # Agg/inline can save images but cannot provide a live desktop window.
        if self.fig.canvas.required_interactive_framework is None:
            plt.close(self.fig)
            raise RuntimeError(
                "Live route map requires a desktop Matplotlib backend (TkAgg or QtAgg). "
                "Install/enable Tk or Qt, or run main(show_route_map=False)."
            )
        self.fig.canvas.manager.set_window_title("Live route map")
        self.fig.canvas.mpl_connect("close_event", self._on_close)
        self.ax = self.fig.axes[0]
        self.base_title = self.ax.get_title()
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
        plt.show(block=False)

    def _on_close(self, event):
        self.closed = True

    def update(self, env, force=False):
        if self.closed or not self.plt.fignum_exists(self.fig.number):
            self.closed = True
            return
        position = np.asarray(env.robots[0].get_position(), dtype=float)[:2]
        yaw = float(env.robots[0].get_rpy()[2])
        # Convert both ends of the arrow: map axes need not match world axes.
        rc = np.asarray(self.scene.world_to_map(position), dtype=float)
        ahead = position + 0.4 * np.array([np.cos(yaw), np.sin(yaw)])
        ahead_rc = np.asarray(self.scene.world_to_map(ahead), dtype=float)
        if not self.history or not np.array_equal(rc, self.history[-1]):
            self.history.append(rc.copy())
        now = monotonic()
        if not force and now - self.last_draw < self.interval:
            return
        self.last_draw = now
        history = np.asarray(self.history)
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
        if not self.closed and self.plt.fignum_exists(self.fig.number):
            self.plt.show(block=True)

    def close(self):
        self.closed = True
        self.plt.close(self.fig)
