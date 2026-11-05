"""Synchronized four-panel preview and video, without advancing simulation."""

from datetime import datetime
from pathlib import Path
from uuid import uuid4

import cv2
import numpy as np

from live_path_visualization import LivePathVisualization


def rgb_uint8(frame):
    frame = np.asarray(frame)[..., :3]
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("Renderer must return an HxWx3/4 RGB frame")
    if np.issubdtype(frame.dtype, np.floating):
        frame = np.clip(frame, 0, 1) * 255
    return np.ascontiguousarray(frame, dtype=np.uint8)


def compose_quad(frames, width, height, status):
    """Fit RGB frames into 2x2 panels; return an OpenCV BGR image."""
    panels = []
    labels = ("1. Live trajectory", "2. Fixed room camera", "3. Follow camera", "4. Robot camera")
    for frame, label in zip(frames, labels):
        rgb = rgb_uint8(frame)
        scale = min(width / rgb.shape[1], (height - 48) / rgb.shape[0])
        w, h = max(1, round(rgb.shape[1] * scale)), max(1, round(rgb.shape[0] * scale))
        resized = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
        panel = np.full((height, width, 3), 24, dtype=np.uint8)
        x, y = (width - w) // 2, 28 + (height - 48 - h) // 2
        panel[y:y+h, x:x+w] = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)
        cv2.putText(panel, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(panel, status, (10, height - 7), cv2.FONT_HERSHEY_SIMPLEX, .4, (230, 230, 230), 1, cv2.LINE_AA)
        panels.append(panel)
    return np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))


class QuadViewRecorder:
    window = "Navigation - four views"

    def __init__(self, env, floor_map, result, waypoints, config, preview=True, show_waypoints=True):
        self.writer = None
        self.map = None
        self.preview = False
        self.video_path = None
        self.frame_count = 0
        self.width = int(config.get("video_panel_width", 640))
        self.height = int(config.get("video_panel_height", 480))
        if self.width < 160 or self.height < 120:
            raise ValueError("Video panels must be at least 160x120")
        self.dt = float(env.action_timestep)
        if not np.isfinite(self.dt) or self.dt <= 0:
            raise ValueError("action_timestep must be finite and positive")
        self.fps = 1.0 / self.dt
        self.hold_seconds = float(config.get("video_final_hold_seconds", 2.0))
        if not np.isfinite(self.hold_seconds) or self.hold_seconds < 0:
            raise ValueError("video_final_hold_seconds must be finite and nonnegative")
        self.fixed_position = np.asarray(config["fixed_camera_position"], dtype=float)
        self.fixed_target = np.asarray(config["fixed_camera_target"], dtype=float)
        if any(p.shape != (3,) or not np.isfinite(p).all() for p in (self.fixed_position, self.fixed_target)):
            raise ValueError("Fixed camera position/target must be finite XYZ points")
        if np.linalg.norm((self.fixed_target - self.fixed_position)[:2]) < 1e-8:
            raise ValueError("Fixed camera viewing direction must not be parallel to world Z")
        try:
            self.map = LivePathVisualization(env.scene, floor_map, result, waypoints=waypoints,
                                             show_waypoints=show_waypoints, display=False)
            if config.get("record_video", True):
                directory = Path(config.get("video_output_dir", "outputs/videos"))
                if not directory.is_absolute():
                    directory = Path(__file__).resolve().parent / directory
                directory.mkdir(parents=True, exist_ok=True)
                self.video_path = directory / ("navigation_" + datetime.now().strftime("%Y%m%d_%H%M%S")
                                               + "_" + uuid4().hex[:8] + ".mp4")
                self.writer = cv2.VideoWriter(str(self.video_path), cv2.VideoWriter_fourcc(*"mp4v"),
                                              self.fps, (2*self.width, 2*self.height))
                if not self.writer.isOpened():
                    raise RuntimeError("Cannot open MP4 writer; check OpenCV video codec support")
                print(f"Video: {self.video_path} | {self.fps:g} FPS (simulation time)")
            if preview:
                cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
                self.preview = True
                cv2.resizeWindow(self.window, 2*self.width, 2*self.height)
        except BaseException:
            self.close()
            raise

    def capture(self, env, summary=None):
        if summary is None:
            self.map.update(env, force=True)
        else:
            self.map.finish(env, summary)
        renderer = env.simulator.renderer
        saved = [np.array(getattr(renderer, name), copy=True) for name in ("camera", "target", "up")]
        try:
            renderer.set_camera(self.fixed_position, self.fixed_target, [0, 0, 1], cache=False)
            fixed = rgb_uint8(renderer.render(modes=("rgb",))[0]).copy()
            robot = env.robots[0]
            position = np.asarray(robot.get_position(), dtype=float)
            yaw = float(robot.get_rpy()[2])
            camera = position - 1.5 * np.array([np.cos(yaw), np.sin(yaw), 0.0])
            camera += .6 * np.array([-np.sin(yaw), np.cos(yaw), 0.0])
            camera[2] += 1.2
            renderer.set_camera(camera, position + [0, 0, .25], [0, 0, 1], cache=False)
            follow = rgb_uint8(renderer.render(modes=("rgb",))[0]).copy()
            frames = renderer.render_robot_cameras(modes=("rgb",), cache=False)
            if not frames:
                raise RuntimeError("No robot camera frame returned by iGibson")
            first_person = rgb_uint8(frames[0]).copy()
        finally:
            renderer.set_camera(*saved, cache=False)
        step = int(env.current_step)
        status = f"Step {step} | t={step*self.dt:.1f}s"
        if summary is not None:
            status += " | " + str(summary["reason"])
        return compose_quad([self.map.frame_rgb(), fixed, follow, first_person],
                            self.width, self.height, status)

    def _output(self, frame):
        if self.writer is not None:
            self.writer.write(frame)
            self.frame_count += 1
        if self.preview:
            if cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) < 1:
                self.preview = False
                return
            cv2.imshow(self.window, frame)
            if cv2.waitKey(1) & 0xff == 27:
                cv2.destroyWindow(self.window)
                self.preview = False

    def update(self, env):
        self._output(self.capture(env))

    def finish(self, env, summary):
        frame = self.capture(env, summary)
        for _ in range(max(1, round(self.hold_seconds * self.fps))):
            self._output(frame)
        if self.preview:
            # Only the terminal preview waits; simulation-time recording never sleeps.
            cv2.waitKey(max(1, round(self.hold_seconds * 1000)))

    def close(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.preview:
            cv2.destroyWindow(self.window)
            self.preview = False
        if self.map is not None:
            self.map.close()
            self.map = None
