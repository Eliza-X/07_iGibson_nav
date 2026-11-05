from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
from quad_view_recorder import QuadViewRecorder, compose_quad


class Renderer:
    def __init__(self):
        self.camera = np.array([8., 9., 10.])
        self.target = np.array([0., 0., 0.])
        self.up = np.array([0., 0., 1.])
        self.cameras = []

    def set_camera(self, camera, target, up, cache=False):
        assert not cache
        self.camera, self.target, self.up = map(np.asarray, (camera, target, up))
        self.cameras.append(self.camera.copy())

    def render(self, modes):
        assert modes == ("rgb",)
        return [np.full((60, 80, 4), [1., 0., 0., 1.])]

    def render_robot_cameras(self, modes, cache=False):
        self.set_camera([2, 2, 2], [3, 2, 2], [0, 0, 1])
        return [np.full((60, 80, 4), [0., 0., 1., 1.])]


def setup_recorder(tmp_path, record=True):
    scene = SimpleNamespace(trav_map_resolution=0.1, trav_map_size=12,
                            world_to_map=lambda xy: (np.asarray(xy)[::-1] / .1 + 6).astype(int))
    robot = SimpleNamespace(get_position=lambda: [0., 0., .1], get_rpy=lambda: [0, 0, np.pi / 2])
    env = SimpleNamespace(scene=scene, robots=[robot], current_step=0, action_timestep=.1,
                          simulator=SimpleNamespace(renderer=Renderer()))
    result = dict(path_map_rc=[[6, 6], [7, 7]], endpoint_map_rc=[[6, 6], [7, 7]],
                  scene_id="test", floor_num=0, geodesic_distance_m=.14)
    config = dict(video_output_dir=str(tmp_path), video_panel_width=320, video_panel_height=240,
                  fixed_camera_position=[1, -2, 2], fixed_camera_target=[0, 0, 0],
                  video_final_hold_seconds=.2, record_video=record)
    recorder = QuadViewRecorder(env, np.full((12, 12), 255), result, [[0, 0], [.1, .1]], config, preview=False)
    return recorder, env


def test_layout_color_order_and_aspect():
    colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [255, 255, 0]]
    frames = [np.full((100, 100, 3), c, dtype=np.uint8) for c in colors]
    frame = compose_quad(frames, 320, 240, "test")
    assert frame.shape == (480, 640, 3)
    for i, (x, y) in enumerate([(160, 120), (480, 120), (160, 360), (480, 360)]):
        np.testing.assert_equal(frame[y, x], colors[i][::-1])
    np.testing.assert_equal(frame[120, 0], [24, 24, 24])  # square frame is not stretched


def test_mp4_roundtrip_timing_and_camera_restore(tmp_path):
    recorder, env = setup_recorder(tmp_path)
    path = recorder.video_path
    renderer = env.simulator.renderer
    try:
        recorder.update(env)
        env.current_step = 1
        recorder.update(env)
        recorder.finish(env, dict(accepted=True, reason="success", steps=1))
        assert recorder.frame_count == 4  # reset, one step, 0.2-second final hold
        np.testing.assert_allclose(renderer.camera, [8, 9, 10])
        np.testing.assert_allclose(renderer.target, [0, 0, 0])
        np.testing.assert_allclose(renderer.cameras[1], [-.6, -1.5, 1.3])
        assert env.current_step == 1  # rendering never advances the simulation
    finally:
        recorder.close()
    video = cv2.VideoCapture(str(path))
    try:
        assert video.isOpened()
        assert video.get(cv2.CAP_PROP_FPS) == pytest.approx(10)
        frames = []
        while True:
            ok, frame = video.read()
            if not ok:
                break
            frames.append(frame)
        assert len(frames) == 4
        assert frames[0].shape == (480, 640, 3)
    finally:
        video.release()


def test_renderer_restored_on_capture_failure(tmp_path):
    recorder, env = setup_recorder(tmp_path, record=False)
    env.simulator.renderer.render_robot_cameras = lambda **kwargs: []
    try:
        with pytest.raises(RuntimeError, match="No robot camera"):
            recorder.capture(env)
        np.testing.assert_allclose(env.simulator.renderer.camera, [8, 9, 10])
        assert recorder.video_path is None
    finally:
        recorder.close()
