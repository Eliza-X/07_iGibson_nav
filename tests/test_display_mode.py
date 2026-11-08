import sys
from types import ModuleType, SimpleNamespace

import pytest

import env_nonint_fixed_point as entry


def test_mode_priority_and_validation():
    assert entry.resolve_display_mode({}) == "classic"
    assert entry.resolve_display_mode({"display_mode": "video"}) == "video"
    assert entry.resolve_display_mode({"display_mode": "video"}, "classic") == "classic"
    assert entry.resolve_display_mode({"display_mode": "classic"}, "video") == "video"
    with pytest.raises(ValueError, match="display_mode"):
        entry.resolve_display_mode({"display_mode": "typo"})


def test_cli_arguments(monkeypatch):
    monkeypatch.setattr(entry, "main", lambda **kwargs: kwargs)
    args = entry.cli(["--display-mode", "classic", "--headless", "--short-exec",
                      "--no-follow-camera", "--no-route-map", "--no-map-waypoints"])
    assert args == dict(display_mode="classic", headless=True, short_exec=True,
                        follow_camera=False, show_route_map=False, show_map_waypoints=False)
    assert entry.cli([])["display_mode"] is None
    with pytest.raises(SystemExit):
        entry.cli(["--display-mode", "unknown"])


@pytest.mark.parametrize("mode", ["video", "classic"])
@pytest.mark.parametrize("headless", [False, True])
@pytest.mark.parametrize("wait_for_map", [False, True])
def test_main_selects_windows_recording_resolution_and_cleanup(monkeypatch, mode, headless, wait_for_map):
    events = []
    config = dict(scene="gibson", scene_id="Rs", build_graph=True, task="point_nav_fixed",
                  initial_pos=[0, 0, 0], target_pos=[1, 1, 0], image_width=128, image_height=128,
                  display_mode="classic" if mode == "video" else "video",
                  record_video=False, quad_view_enabled=False)  # obsolete flags must not win
    monkeypatch.setattr(entry.yaml, "safe_load", lambda _: config.copy())

    class Env:
        def __init__(self, config_file, mode):
            events.append(("env", mode, config_file))
            self.scene = object()
            self.task = SimpleNamespace(floor_num=0, initial_pos=[0, 0, 0], target_pos=[1, 1, 0])

        def close(self):
            events.append("env.close")

    class Map:
        def __init__(self, *args, **kwargs):
            events.append("map.create")

        def update(self, env):
            events.append("map.update")

        def finish(self, env, summary):
            events.append("map.finish")

        def wait_until_closed(self):
            events.append("map.wait")

        def close(self):
            events.append("map.close")

    class Recorder(Map):
        video_path = "test.mp4"

        def __init__(self, env, floor_map, result, waypoints, config, preview, show_waypoints):
            events.append(("recorder", preview, config["record_video"]))

        def update(self, env):
            events.append("recorder.update")

        def close(self):
            events.append("recorder.close")

    modules = {name: ModuleType(name) for name in (
        "igibson", "igibson.envs", "igibson.envs.igibson_env", "igibson.utils",
        "igibson.utils.assets_utils", "quad_view_recorder", "live_path_visualization")}
    modules["igibson.envs.igibson_env"].iGibsonEnv = Env
    modules["igibson.utils.assets_utils"].download_assets = lambda: None
    modules["igibson.utils.assets_utils"].download_demo_data = lambda: None
    modules["quad_view_recorder"].QuadViewRecorder = Recorder
    modules["live_path_visualization"].LivePathVisualization = Map
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(entry, "plan_path", lambda *args: (None, dict(path_world_xy=[[0, 0], [1, 1]], geodesic_distance_m=1.4)))
    monkeypatch.setattr(entry, "execution_waypoints", lambda *args: [[0, 0], [1, 1]])
    monkeypatch.setattr(entry, "update_follow_camera", lambda env: events.append("follow"))

    def run(env, config_data, action_fn, observation_callback):
        observation_callback(env)
        return dict(accepted=True, reason="success", steps=1)

    monkeypatch.setattr(entry, "run_closed_loop", run)
    summary = entry.main(display_mode=mode, headless=headless, wait_for_map=wait_for_map,
                         config_data=config)
    assert "max_step" not in config  # Entry must not mutate the caller's config.
    _, env_mode, effective = events[0]
    assert env_mode == ("gui_interactive" if mode == "classic" and not headless else "headless")
    assert effective["display_mode"] == mode
    assert effective["image_width"] == (640 if mode == "video" else 128)
    assert effective["record_video"] is (mode == "video")
    if mode == "video":
        assert ("recorder", not headless, True) in events
        assert "recorder.update" in events and "map.create" not in events and "follow" not in events
        assert events.index("recorder.close") < events.index("env.close")
        assert summary["video_path"] == "test.mp4"
    else:
        assert "video_path" not in summary and "recorder.update" not in events
        assert ("map.create" in events) is (not headless)
        assert ("follow" in events) is (not headless)
        if not headless:
            assert events.index("env.close") < events.index("map.close")
            if wait_for_map:
                assert events.index("env.close") < events.index("map.wait") < events.index("map.close")
            else:
                assert "map.wait" not in events
