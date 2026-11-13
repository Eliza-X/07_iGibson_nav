import json

import numpy as np

from interaction_scene import ROOT
from plot_interaction_path import main, plan_interaction
from run_interaction_nav import load_config


def test_plan_segments_match_runtime_map_and_interaction():
    config = load_config(ROOT / 'interaction_nav.yaml')
    scene, result = plan_interaction(config)
    assert result['errors'] == []
    assert result['geometrically_complete']
    assert [s['name'] for s in result['segments']] == ['approach', 'crossing', 'to_goal']
    for index, segment in enumerate(result['segments']):
        scene.update(0 if index == 0 else config['door_open_angle'])
        points = segment['points']
        assert points[1:] == scene.plan(points[0], points[-1])
        assert all(scene.segment_clear(a, b) for a, b in zip(points, points[1:]))
    assert result['segments'][0]['points'][-1] == result['interaction']['position']
    assert result['segments'][1]['points'][0] == result['interaction']['position']
    np.testing.assert_allclose(result['segments'][-1]['points'][-1], config['target_position'][:2])


def test_blocked_corridor_has_no_second_navigation():
    _, result = plan_interaction(load_config(ROOT / 'interaction_nav.yaml', 'blocked'))
    assert not result['passage_clear_at_target']
    assert not result['geometrically_complete']
    assert [s['name'] for s in result['segments']] == ['approach']
    assert result['errors']


def test_jammed_route_explicitly_hypothetical():
    _, result = plan_interaction(load_config(ROOT / 'interaction_nav.yaml', 'jammed'))
    assert result['geometrically_complete']
    assert any('hypothetical' in error for error in result['errors'])


def test_export_png_and_action_json_without_simulator(tmp_path):
    png = main(['--no-show', '--output-dir', str(tmp_path)])
    assert png.read_bytes().startswith(b'\x89PNG')
    data = json.loads(png.with_suffix('.json').read_text(encoding='utf-8'))
    assert data['interaction']['actions'] == ['stop', 'request_open_if_needed', 'wait_for_clearance']
