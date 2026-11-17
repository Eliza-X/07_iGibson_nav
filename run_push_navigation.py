"""Single-box interactive navigation; reuse the probe runtime and existing A*."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import yaml
from interactive_nav.push_probe import run_probe, validate_config
from interactive_nav.push_planning import check_layout


def load_config(path):
    root = Path(__file__).parent
    c = yaml.safe_load((root/'push_probe.yaml').read_text(encoding='utf-8'))
    validate_config(c)
    overrides = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    c.update(overrides)
    for key, value in c.items():
        if key in ('start', 'goal', 'box_position', 'prepush', 'release_point'):
            if len(value) != 2 or not all(math.isfinite(v) for v in value):
                raise ValueError(f'{key} must be finite XY')
        else:
            validate_config({key: value, 'approach_action': c['approach_action'], 'push_action': c['push_action']})
    if c['box_position'][1] != 0 or c['prepush'][1] != 0 or c['release_point'][1] != 0:
        raise ValueError('First version requires box/prepush/release aligned with world +X')
    if not c['prepush'][0] < c['box_position'][0] < c['release_point'][0] < c['box_position'][0]+c['push_distance']:
        raise ValueError('Invalid approach/push/release ordering')
    if c['doorway_width'] <= max(c['box_size'][1], 2*c['clearance_radius']):
        raise ValueError('Doorway must admit both robot and box')
    if c['release_action'] > 1 or int(c['max_replans']) != c['max_replans']:
        raise ValueError('Invalid release action or replan count')
    if not 0 < c['fine_tolerance'] < c['corner_tolerance'] <= c['waypoint_tolerance']:
        raise ValueError('Require 0 < fine_tolerance < corner_tolerance <= waypoint_tolerance')
    return c


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).with_name('push_navigation.yaml')))
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--plan-only', action='store_true')
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--playback-speed', type=float, default=.5)
    args = parser.parse_args()
    if args.repeats < 1 or not math.isfinite(args.playback_speed) or args.playback_speed <= 0:
        parser.error('repeats and playback speed must be positive')
    c = load_config(args.config)
    layout = check_layout(c)
    if args.plan_only:
        print(json.dumps(layout, indent=2))
        return
    output = Path(__file__).parent/'outputs'/'push_navigation'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    (output/'config.json').write_text(json.dumps(c, indent=2), encoding='utf-8')
    (output/'layout.json').write_text(json.dumps(layout, indent=2), encoding='utf-8')
    results = []
    for i in range(args.repeats):
        result = run_probe(c, 'headless' if args.headless else 'gui_interactive', output/f'trial_{i+1:02d}',
                           playback_speed=args.playback_speed, navigation=True)
        results.append(result)
        (output/'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
        print(json.dumps(result, indent=2))
        if result['reason'] == 'user_closed':
            break
    print('Logs:', output)


if __name__ == '__main__':
    main()
