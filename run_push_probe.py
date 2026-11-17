"""Independent physical pushing probe; existing door experiments are unchanged."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import yaml
from interactive_nav.push_probe import validate_config, run_probe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).with_name('push_probe.yaml')))
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--mass', type=float)
    parser.add_argument('--repeats', type=int, default=1)
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text(encoding='utf-8'))
    if args.mass is not None:
        config['box_mass'] = args.mass
    validate_config(config)
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    output = Path(__file__).parent/'outputs'/'push_probe'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    (output/'config.json').write_text(json.dumps(config, indent=2), encoding='utf-8')
    results = []
    for i in range(args.repeats):
        result = run_probe(config, 'headless' if args.headless else 'gui_interactive', output/f'trial_{i+1:02d}')
        results.append(result)
        print(json.dumps(result, indent=2))
    (output/'summary.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    print('Logs:', output)


if __name__ == '__main__':
    main()
