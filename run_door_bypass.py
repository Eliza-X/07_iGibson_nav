"""Run the ground-truth door experiment without modifying installed iGibson."""
import argparse
from datetime import datetime
from pathlib import Path
import json
import yaml
from interactive_nav.planning import Snapshot, START, OPEN_ANGLE, choose_plan

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',default=str(Path(__file__).with_name('door_bypass.yaml')))
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--plan-only',action='store_true',help='Geometry/cost check; does not run physics')
    parser.add_argument('--open-seconds',type=float)
    parser.add_argument('--initially-open',action='store_true')
    parser.add_argument('--fail-open',action='store_true')
    args=parser.parse_args()
    config=yaml.safe_load(Path(args.config).read_text(encoding='utf-8'))
    if args.open_seconds is not None: config['open_seconds']=args.open_seconds
    if args.initially_open: config['initially_open']=True
    if args.fail_open: config['fail_open']=True
    for key in ('resolution','clearance_radius','estimated_speed','estimated_angular_speed','dt','goal_tolerance','stall_steps','max_steps','max_replans'):
        if config[key] <= 0: raise ValueError(f'{key} must be positive')
    if config['open_seconds']<0: raise ValueError('open_seconds must be nonnegative')
    if config['angular_action_sign'] not in (-1,1): raise ValueError('angular_action_sign must be -1 or 1')
    if args.plan_only:
        state=Snapshot((*START,0.),OPEN_ANGLE if config['initially_open'] else 0.,0)
        selected,candidates=choose_plan(state,config)
        print(json.dumps(dict(selected=selected.candidate_id,candidates=[dict(id=p.candidate_id,seconds=p.estimated_seconds) for p in candidates]),indent=2))
        return
    from interactive_nav.runtime import create_env,run_episode
    output=Path(__file__).parent/'outputs'/'door_bypass'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    (output/'config.json').write_text(json.dumps(config,indent=2),encoding='utf-8')
    env=create_env(config,'headless' if args.headless else 'gui_interactive',output)
    try:
        print(json.dumps(run_episode(env,config,output),indent=2))
        print('Logs:',output)
    finally:
        env.close()

if __name__=='__main__':
    main()
