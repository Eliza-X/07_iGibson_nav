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
    parser.add_argument('--preview',action='store_true',help='Inspect the frozen scene in a resizable window; no navigation')
    parser.add_argument('--inspect-door',action='store_true',help='Stationary robot, live physics, O/C motorized door controls')
    parser.add_argument('--plan-only',action='store_true',help='Geometry/cost check; does not run physics')
    parser.add_argument('--initially-open',action='store_true')
    parser.add_argument('--fail-open',action='store_true')
    args=parser.parse_args()
    if args.inspect_door and (args.preview or args.headless or args.plan_only):
        parser.error('--inspect-door cannot combine with --preview, --headless or --plan-only')
    if args.preview and (args.headless or args.plan_only):
        parser.error('--preview cannot be combined with --headless or --plan-only')
    config=yaml.safe_load(Path(args.config).read_text(encoding='utf-8'))
    if args.initially_open: config['initially_open']=True
    if args.fail_open: config['fail_open']=True
    for key in ('resolution','clearance_radius','estimated_speed','estimated_angular_speed','dt','goal_tolerance','stall_steps','max_steps','max_replans'):
        if config[key] <= 0: raise ValueError(f'{key} must be positive')
    if config['open_seconds']<0: raise ValueError('open_seconds must be nonnegative')
    if config['angular_action_sign'] not in (-1,1): raise ValueError('angular_action_sign must be -1 or 1')
    for key in ('door_speed','door_force','door_timeout','stop_speed','stop_yaw_rate','stop_hold_time','stop_timeout','interaction_distance','navigation_timeout','crossing_action_limit'):
        if config[key] <= 0: raise ValueError(f'{key} must be positive')
    if not 0 < config['passage_angle'] <= abs(OPEN_ANGLE): raise ValueError('passage_angle must be in (0, pi/2]')
    if args.plan_only:
        print('Policy: approach -> stop -> open -> verify passage -> cross -> stop at goal. Cost selection disabled.')
        return
    from interactive_nav.runtime import create_env,run_episode
    output=Path(__file__).parent/'outputs'/'door_bypass'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    (output/'config.json').write_text(json.dumps(config,indent=2),encoding='utf-8')
    # Preview owns its window; suppress the native iGibson viewer.
    if args.preview or args.inspect_door:
        config.update(render_width=1280,render_height=960)
    env=create_env(config,'headless' if args.headless or args.preview or args.inspect_door else 'gui_interactive',output)
    try:
        if args.preview or args.inspect_door:
            from interactive_nav.preview import preview_scene
            preview_scene(env,interactive=args.inspect_door)
        else:
            print(json.dumps(run_episode(env,config,output),indent=2))
        print('Logs:',output)
    finally:
        env.close()

if __name__=='__main__':
    main()
