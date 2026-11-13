"""Draw one conditional interaction plan without loading or moving a robot."""

import argparse
import json
import math
import os
from pathlib import Path

import numpy as np

from interaction_map import InteractionMap
from interaction_scene import ROOT
from run_interaction_nav import load_config


def plan_interaction(config):
    c = config
    scene = InteractionMap(c['map_resolution'], c['robot_radius'], c['safety_margin'], c['obstacles'])
    start, wait, exit_point, goal = (list(c[key][:2]) for key in
                                   ('robot_position', 'door_wait_point', 'door_exit_point', 'target_position'))
    initial_angle = c['door_open_angle'] if c['scenario'] == 'open' else 0.0
    result = {'scenario': c['scenario'], 'start': start, 'wait': wait, 'exit': exit_point, 'goal': goal,
              'initial_angle': initial_angle, 'planned_open_angle': c['door_open_angle'],
              'robot_radius': c['robot_radius'], 'safety_margin': c['safety_margin'],
              'map_resolution': c['map_resolution'], 'segments': [], 'errors': [],
              'note': 'Conditional geometric plan, not an executed trajectory. Runtime replans from measured pose.',
              'interaction': {'position': wait, 'actions': ['stop', 'request_open_if_needed', 'wait_for_clearance'],
                              'interaction_distance': c['interaction_distance'],
                              'required_angle': c['passage_angle'],
                              'condition': 'Measured angle reaches threshold AND inflated crossing corridor is clear'}}

    def segment(name, source, target):
        try:
            route = [source] + scene.plan(source, target)
            result['segments'].append({'name': name, 'points': route,
                                       'length_m': float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())})
        except ValueError as exc:
            result['errors'].append(name + ': ' + str(exc))

    scene.update(initial_angle)
    segment('approach', start, wait)
    scene.update(c['door_open_angle'])
    result['passage_clear_at_target'] = scene.segment_clear(wait, exit_point)
    if result['passage_clear_at_target']:
        segment('crossing', wait, exit_point)
        segment('to_goal', exit_point, goal)
    else:
        result['errors'].append('Crossing corridor blocked: second navigation is not executable')
    if c['scenario'] == 'jammed':
        result['errors'].append('Jammed scenario: opening will fail; green route is hypothetical only')
    result['geometrically_complete'] = len(result['segments']) == 3
    return scene, result


def draw_plan(scene, result):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Wedge, FancyArrowPatch
    from matplotlib.colors import ListedColormap

    fig, ax = plt.subplots(figsize=(12, 9))
    fig.subplots_adjust(left=0.07, right=0.98, top=0.89, bottom=0.21)
    blue, green, orange = '#2473b9', '#18845b', '#de7c16'
    r = scene.resolution
    extent = [scene.origin[0] - r / 2, scene.origin[0] + (scene.shape[1] - .5) * r,
              scene.origin[1] - r / 2, scene.origin[1] + (scene.shape[0] - .5) * r]
    ax.imshow(~scene.free, origin='lower', extent=extent, interpolation='nearest',
              cmap=ListedColormap(['#fafbfc', '#e2e6eb']), vmin=0, vmax=1)

    def box(center, size, angle, **kwargs):
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * np.asarray(size) / 2
        cs, sn = math.cos(angle), math.sin(angle)
        corners = corners @ np.array([[cs, sn], [-sn, cs]]) + center
        ax.add_patch(Polygon(corners, **kwargs))

    for center, size, angle in scene.static:
        box(center, size, angle, facecolor='#586574', edgecolor='#394451', zorder=2)
    opening = math.degrees(result['planned_open_angle'])
    ax.add_patch(Wedge(scene.hinge, scene.leaf_size[0], 0, opening,
                       color=orange, alpha=.13, label='Door sweep', zorder=3))
    box(scene.hinge + scene.leaf_offset, scene.leaf_size, 0,
        fill=False, edgecolor=orange, linestyle='--', linewidth=2, label='Closed door (reference)', zorder=5)
    box(*scene.boxes[-1], facecolor=orange, edgecolor='#a7590b', label='Open door (planned)', zorder=5)
    a = result['planned_open_angle']
    arc_start = scene.hinge + .9 * np.array([math.cos(.12 * a), math.sin(.12 * a)])
    arc_end = scene.hinge + .9 * np.array([math.cos(.88 * a), math.sin(.88 * a)])
    ax.add_patch(FancyArrowPatch(arc_start, arc_end, connectionstyle='arc3,rad=.25',
                                arrowstyle='-|>', mutation_scale=16, color=orange, linewidth=2, zorder=6))
    ax.scatter(*scene.hinge, s=30, c='#663b13', zorder=7)
    for segment in result['segments']:
        points = np.asarray(segment['points'])
        approach = segment['name'] == 'approach'
        color = blue if approach else green
        label = '1. Navigate to door' if approach else '3. Navigate after clearance' if segment['name'] == 'crossing' else None
        ax.plot(*points.T, color=color, linewidth=3, label=label, zorder=6)
        for source, target in zip(points, points[1:]):
            if np.linalg.norm(target - source) > .2:
                ax.annotate('', xy=source + .65 * (target - source), xytext=source + .4 * (target - source),
                            arrowprops={'arrowstyle': '-|>', 'color': color, 'lw': 2}, zorder=7)
    start, wait, exit_point, goal = [result[key] for key in ('start', 'wait', 'exit', 'goal')]
    ax.add_patch(Circle(start, result['robot_radius'], facecolor=blue, alpha=.12,
                        edgecolor=blue, label='Robot footprint', zorder=4))
    ax.add_patch(Circle(wait, scene.padding, fill=False, edgecolor=orange, linestyle=':', zorder=4))
    ax.scatter(*start, s=70, color=blue, edgecolor='white', zorder=8)
    ax.scatter(*wait, s=130, marker='D', color=orange, edgecolor='white', zorder=8, label='2. Door interaction')
    ax.scatter(*exit_point, s=45, color=green, edgecolor='white', zorder=8)
    ax.scatter(*goal, s=200, marker='*', color=green, edgecolor='white', zorder=8)
    ax.annotate('START', start, xytext=(-12, -25), textcoords='offset points', weight='bold', color=blue)
    ax.annotate('GOAL / STOP', goal, xytext=(10, 10), textcoords='offset points', weight='bold', color=green)
    ax.annotate('Door exit', exit_point, xytext=(20, 0), textcoords='offset points', color=green)
    text = '2  STOP → REQUEST OPEN → CHECK' if result['scenario'] != 'open' else '2  STOP → CHECK (already open)'
    ax.annotate(text, wait, xytext=(.8, -1.35), textcoords='data', fontsize=10, color='#9a5100',
                bbox={'boxstyle': 'round,pad=.5', 'fc': '#fff5e8', 'ec': orange},
                arrowprops={'arrowstyle': '->', 'color': orange}, zorder=9)
    ax.text(-3.7, -2.65, 'ROOM A', color='#87929f', fontsize=13, weight='bold')
    ax.text(-3.7, 2.5, 'ROOM B', color='#87929f', fontsize=13, weight='bold')
    ax.set(xlim=(-4.25, 4.25), ylim=(-3.25, 3.25), xlabel='X (m)', ylabel='Y (m)')
    ax.set_aspect('equal')
    ax.grid(alpha=.12)
    fig.suptitle('INTERACTIVE NAVIGATION | ONE-MAP PLAN', fontsize=17, weight='bold', y=.97)
    fig.text(.5, .925, f"Scenario: {result['scenario']}  •  Blue: before interaction  •  Orange: open door  •  Green: after interaction",
             ha='center', fontsize=10)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(.5, .075), ncol=3, frameon=False, fontsize=9)
    fig.text(.5, .065, 'Green route executes ONLY after measured door angle and corridor clearance checks pass.',
             ha='center', fontsize=10, color='#126b49')
    fig.text(.5, .035, 'Gray halo: clearance at planned open angle. Dashed door: closed reference. This is a plan, not a recorded trajectory.',
             ha='center', fontsize=9, color='#596273')
    if result['errors']:
        ax.text(.02, .98, '\n'.join(result['errors']), transform=ax.transAxes, va='top', fontsize=9,
                color='#b32323', bbox={'facecolor': '#fff1ef', 'edgecolor': '#b32323', 'alpha': .97}, zorder=10, wrap=True)
    return fig


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'interaction_nav.yaml')
    parser.add_argument('--scenario', choices=('closed', 'open', 'jammed', 'blocked'))
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'outputs/interaction_planning')
    parser.add_argument('--no-show', action='store_true')
    args = parser.parse_args(argv)
    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    config = load_config(args.config, args.scenario)
    scene, result = plan_interaction(config)
    fig = draw_plan(scene, result)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.output_dir / ('interaction_plan_' + config['scenario'])
    png = stem.with_suffix('.png').resolve()
    try:
        fig.savefig(png, dpi=180)
        stem.with_suffix('.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print('Plan image:', png)
        print('Plan data:', stem.with_suffix('.json').resolve())
        for error in result['errors']:
            print('Plan warning:', error)
        if not args.no_show:
            if str(matplotlib.get_backend()).lower() == 'agg':
                if os.name == 'nt':
                    os.startfile(str(png))
                else:
                    print('No interactive backend; view the saved PNG or configure Tk/Qt.')
            else:
                plt.show(block=True)
    finally:
        plt.close(fig)
    return png


if __name__ == '__main__':
    main()
