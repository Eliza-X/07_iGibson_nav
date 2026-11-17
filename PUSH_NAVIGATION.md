# Single-box closed-loop navigation

Run in the same iGibson environment used for the successful push probe:

```powershell
cd D:\Projects\07_iGibson_nav
python run_push_navigation.py --plan-only
python run_push_navigation.py
python run_push_navigation.py --headless --repeats 3
```

Visual mode starts paused: Space starts/pauses, Esc exits. Default playback
is 0.5x; use `--playback-speed 1` for real time. Closing a finished trial
continues the next repeat. Closing during a trial ends the repeat sequence.
The final frame stays visible after results have been saved.

The map is an 8 by 6 metre room with a full dividing wall and a 0.9 m opening.
The original probe box blocks that opening. Robot starts at (-2.5, 0), box at
(0, 0), and goal at (2.5, 0). The robot approaches (-0.85, 0), aligns with +X,
pushes the box at least 1.3 m, stops, and withdraws to x=0.55. It then updates
the box occupancy, runs the existing A*, and follows existing waypoint/action
control around the box to the goal. Success requires stable final stopping.

The 1.3 m push is a scene-specific starting configuration, not a physically
validated guarantee. `--plan-only` checks initial disconnection and a predicted
post-push path without running physics. Actual box pose and orientation are
used for post-push planning. No object teleportation occurs during execution.
The physical robot footprint is checked against the configured planning radius
at startup. The check uses conservative link AABB corners; if it rejects the
radius, inspect geometry and adjust the radius/layout together, not just remove
the check.

Configuration inherits `push_probe.yaml`, then applies `push_navigation.yaml`.
This retains the probe's box and pushing defaults. The first version assumes
world +X straight pushing and a centered box; it does not retry failed pushes.
Wall contacts, tilt, stall, lost contact and unsafe release stop the task.
Only controlled contact/withdrawal phases permit box contact. Release checks
walls independently while withdrawing from the box's inflated footprint.
Ordinary navigation treats the box as an obstacle. Blocked navigation stops
before bounded replanning. Floor IDs are captured before walls are imported.

Logs: `outputs/push_navigation/<timestamp>/` contains effective configuration,
predicted layout paths, per-trial geometry/assets, step observations and final
result with actual planned paths and replan count. JSONL observations are taken
before each action. Simulation state-machine tests are synthetic feedback tests;
they do not establish physical success. Run at least three actual trials after
the first visual check. Existing door and standalone probe entry points remain.

## Corner tracking

This task enables mandatory direction-change waypoints. Ordinary waypoint
tolerance remains 0.06 m; corners require 0.025 m. Every attempted advance
checks the actual-position-to-next-target segment. If that connection is
blocked, retain the current target and refine toward it with 0.005 m tolerance.
Unsafe current segments, exhausted precision or 8 seconds of continuous
refinement trigger a stopped, bounded replan (or failure during approach).
Reaching a corner issues one zero-action cycle before continuing; it is not
a claim of fully settled physical motion. Existing heading control then turns
toward the next target. These values still require physical validation.

Step logs include `tracking`: selected waypoint/index, critical flag, distance,
consumed indices, refinement/pause flags, proposed blocked target, first blocked
sample/cell, and decision. Other tasks retain the original tracker defaults.
