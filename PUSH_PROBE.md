# Straight pushing probe

This experiment is independent of door traversal. A Turtlebot pushes a free
dynamic box along world +X. No box pose resets or external box forces occur
during control. Actions are normalized commands, not metres per second.

Run from the project root in an environment containing iGibson and its assets:

```powershell
python run_push_probe.py
python run_push_probe.py --playback-speed 1
python run_push_probe.py --headless --repeats 3
python run_push_probe.py --headless --mass 0.5 --repeats 3
python run_push_probe.py --headless --mass 2 --repeats 3
```

Visual mode opens a resizable fixed oblique side view, initially paused.
Press Space to start/pause and Esc (or close the window) to exit. Default
playback is 0.5x; `--playback-speed 1` requests real time. Rendering overhead
can make playback slower. Playback pacing never changes the physics timestep.
The status panel shows controller state, contact, displacement, lateral drift,
simulated time and result. The final frame stays frozen until closed; results
are saved before that wait. Closing during a trial records `user_closed` and
ends the repeat sequence. Close a completed trial to start the next repeat.
Headless runs have no UI waits or playback delay.

The custom display uses Tkinter and Pillow with the iGibson offscreen renderer,
as the project's scene preview does. It replaces the native viewer only for
this probe. No camera tracking or physics parameters are changed.

The initial box position uses the robot's measured AABB and configured gap.
Check `geometry.json` and the first visual run to verify chassis contact height.
The box dimensions and friction are starting hypotheses, not calibrated values.
The empty scene's original bodies are treated as the supporting floor; this
contact policy must be changed before adding walls or other obstacles.

The controller waits for settling, approaches until contact, pushes, stops,
and verifies final displacement. It fails on drift, tilt, no contact, lost
contact, stalled progress, unexpected collision or timeout. A successful
trial requires 0.5 m displacement, <=0.1 m lateral drift and <=15 degrees yaw
change with final stable velocities. See `push_probe.yaml` for thresholds.

Outputs are in `outputs/push_probe/<timestamp>/`: config, per-trial URDF,
geometry, JSONL observations/actions/contact link IDs and forces, and results.
Each JSONL observation is sampled before its logged action. Final observations
are also recorded. Inspect robot link IDs to confirm the intended body contact.
Three repeat successes are the initial stability check, not a general guarantee.

Logic tests do not establish physical pushability:

```powershell
python -m pytest tests/test_push_probe.py tests/test_door_bypass.py -q
```
