# Sandro user guide

The robot serves its control panel on **http://10.42.0.169:8080** (the Pi's
address on the shared connection; any address of the Pi works). It starts
with the Pi: power on, wait about a minute, open the page. The same guide is
on the panel's Guide page, with a gamepad drawing that lights the buttons of
each combination.

## Every start

1. **Power and secure.** Robot lifted or on its stand, drives powered, the
   power cut-off in reach.
2. **Bus and drives.** Overview: CAN bus *error-active* at 1 Mbit/s and
   Drives *6/6 heard*, before anything is launched (the panel listens to the
   bus passively). A silent drive shows its node and how long it has been
   quiet. Axis error `0x104000` is the E-stop latch, which arming clears; any
   other code must be inspected first.
3. **Gamepad.** Press PS: the Gamepad check turns green with the battery
   level. A new pad is paired on the Gamepad page.
4. **Start the stack.** Run → pick the controller to preload, gamepad teleop
   ticked → *Start stack*. One launch serves the whole session: the drives
   come up configured but not armed, and the controller is loaded ready to
   start. The Stack check reads *starting* until the hardware, the
   broadcasters and the controller are there, then *drives not armed*.
   Overview: joint states at 300 Hz, the IMU fresh, and the 3D legs following
   when a leg is moved by hand.
5. **Arm and start.** Hold the robot upright (lay it down for
   `rl_stand_up`, which stands up by itself), then Run → *Arm and start* and
   confirm. The first arm of a launch also asks you to tick that the robot is
   secured. *Arm* and *Start* can be separate steps: armed with no
   controller, the drives hold zero torque.
6. **Drive.** Hold L1 and use the sticks (below). The Run page shows the
   controller's live diagnostics.
7. **Stop.** The header's E-STOP or L2 + R2 for an immediate stop. Run →
   Stop for a normal shutdown: one SIGINT, so the drives go to zero torque,
   idle and deadman off before the launch exits. A balancing robot falls.

**IMU alone (optional, diagnostic).** Run → Advanced → Launch *IMU check* →
Start IMU check. Only the BNO085 plugin is loaded, no drive plugin. The IMU
page should show about 240 new samples per second, tilt near 0° on a level
floor, and positive pitch when the robot tips forward. Stop it before
starting the stack.

Every action that moves the robot (start, restart, swap, arm and start) asks
one confirmation; stopping a controller, disarming and stopping the stack ask
none. A button spins while its request runs, and a second action waits until
the first is done.

## Gamepad

Teleop runs when *Gamepad teleop* is ticked; it starts with the stack, and
drives only once a controller runs. Full stick deflection is the running
controller's own limit, read from its parameters, so it means the same on
every policy.

| Buttons | What it does |
|---|---|
| hold L1 | Deadman. Driving commands are sent only while it is held; releasing it sends zero speed once and the controller holds position. |
| L1 + left stick up/down | Forward and backward speed. Full deflection is the running controller's own speed limit times speed_fraction (1). |
| L1 + left stick left/right | Turn (yaw rate); stick left turns left. |
| L1 + right stick up/down | Body height up and down (a height rate: the body stays where you leave it). |
| L1 + right stick left/right | Body roll, stick right puts the right side down. Only policies with a roll command (rl_agile_terrain) use it. |
| L1 + R3 | Height back to the running policy's initial height. |
| L1 + Cross | One hop, about 5 cm of wheel clearance, standing or driving. Only policies with a hop input (rl_agile_step) use it. |
| L1 + Triangle | Switch to the next policy (rl_flat_level → rl_agile_terrain → rl_flat_level_fast → rl_agile_terrain_v3 → rl_agile_terrain_v5 → rl_agile_terrain_v6 → rl_agile_step). The robot first settles at 0.17 m with zero speed and roll, then the incoming policy takes over in one control cycle. Releasing L1 or the E-stop aborts the switch (gives up after 8 s). |
| Circle | Rumble the running policy's number: one pulse for the first, two for the second. One long rumble means a switch was refused or aborted. |
| L2 + R2 | Software E-stop: squeeze both triggers past 75%. Deactivates every controller (all torques to zero) and sends the CAN E-stop to the drives. It is not a substitute for the power cut-off. |
| hold Create + Options | Re-arm after an E-stop, in two holds of 2 s each; only once the drives have been armed in this launch (the first arming is the operator's, on the panel). First hold: the drives are armed at zero torque with no controller. Hold the robot upright, then hold again: the controller that ran last starts again (or whole_body_controller). A fallen controller restarts with a single hold. |

Light bar colour:

- rgb(0, 255, 0): rl_flat_level
- rgb(255, 100, 0): rl_agile_terrain
- rgb(0, 80, 255): rl_flat_level_fast
- rgb(255, 220, 0): rl_agile_terrain_v3
- rgb(180, 0, 255): rl_agile_terrain_v5
- rgb(0, 255, 255): rl_agile_terrain_v6
- rgb(255, 0, 120): rl_agile_step
- rgb(255, 0, 0): no controller running
- rgb(255, 255, 255): a controller outside the switch list

During a policy switch the light bar blinks between the two colours, and the
player LEDs light up to the running policy's position in the list. The table
above is generated from `sandro_bringup/config/teleop.yaml`
(`python3 -m sandro_gui.main --print-guide`); a test fails when they differ.

Swapping controllers on the Run page while one runs is a single controller
switch. When the incoming controller is an RL policy and a balancer runs, the
panel arms the policy's handover first, as L1 + Triangle does: an upright
robot is taken over at once, otherwise the policy starts its stand phase.
L1 + Triangle is still the soft way, since it settles the robot first.

## Driving from a phone

Without the gamepad, for example outside with the robot on the phone's
hotspot: open the panel on the phone (`http://<robot>:8080`) and go to
**Drive**. Start the stack, arm and start a controller as always (Run page);
the stack must run with teleop (the default).

| On the page | What it does |
|---|---|
| Left stick | Forward and back, turn. Touching a stick is holding L1: the robot drives only while a finger is down. |
| Right stick | Body height up and down, roll left and right. |
| Hop | One hop (policies with a hop input). |
| Next policy (hold) | Switches to the next policy of the gamepad's list. Keep holding until it is done; letting go aborts it. |
| Height reset (hold) | Height back to the running policy's initial height. |
| Speed | Scales the left stick, as a fraction of the running controller's limits. Start low. |
| Routines | Hold a routine's button and the panel moves the sticks; let go and the robot stops at once. Each press starts from the beginning. |

The routines (`config/routines.yaml`; the figures are for 50 % speed on a
policy with 0.5 m/s and 1.5 rad/s):

| Routine | Lasts | What the robot does | Room |
|---|---|---|---|
| Greeting | 9 s | Two bows of 2.4 cm, then a look left, right and back, about 25° each side. | On the spot. |
| Spin | 12 s | A turn to the right, a pause, the same turn back: about 200° each (more than a full turn at 100 %). | 0.5 m clear all round. |
| Sway | 10 s | Leans right and left three times, dipping into each lean. Policies without roll only dip. | On the spot. |
| Figure eight | 18 s | A circle to the right, then one to the left, each about 0.55 m across. The circles close at 50 % speed only. | 1.5 m by 1 m, starting in the middle facing along the short side. |
| Show | 29 s | Bows, a turn, a hop, 0.3 m forward with a hop and back, a sway, the turn back, two hops. | 1 m ahead, 0.5 m all round. |

- A routine is the sticks and the hop, nothing more: it works with any
  policy, and a policy without roll, height or hop skips those parts. The
  speed slider scales its driving and turning like the left stick's.
- Routines with bows start with the height reset, so they begin at the
  policy's own height; the first second looks like nothing is happening.
- First run of each routine: robot held or on the stand, low speed. Nothing
  in a routine knows where the walls are.
- While a routine's button is held the sticks on the page do nothing.

- The robot stops when the finger lifts. If the phone loses the robot, the
  screen locks or the browser goes to the background, it stops by itself
  within about half a second and the page says "link lost".
- Hold the phone in landscape for larger sticks. Keep the screen on.
- A connected gamepad has priority: the page is refused while one is
  connected. Only one browser drives at a time.
- E-STOP is the red button at the top of every page. Re-arm from the Run
  page; the page has no E-stop chord and cannot re-arm.
- The number next to the status is the round trip of a command to the robot
  in milliseconds. Above 150 ms, slow down.

## E-stop and re-arm

The header E-STOP sends the CAN E-stop from the panel's own CAN socket first,
so it reaches the drives even with no launch and no ROS; then, at the same
time, through the E-stop node and by deactivating every controller. L2 + R2
deactivates every controller and sends the CAN E-stop through the E-stop
node. The drives latch `ESTOP_REQUESTED`, the hardware drops out, and the
Stack check reads *E-stopped*. To continue, Run → *Re-arm and restart*
(one confirmation: it arms the drives and starts the controller that ran
last), or *Re-arm only*, or hold Create + Options twice on the pad. Software
stops ride on the same bus and computer that just misbehaved: the physical
cut-off is the real E-stop.

A controller that fell or latched (a policy's fall cut-off or oscillation
watchdog, a balancer's fall latch) shows *fallen* in red on the Run page and
in the Controller check. Put the robot back, then *Restart* it: one
confirmation, it is deactivated and activated again (or one hold of
Create + Options on the pad).

## When something is off

| Symptom | What to check |
|---|---|
| A drive is silent | Its heartbeat age grows in the Drives table. Check that drive's CAN cable and power; arming refuses while any drive is missing. |
| CAN bus-off or error-passive | Bit rate or sample point, a missing terminator, or a drive powered down mid-run. `can0.service` sets 1 Mbit/s at boot; the bus error counters are in the CAN check. |
| IMU stale, `IMU FAULT` in the log | Run the IMU check (Run → Advanced) with the drives off. SPI6: INT on GPIO25, RST on GPIO24, P0 and P1 tied high (`deploy/pi/README.md`). |
| The stack stays *starting* | The Run page lists what it still waits for. *controller manager died* means `ros2_control_node` exited (the Run log says why, usually a drive that refused to configure): Stop, then Start again. |
| Arm refused | The message names each drive, its axis error and whether it is clearable. The E-stop and watchdog bits clear on arming, or with *Clear drive errors*; any other bit is a drive fault: inspect it, then power-cycle the drive. A MIT frame bias above 0.2 rad means a drive zero is off. |
| A controller shows *fallen* | It cut the drives after a fall or an oscillation and stays latched. Put the robot upright (lying for `rl_stand_up`) and Restart it on the Run page. |
| Gamepad does not connect | Gamepad page: unblock the radio if rfkill blocks it, scan, and pair while the scan still runs. Once paired, PS reconnects it. |
| A policy oscillates | The RL controllers cut the drives on a 5-15 Hz oscillation (`RL_OSCILLATION` in the log) and show *fallen*. Restart the controller from a calmer pose; check the leg zeros if it repeats. |

Every launch's output is kept on the Pi in `~/sandro/gui_runs/run-*.log`
(Run → Log → Past runs), the last 30 runs and at most 200 MB, so a run can be
read after the fact. Its first lines name the commit it ran.

## Camera

The D435 and the terrain profile start with the stack only when **Depth
camera** is ticked on the Run page (`camera:=true`). Without it
`rl_agile_step` drives blind: it neither hops onto steps nor stops at blocks.

The Overview's 3D view draws what the policy gets: a ribbon ahead of each
wheel through the floor heights of the terrain profile, relative to the floor
under that wheel, interpolated between its eight distances. Green is flat,
amber to red a step, blue a drop; a grey ring is a point the camera has not
seen (the policy takes it as flat). **Terrain** above the view hides it. The
**Camera** panel below has the same heights in cm and the camera's readings.
To check it, put a book 0.8 m ahead of one wheel: it appears there in the
ribbon, and stays as you roll up to it. On a flat floor the readings' floor
pitch, roll and offset are the mount's error and should be near zero.

## Updating the robot

Software page, with the stack stopped: pick the branch (pushed from the laptop
with `deploy/pi/pi push`, or *Fetch from GitHub* when the robot has internet)
and *Deploy* a commit. The panel writes it, builds what changed, restarts and
the page reloads; the Versions table and the SW chip then read the new commit.
A yellow SW chip says what is out of step: source not built, panel or stack
started from an older build.

## Measurements (Sysid page)

Three runs, each one button. A greyed Start says next to it what is missing.

**Yaw response to 45 Hz** (about 75 s): what the RL training holds a policy's
yaw loop against.

1. Run page: start the stack with `whole_body_controller`, Arm and start.
2. Flat floor, a metre of room, hands off the sticks.
3. Sysid page: Start, tick the box. The wheels twitch against each other,
   faster and faster, twice (0.2 and 0.4 Nm).
4. In Results the run should read *good*. Download **Plant table**
   (`yaw_plant_<time>.json`), put it in `sandro_rl/data/` on the training
   machine and set `train.loop_margin.plant` to its name.
5. Optional, each its own run: crouch to the lowest height with the pad and
   repeat, then the highest.

**Roll and yaw response, wheels and legs** (about 3 min 20 s): the plant
behind both buzzes, the 9 Hz yaw one through the wheels and the 14 Hz roll one
through the legs. It needs the software with the leg sweeps on the robot
(Software page: deploy, then start the stack again).

1. Run page: start the stack with `whole_body_controller`, Arm and start.
2. Flat floor, a metre of room, the default standing height, hands off the
   sticks.
3. Sysid page: Start, tick the box. Six sweeps follow, named in the large
   line: the wheels twitch (twice), the body rocks side to side by up to
   about 2 degrees (twice), the legs scissor fore-aft (twice).
4. Results should read *good* with how far each of yaw, roll and fore-aft
   answered cleanly. Download **Opposed plant** (`opposed_plant_<time>.json`),
   put it in `sandro_rl/data/` on the training machine and set
   `train.loop_margin.plant` to its name. **Recording** downloads the whole
   run as a `.tar` for `recordings/`.

If Start answers that this whole_body_controller cannot sweep the legs, the
running stack is older than the page: stop it, deploy, start it again.

**Record a controller's buzz** (about 95 s): where a buzz is.

1. Run page: start the controller under test (for example
   `rl_flat_level_sysid`) and let it stand.
2. Sysid page: Start, tick the box, then do what the large line says: stand
   still 30 s, drive and turn gently 30 s, stand still 30 s.
3. Results gives the verdict (quiet, or the mode, its size and frequency);
   **Recording** downloads the whole run as a `.tar` for `recordings/`.

**Record a policy trial** (open, 15 minutes at most): which policy did what.

1. Start the stack. Sysid page: Start, tick the box. Then start the first
   controller, so that its start is recorded too.
2. Try the policies as you like, from the Run page or the gamepad. The large
   line names the controller being recorded and changes with every switch.
   For a fair comparison give each one 20 s standing still and some driving.
3. **Stop measurement** ends the recording. The E-stop does not: a fall and
   what followed belong to the trial.
4. Results has one line per stay of a controller: how long, quiet or buzzing
   while standing (size, axis, frequency), how far it drove, how long it lay
   on the floor. **Recording** downloads the run as a `.tar`: untar it under
   `recordings/policy_trials/` (its README says what is inside) and add what
   you saw and heard to `notes.md` beside it.

Stop measurement ends a run cleanly at any time; so does the E-stop. A run
that ends early keeps what it recorded.
