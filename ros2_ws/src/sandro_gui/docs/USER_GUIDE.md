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
| L1 + Triangle | Switch to the next policy (rl_flat_level → rl_agile_terrain). The robot first settles at 0.17 m with zero speed and roll, then the incoming policy takes over in one control cycle. Releasing L1 or the E-stop aborts the switch (gives up after 8 s). |
| Circle | Rumble the running policy's number: one pulse for the first, two for the second. One long rumble means a switch was refused or aborted. |
| L2 + R2 | Software E-stop: squeeze both triggers past 75%. Deactivates every controller (all torques to zero) and sends the CAN E-stop to the drives. It is not a substitute for the power cut-off. |
| hold Create + Options | Re-arm after an E-stop, in two holds of 2 s each; only once the drives have been armed in this launch (the first arming is the operator's, on the panel). First hold: the drives are armed at zero torque with no controller. Hold the robot upright, then hold again: the controller that ran last starts again (or whole_body_controller). A fallen controller restarts with a single hold. |

Light bar colour:

- rgb(0, 255, 0): rl_flat_level
- rgb(255, 100, 0): rl_agile_terrain
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
read after the fact.
