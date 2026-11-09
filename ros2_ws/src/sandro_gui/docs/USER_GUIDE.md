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
4. **IMU with the drives off (optional).** Run → *IMU check* → Start. Only
   the BNO085 plugin is loaded. The IMU page should show about 240 new
   samples per second, tilt near 0° on a level floor, and positive pitch when
   the robot tips forward. Stop it before the next step.
5. **Arm.** Run → *Armed*, controller *none*, gamepad teleop ticked → Start,
   and confirm. The drives hold zero torque. Overview: joint states at 300 Hz,
   the IMU fresh, and the 3D legs following when a leg is moved by hand.
6. **Start the controller.** Hold the robot upright, then Start it in Run →
   Controllers, or hold Create + Options for 2 s on the pad. It engages once
   the robot is upright and still.
7. **Drive.** Hold L1 and use the sticks (below). The Run page shows the
   controller's live diagnostics.
8. **Stop.** The header's E-STOP or L2 + R2 for an immediate stop. Run →
   Stop for a normal shutdown: one SIGINT, so the drives go to zero torque,
   idle and deadman off before the launch exits. A balancing robot falls.

Steps 5 and 6 can be one: pick the controller in the Start form directly.

## Gamepad

Teleop runs when *Gamepad teleop* is ticked, and on the real robot only once
the drives are armed. Full stick deflection is the running controller's own
limit, read from its parameters, so it means the same on every policy.

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
| hold Create + Options | Re-arm after an E-stop, in two holds of 2 s each. First hold: the drives are armed at zero torque with no controller. Hold the robot upright, then hold again: the selected policy (or whole_body_controller) starts. |

Light bar colour:

- rgb(0, 255, 0): rl_flat_level
- rgb(255, 100, 0): rl_agile_terrain
- rgb(255, 0, 0): no controller running
- rgb(255, 255, 255): a controller outside the switch list

During a policy switch the light bar blinks between the two colours, and the
player LEDs light up to the running policy's position in the list. The table
above is generated from `sandro_bringup/config/teleop.yaml`
(`python3 -m sandro_gui.main --print-guide`); a test fails when they differ.

## E-stop and re-arm

Both software E-stops (the header button, L2 + R2) deactivate every
controller, so all torques go to zero, and send the CAN E-stop. The drives
then latch `ESTOP_REQUESTED`. To continue, arm again (Run → Hardware → Arm,
or hold Create + Options for 2 s), then start the controller (Run page, or
hold Create + Options again with the robot held upright). Software stops ride
on the same bus and computer that just misbehaved: the physical cut-off is
the real E-stop.

## When something is off

| Symptom | What to check |
|---|---|
| A drive is silent | Its heartbeat age grows in the Drives table. Check that drive's CAN cable and power; arming refuses while any drive is missing. |
| CAN bus-off or error-passive | Bit rate or sample point, a missing terminator, or a drive powered down mid-run. `can0.service` sets 1 Mbit/s at boot; the bus error counters are in the CAN check. |
| IMU stale, `IMU FAULT` in the log | Run the IMU check with the drives off. SPI6: INT on GPIO25, RST on GPIO24, P0 and P1 tied high (`deploy/pi/README.md`). |
| Arm refused | The Run log names the drive and its axis error. Only the E-stop and watchdog bits clear on arming. A MIT frame bias above 0.2 rad means a drive zero is off. |
| Gamepad does not connect | Gamepad page: unblock the radio if rfkill blocks it, scan, and pair while the scan still runs. Once paired, PS reconnects it. |
| A policy oscillates | The RL controllers cut the drives on a 5-15 Hz oscillation (`RL_OSCILLATION` in the log). Re-arm and start from a calmer pose; check the leg zeros if it repeats. |

Every launch's output is kept on the Pi in `~/sandro/gui_runs/run-*.log`
(Run → Log → Past runs), so a run can be read after the fact.
