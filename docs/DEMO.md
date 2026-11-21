# A live demo with Sandro

What the robot can show, in what order, and what to check first. Written
2026-11-21 from simulation results only: **none of the five demo policies has
run on the robot.** Each needs a first try with the robot held before it goes
in front of people.

The simulated version of the whole thing is
[`sandro_rl/docs/demo_reel.mp4`](../sandro_rl/docs/demo_reel.mp4) (85 s).

## Before the demo

Do these at home, not at the venue.

1. **Rebuild on the robot** (`make pi-launch` syncs and builds). The demo
   policies, the Drive page and the routines are not on it yet. The
   controller's new terrain diagnostics and the depth preview have never been
   compiled: if the build fails there, that is where to look.
2. **Charge the battery.** The hops and the one-wheel stand push the leg
   drives to their 9 Nm peak.
3. **Phone.** Robot on the phone's hotspot, panel open at
   `http://<robot>:8080`, **Drive** tab. Check that a stick drives, that the
   robot stops when the finger lifts, and that it stops when the screen
   locks. A paired gamepad has priority over the phone: switch it off to
   drive from the phone.
4. **First try of each policy, robot held, E-stop at hand,** in the order of
   the table below (safest first). Speed slider at 30 % for the first tries.
5. **Record the first tries:**
   `python3 ros2_ws/src/sandro_bringup/tools/policy_record.py` on the robot
   while you try them. If something does not behave like the reel, the
   recording says why.
6. Drop from the running order whatever did not work held. Every item below
   stands on its own.

### Order of the first tries

| # | Controller | Why here |
|---|---|---|
| 1 | `rl_agile_terrain_v6` | Known on the robot. Check the Drive page and the Greeting and Figure eight routines with it. `rl_agile_terrain_v7` (panel list, never run on the robot) is the same policy trained with the drives' speed limit modelled: in simulation a shove at full stick fells v6 in 3 runs of 18 under that limit and v7 in none. Until v7 has been tried, keep v6 at 0.8 of the stick or less. Keep both on flat floor: with the camera fitted v6 crosses the 2.5 cm plank course in 11 simulated runs of 27, not the 27 of its record (`sandro_rl/docs/DESIGN.md` 28). |
| 2 | `rl_fun_bow` | Slow, low torque (4.9 Nm), wheels under 10 rad/s standing. |
| 3 | `rl_fun_stride` | Low torque (3.6 Nm), but the legs move fast and it has no margin for edges. |
| 4 | `rl_fun_show` | Fast spins, hops of 5–7 cm at up to 8.2 Nm. Start with the slider low: full stick is 5.5 rad/s of spin. |
| 5 | `rl_fun_hop_high` | The hardest landing of any policy: 9 Nm, down at 1.2–1.8 m/s. |
| 6 | `rl_fun_one_leg_v4` | Balances 0.2 rad from the angle at which the controller declares a fall. The likeliest to fall. |

## How to run an item

1. **Run** page: start the stack, Arm, start the controller named in the item
   (the demo policies are in the panel's list, not on the gamepad's L1 +
   Triangle switch). Changing from one to the next on the Run page is a
   single switch while the robot stands still; let it settle first.
2. **Drive** page: set the speed slider, then either drive with the sticks or
   hold a routine's button. A routine runs only while the finger holds it;
   letting go stops the robot at once. Each press starts from the beginning.
3. The gamepad works the same way with L1 held (L1 + Cross is the hop);
   routines are on the phone only.

## Suggested running order (about 3½ minutes)

| | Item | Controller | Do | Lasts |
|---|---|---|---|---|
| 1 | Hello | `rl_fun_show` | Routine **Greeting** | 9 s |
| 2 | The show | `rl_fun_show` | Routine **Show** at 50 % | 29 s |
| 3 | Free driving | `rl_fun_show` | Sticks: a pirouette, a sway, a hop on request | 30 s |
| 4 | Walking | `rl_fun_stride` | Routine **Stride**, then stride by hand with the right stick | 30 s |
| 5 | The bow | `rl_fun_bow` | Routine **Bow**; nod with the right stick | 20 s |
| 6 | High hops | `rl_fun_hop_high` | Routine **Hops** | 7 s |
| 7 | One wheel | `rl_fun_one_leg_v4` | **Hop** button once, hands off for 8 s | 15 s |
| 8 | Goodbye | `rl_fun_bow` | Routine **Bow** | 7 s |

Each change of controller takes a few seconds on the Run page; say something
meanwhile. If time is short, 1, 2, 4 and 5 are the safest four.

## The items

Floor space is what the routine used in simulation at 50 % speed, on the
stock model and on the model fitted to the robot's drive train, with the
drives' speed limit modelled (`sandro_rl/scripts/routine_runs.py`). No routine
fell at 50 % or 100 % speed, and no wheel passed the drives' 19.5 rad/s.

### `rl_fun_show`: spins, sway, bob, hop

- **Sticks:** left as always, but full stick sideways is a 5.5 rad/s
  pirouette. Right stick: height (fast) and roll up to 0.4 rad. Hop button:
  one hop of 5–7 cm, also in the middle of a spin.
- **Routines:** Greeting, Spin, Sway, Figure eight, Show.
- **Floor:** 1.5 m by 1.5 m, clear, flat. The Show routine used up to 0.9 m by
  0.7 m and ended up to 0.5 m from where it started; Greeting and Sway creep
  about 0.3 m forward; the figure eight used 0.8 m by 0.65 m.
- **In simulation:** no fall in 18 runs; hops 36 of 36; it stops a spin within
  1 s.
- **Caveats:** the spin depends on the floor: 5 rad/s where the tyre twists
  easily, about 2.5 on a grippy one. Its straight-line speed is only
  0.65 m/s. Several hops in quick succession may trip the oscillation
  watchdog (it waits 1.2 s per hop); the Show routine spaces them.

### `rl_fun_stride`: the legs scissor like a walk

- **Sticks:** right stick left-right is the legs' fore-aft split: right puts
  the right wheel ahead (up to 16 cm apart). Sweep it and the robot strides,
  standing or driving. There is no roll on this policy.
- **Routine:** Stride (three strides on the spot, three driving forward,
  three driving back).
- **Floor:** a clear strip 1.5 m long at 50 % speed (2.2 m at 100 %); it ends
  about 0.45 m ahead of where it started. **Flat and smooth only.**
- **In simulation:** no fall in 18 runs on flat floor; strides of ±11 cm at
  0.5 Hz; only the tyres touch the floor.
- **Caveats:** it fell at a 2 cm edge in 6 runs of 6: no cables, door sills or
  carpet edges. Its heading wiggles while it strides. The stride itself is
  close to what the oscillation watchdog looks for (0.81 of its threshold),
  so the watchdog waits 0.5 s on this policy; if the controller cuts out in
  the middle of a stride on the robot, that is the first suspect.

### `rl_fun_bow`: a bow and a nod

- **Sticks:** right stick left-right is the body's lean: **right is
  forward**, up to 0.6 rad each way. The Run page's diagnostics show this as
  "roll", 0.25 at the full bow.
- **Routine:** Bow (a slow deep bow, held 1.5 s, upright, two nods).
- **Floor:** on the spot; it rolls about 0.15 m back and forth under the
  bow. Keep 0.5 m clear in front.
- **In simulation:** no fall in 18 runs; 0.56 rad held forward, 0.60 back;
  only the tyres touch, the knees stay 11 cm up.
- **Caveats:** while driving, reverse the lean gently: a quick reversal at
  speed sent the wheels to 24 rad/s in one simulated case, past the drives'
  limit. Standing, there is no such problem. Not measured on obstacles.

### `rl_fun_hop_high`: 11–13 cm hops

- **Sticks:** as a normal driving policy; the Hop button is the high hop,
  whatever the height asked.
- **Routine:** Hops (four, one every 1.5 s).
- **Floor:** 1 m clear all round, flat, and not slippery: the four hops
  carried it up to 0.6 m forward and 0.3 m sideways.
- **In simulation:** 108 hops of 108, no fall, no bounce on landing; three in
  a row work.
- **Caveats:** this is the hardest thing asked of the hardware so far. The
  legs push at the drives' full 9 Nm peak and at 17 rad/s (limit 19.5), and
  the robot comes down at 1.2–1.8 m/s. Check the leg screws and the battery
  mount before and after the first try. It stands 1–2.5 cm lower than the
  height asked.

### `rl_fun_one_leg_v4`: eight seconds on the left wheel

- **Sticks:** Hop button once; the robot rolls onto its left tyre, lifts the
  right wheel and holds for 8 s, then comes back down. Leave the
  sticks alone meanwhile.
- **Routine:** none; one press.
- **Floor:** 2 m by 2 m, smooth. It balances by turning on the spot (about
  two turns) and wanders up to 0.9 m.
- **In simulation:** 54 requests of 54 held, no fall in 18 runs; on the model
  fitted to the robot, 27 holds of 27 without a single tap or hop.
- **Caveats:** the least certain item.
  - It balances at 0.8 rad of roll; the controller declares a fall at
    1.0 rad.
  - It holds cleanly only where the tyre twists easily on the floor. On the
    stock model's grippier tyre it makes about 7 small hops per hold.
  - The clean result was judged with the leg links' real meshes. With the
    simplified collision boxes switched on it fell in 18 runs of 18; the
    boxes are larger than the links, but that explanation has not been
    checked against the robot.
  - The raised wheel spins up to 25 rad/s in the air.

## Not for the demo

- **`rl_fun_fast` above half speed.** With the drives' speed limit modelled
  (19.5 rad/s, 1.3 m/s at the tyre; the limiter's exact behaviour is an
  assumption) it fell in 18 runs of 18 when asked for 1.5 or 2 m/s. Asked
  for 1 m/s it did not fall. Keep the speed slider (or `speed_fraction`) at
  0.5 or less on it.
- **A flip.** Not possible with these drives: in simulation it did not get
  off the floor with the real limits, and with twice the leg torque it
  turned 5.1–5.5 rad of a full 6.3 and crashed every time
  (`sandro_rl/docs/DESIGN.md` section 22).
- **`rl_fun_one_leg`, `rl_fun_one_leg_long`.** Superseded: they rock and tap
  the floor instead of standing.

## If something goes wrong

- **The robot does not move from the phone:** a gamepad is connected (it has
  priority), another browser is driving, or the stack runs without teleop.
  The Drive page's status line says which.
- **A controller shows *fallen*:** it cut the drives after a fall or an
  oscillation. Stand the robot up, Restart it on the Run page.
- **E-stop:** the panel's button on every page, or L2 + R2 on the gamepad.
  Neither replaces the power switch.

## Trying the routines in simulation

```bash
cd sandro_rl
# one routine with one policy, at the page's speed (0..1), with a video
MUJOCO_GL=egl uv run python -m sandro_rl.evaluate policies/fun_show --scenario routine:show:0.5 \
    --sim wheel_speed_limit=19.5 --video show.mp4
# every routine with its policy: falls, wheel speed, floor used
uv run python scripts/routine_runs.py
```

The evaluator turns the routine's sticks into commands as `joy_teleop` does
for the running controller (its limits, the height as a rate, the height
reset, the hop), so a change to `routines.yaml` can be checked here first.
