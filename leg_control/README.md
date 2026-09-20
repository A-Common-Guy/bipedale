# Five-bar leg control (impedance over CAN)

`leg_server.py` runs joint impedance control on the two GIM6010 drives and
serves the five-bar GUI (`../math/fivebar_viz.html`) over HTTP. The browser
is only a remote — targets in, telemetry out; every control decision (IK,
target slewing, torque limiting, watchdogs) runs in Python, so the same
`LegController` class can later be driven by gait code instead of the GUI.

```
tau_i = Kp (th_des_i − th_i) − Kd w_i      per joint, output-shaft Nm
```

Two control modes (`mode` in the config, or `--mode`):

- **`mit` (default)** — the impedance law runs **on the drive** via MIT
  frames (0x08): every cycle Python streams pos setpoint + Kp/Kd; the
  drive closes the loop at its internal rate, so CAN latency is out of
  the stiffness path. The torque clamp is enforced by capping the
  commanded position within ±tau_max/Kp of the measured one. The drive's
  `mit_max_*` limits must match `MitLimits` in `gim6010_can.py` (factory
  defaults: pos ±12.5 rad, vel 65, kp 500, kd 5, torque 50); kd above 5
  cannot be encoded in the MIT frame.
- **`torque`** — the original software loop: Python computes tau at
  200 Hz and streams `Set_Input_Torque`.

Both share the same safety net: torque clamp, target slew limit, joint
soft limits, overspeed kill, stale-feedback/heartbeat/drive-error
watchdogs, and the drives' own `break_timeout` dead-man in case this
process dies.

## Run

```bash
V=~/Personal/gim6010/control/venv/bin/python    # python-can + aiohttp

$V leg_server.py --sim                          # software motors, no hardware
$V leg_server.py --channel can0 --nodes 1 2     # the real leg (MIT mode)
$V leg_server.py --channel can0 --mode torque   # software impedance loop
```

Open http://localhost:8321 — the GUI detects the server and shows the
**Hardware** card (Enable torque / Disable / E-STOP / Clear errors, live
Kp/Kd/τ-limit, per-motor status). Solid linkage = measured pose, dashed
ghost = commanded target. Drag the foot to move the leg.

Before the first hardware run:
1. `cd ~/Personal/gim6010/can_control && ./canup.sh`
2. `gim_cli.py scan` — both nodes present?
3. `read_pos.py 1 2` while hand-moving the leg — check each motor reads
   **positive clockwise** and note the pose at zero.
4. Start the server, put the leg physically in the zero pose (both upper
   arms straight down), press **Zero here** in the GUI (stores rotor-rev
   offsets in `leg_config.json`).
5. Enable with the default gentle gains (Kp 8 Nm/rad, τ limit 2 Nm) and a
   hand on the e-stop.

## Frames

Leg frame = `math/ik.py`: +x right, +y down, motor 1 at (0,0), motor 2 at
(a,0); th=0 with the upper arm straight down; **th1 positive CW, th2
positive CCW**. Both real motors count positive clockwise with the same
zero, so `signs: [1, -1]` in `leg_config.json` maps hardware → leg frame.
Wire units are rotor rev / rotor Nm; `ratio: 8` converts (GIM6010-8).

## Config (`leg_config.json`, auto-created)

geometry `a/l1/l2` (cm) · `nodes` · `signs` · `ratio` · `offsets_rev`
(written by *Zero here*) · gains `kp/kd/tau_max` · `target_speed` (rad/s
target slew) · `vel_kill` · `th_min/th_max` · `loop_hz` (200) ·
`encoder_ms` (5 ms pos/vel broadcast) · `deadman_ms` (200).

Torque is hard-capped at 6 Nm output (`TAU_HARD_MAX`) no matter what the
GUI asks for.

## WebSocket API (`ws://host:8321/ws`, JSON)

→ `{"type":"enable"|"disable"|"estop"|"clear_errors"|"zero_here"}`
→ `{"type":"target_foot","x":4.5,"y":16}` (cm, leg frame)
→ `{"type":"target_joints","th1":0.2,"th2":0.2}` (rad)
→ `{"type":"gains","kp":8,"kd":0.5,"tau_max":2}`
← `{"type":"state", mode, th, w, tau, goal, loop_hz, motors[]}` @ 30 Hz
← `{"type":"config"|"ack"|"err", ...}`

## Test without hardware

`--sim` spins up two `sim_motor.py` emulators on a virtual CAN bus in the
same process — the full stack (CANSimple frames included) minus the wire.
The emulator implements the MIT impedance law too, so both modes are
testable end-to-end.

## Wheel bench: `wheel_mit_test.py`

One drive, MIT frames, **pure feedforward torque** (`kp = 0`), to settle
whether the drive wheel can use the same synchronous 0x08 path the leg
uses. The MIT position field is 16 bits over `±mit_max_pos` (±12.5 rad on
the output shaft = **±1.99 turns**), so a continuous wheel leaves it after
two revolutions — but the drive computes

```
tau = kp (pos_cmd − pos) + kd (vel_cmd − vel) + tau_ff
```

so with `kp = 0` the position field is multiplied by zero. What is left to
find out is what the *feedback* half of the frame does out there, and
whether the drive keeps producing torque. The script measures it: it
compares the echoed MIT position against the true unbounded multi-turn
position from 0x09 every cycle and prints a verdict (`RAIL` = clamps at
±mit_max_pos, `WRAP` = wraps modulo the window), plus the echoed torque
and measured Iq inside vs outside the window, and MIT round-trip latency.

```bash
V=~/Personal/Bipedal__wheeled/leg_control/venv/bin/python

$V wheel_mit_test.py --sim 1                      # emulated drive
$V wheel_mit_test.py --channel can0 3 --csv w.csv # node 3, real bus
$V wheel_mit_test.py -c can0 3 --kp 2 --pos-field sweep   # why kp>0 fails
$V wheel_mit_test.py -c can0 3 --mode torque      # Set_Input_Torque, for comparison
```

Nothing moves until asked: initial torque is 0. Keys: `+`/`-` torque by
`--step`, `0` zero, `r` reverse, `space` stop, `[`/`]` damping kd, `p`
cycle the position field (zero → track → sweep), `q` quit. Safety:
`--tau-max` (hard ceiling 6 Nm), `--ramp` slew, `--vel-max` overspeed
kill, `--turns-max`, `--duration`, drive-error / heartbeat / echo-timeout
watchdogs, `break_timeout` dead-man; every exit path zeroes the command
and returns the axis to IDLE.

`--mode torque` and `--mode velocity` run the same telemetry and safety
around `Set_Input_Torque` / `Set_Input_Vel` so the three candidates can be
compared on the same wheel — note neither of those echoes anything, which
is the whole reason MIT is worth the position-range awkwardness.
