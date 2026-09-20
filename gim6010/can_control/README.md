# GIM6010 CAN control (CANSimple)

Standalone Python library + CLI for controlling SteadyWin/CyberBeast GIM6010
drives over CAN, implementing the CANSimple protocol from
`../manual/CyberBeast_Driver_Protocol_Manual_EN.md` §1. Torque/velocity/
position control, MIT mode, synchronous feedback, decoded errors, e-stop, and
arbitrary parameter read/write (RxSdo/TxSdo). No odrive USB package needed —
only `python-can` (installed in `../control/venv`).

```
canup.sh        bring up can0 (1 Mbps — must match can.config.baud_rate;
                factory default is 500 kbps!)
gim6010_can.py  library: GIM6010Bus / Motor
gim_cli.py      command-line tool
read_pos.py     read-only raw position / turn monitor for N nodes
sim_motor.py    software motor emulator (python-can 'virtual' bus)
selftest.py     end-to-end test of the library against the emulator
```

## Quick start

```bash
./canup.sh                     # bring up SocketCAN
V=../control/venv/bin/python
$V selftest.py                 # no hardware needed — verifies the stack

$V gim_cli.py scan                         # who's on the bus (heartbeats)
$V read_pos.py 1 2 6                       # raw pos + turns, motor NOT enabled
$V gim_cli.py status 1
$V gim_cli.py errors 1                     # decoded per-subsystem errors
$V gim_cli.py deadman 1 2000               # STRONGLY recommended before torque
$V gim_cli.py torque 1 0.05 --enable --duration 3   # 0.05 Nm rotor, 3 s
$V gim_cli.py estop all                    # e-stop every node seen
$V gim_cli.py errors 1 --clear             # recover from estop
$V gim_cli.py rates 1                      # show periodic broadcast periods
$V gim_cli.py rates 1 --encoder 2 --iq 10  # 500 Hz pos/vel + 100 Hz current
$V gim_cli.py sdo read 1 vbus_voltage
$V gim_cli.py sdo write 1 vel_limit 20     # RAM only
$V gim_cli.py save 1                       # persist to flash + reboot
```

## Reading positions without touching the motors

`read_pos.py` monitors the raw rotor position and whole-turn count of any set
of nodes, selected by CAN node id, and never leaves the drives idle: it only
sends RTR read requests (0x09 pos/vel, 0x0A raw encoder counts). Safe to run
while hand-turning an actuator to check direction, gear ratio or zeroing.

```bash
$V read_pos.py 1 2 6                  # one line per motor, 5 Hz
$V read_pos.py --scan --rate 20       # every node that heartbeats
$V read_pos.py 1 --once               # single snapshot
$V read_pos.py 1 --zero               # count turns from the current position
$V read_pos.py 1 --passive            # transmit nothing at all (broadcasts only)
$V read_pos.py 1 2 --csv turns.csv --duration 60
```

`pos`/`turns` are rotor-side revolutions exactly as received on the wire; the
`out` column divides by `--ratio` (default 8) for the output shaft. `--passive`
needs the pos/vel broadcast enabled on the drive (`rates <node>`, default
10 ms). Positions read as `-` until the drive's encoder is ready.

Multi-motor: every command takes the node id; `scan` lists what's alive,
`estop all` broadcasts to all discovered nodes. Give each drive a unique
`node_id` (0–63) before wiring them together (`sdo write <n> node_id <m>` +
`save`, or via the web UI).

## Commissioning a chain (baud / termination / ids)

All GIM6010 drives support 1 Mbps (manual §5.1.4; hw 3.11 / fw 0.6.5
bench-verified). **The baud register applies IMMEDIATELY on write** (before
any save) — the drive drops off the old-baud bus the moment it's written, so
`commission` applies it last and guides the recovery. Recipe for a 3-drive
chain (factory 500 kbps → 1 Mbps), one drive at a time on the bench:

```bash
# per drive, at 500 kbps: unique id, termination only on the LAST drive
gim_cli.py commission 0 --node-id 4 --r120 off        # saves, stays at 500k
gim_cli.py commission 0 --node-id 5 --r120 off
gim_cli.py commission 0 --node-id 6 --r120 on         # chain end

# then flip each to 1M (drive jumps immediately and stops answering at 500k)
gim_cli.py commission 4 --baud 1000000                # exits with re-link steps
...
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000        # or just ./canup.sh
sudo ip link set can0 up
gim_cli.py scan                                       # all nodes back?
gim_cli.py save 4 && gim_cli.py save 5 && gim_cli.py save 6   # persist 1M
```

A power cycle before the final `save` reverts an unsaved baud change. The
host adapter end of the bus needs its own 120 Ω (many USB-CAN adapters have
one switchable onboard) — target ~60 Ω total across CANH/CANL.

### Troubleshooting: scan works but SDO/save time out

That pattern means drive→host frames arrive but host→drive frames are lost
(bench-observed at 1 Mbps). Usual cause: **CAN sample point**. gs_usb
adapters default to 74.7 % at 1 Mbps while the drive samples at ~87.5 % —
marginal enough that requests and the (fire-and-forget) save frame get
dropped. `canup.sh` pins `sample-point 0.875` at every bitrate; always bring
the link up with it. `save`/`commission` verify the reboot via the heartbeat
gap and read config back, so a swallowed save is reported instead of looking
like success. Other suspects: missing 120 Ω at one end, no common ground.

## Library

```python
from gim6010_can import GIM6010Bus

with GIM6010Bus("can0", endpoints="…/endpoints_v3.json") as bus:
    m = bus.motor(1)
    m.set_deadman(2000)              # firmware faults if commands stop 2 s
    m.enable_torque_mode()           # torque/passthrough + closed loop,
                                     # with setpoint reconciliation
    for _ in range(1000):            # 100 Hz control loop
        m.set_input_torque(0.05)                  # Nm, rotor side
        pos, vel = m.get_encoder_estimates()      # synchronous RTR poll
        # ... or zero-latency reads of the periodic broadcasts:
        s = m.state                  # .pos .vel .torque .axis_error .temperature ...
        if s.any_error:
            break
        time.sleep(0.01)
    m.set_input_torque(0.0)
    m.idle()
```

- **Feedback, two ways.** A background RX thread decodes every frame into
  `motor.state` (timestamped) — the drive broadcasts heartbeat @100 ms and
  pos/vel @10 ms by default, and `m.set_feedback_rates(encoder=2, iq=10)`
  retunes any broadcast period over CAN (CLI: `rates`). For strict
  request/response semantics every `get_*()` sends an RTR frame and waits for
  the reply — fine for status/tools, not for a control loop. MIT mode
  (`m.mit_control(...)`) echoes pos/vel/torque in the same transaction.

### Non-blocking real-time loop (e.g. 500 Hz)

Never poll in the hot path — set the pos/vel broadcast to your cycle time and
read `motor.state`, which the RX thread updates with zero blocking. The drive
itself becomes the clock source: `encoder_event` fires on every pos/vel frame,
so waiting on it paces the loop at exactly the broadcast rate and each cycle
runs right after fresh feedback arrives.

```python
m.set_feedback_rates(encoder=2)     # 0x09 every 2 ms = 500 Hz
m.set_deadman(20)                   # fault if commands stop for 20 ms
m.enable_torque_mode()

while running:
    m.encoder_event.wait(0.004)     # released by the 2 ms broadcast
    m.encoder_event.clear()
    s = m.state                     # plain reads, never blocks
    if s.any_error or s.encoder_age() > 0.01:
        m.set_input_torque(0.0); m.idle(); break
    m.set_input_torque(controller(s.pos, s.vel))
```

Torque feedback without polling: enable the Iq broadcast
(`set_feedback_rates(iq=10)`) and use `state.iq_measured * torque_constant`
(≈0.07 Nm/A rotor-side on the GIM6010-8 — read `torque_constant` via SDO).

Bus load at 1 Mbps: a broadcast/command frame is ~130 bits, so pos/vel @500 Hz
+ torque commands @500 Hz ≈ 13 % load per motor — 2 motors ≈ 26 %, 4 ≈ 55 %
(plus heartbeats). Keep the 1 Mbps baud rate; at the factory 500 kbps two
motors at 500 Hz would already saturate the bus.
- **Frames/units.** Classic commands are rotor-side (rev, rev/s, motor Nm) —
  manual §1.3. `GIM6010Bus(..., frame="output", ratio=8)` (CLI:
  `--frame output`) converts both directions for the 8:1 GIM6010-8. MIT mode
  is always output-shaft radians per the protocol and is never converted.
- **Errors.** Heartbeat carries `axis_error` + per-subsystem flag bits;
  `m.get_all_errors()` fetches and decodes motor/encoder/controller/system
  codes (tables from manual §4.1 incl. the SteadyWin `SEC_ENC_COM_FAIL` bit).
- **E-stop.** `m.estop()` / `bus.estop_all()` latch `ESTOP_REQUESTED`;
  recover with `clear_errors()` + re-enable.
- **Parameters.** `m.sdo_read/sdo_write(path)` accesses any endpoint in the
  firmware tree (suffix match allowed, e.g. `"gear_ratio"`); writes verify by
  read-back. Types >32 bit (`uint64` serial number etc.) can't fit the
  4-byte SDO value field. `save_configuration()` persists (drive reboots).
  Endpoint IDs come from `../manual/assets_protocol_manual/endpoints_v3.json`
  (matches fw 0.6.5 / V3 hardware — verified against the manual's examples:
  `vbus_voltage`=2, `gear_ratio`=0xF2). Other hw/fw: pass the matching JSON.

## Safety notes

- **Enable the dead-man first** (`deadman <n> 2000`): torque mode keeps
  pushing if your process dies; with `break_timeout` the drive faults when
  the command stream stops (user manual §5.1.9).
- `enable`/`enable_torque_mode` reconcile setpoints before closed loop
  (zero torque/vel, pin `input_pos` to the current position) — same policy
  as the USB web UI.
- The CLI's `torque` command zeroes torque and idles the axis on Ctrl-C,
  end of `--duration`, or any axis error.
- `estop all` only reaches nodes whose heartbeats have been seen — it prints
  the list; verify it matches your motor count.
- Keep `current_lim` within supply/actuator limits (≤ 50 A q-axis per manual).

## Protocol cheat sheet (CANSimple)

11-bit ID = `(node_id << 5) | cmd`. Little-endian, IEEE-754 float32.

| cmd | name | payload |
|---|---|---|
| 0x01 | Heartbeat (drive→host, 100 ms) | axis_error u32, state u8, flags u8, temp i8, life u8 |
| 0x02 | Estop | 8×00 |
| 0x03 | Get_Error | req: type u8 (0 motor / 1 enc / 3 ctrl / 4 sys) → code u32/u64 |
| 0x04/0x05 | RxSdo/TxSdo | opcode u8 (0 r / 1 w), endpoint u16, rsvd, value 4 B |
| 0x07 | Set_Axis_State | u32: 1 idle, 3 full-cal, 4 motor-cal, 7 enc-cal, 8 closed loop |
| 0x08 | Mit_Control | packed pos16/vel12/kp12/kd12/torque12 → echo pos/vel/torque |
| 0x09 | Get_Encoder_Estimates (10 ms) | pos f32 rev, vel f32 rev/s |
| 0x0B | Set_Controller_Mode | control u32 (1 T / 2 V / 3 P), input u32 (1 pass / 2 vramp / 3 filt / 5 traj / 6 tramp / 9 MIT) |
| 0x0C | Set_Input_Pos | pos f32 rev, vel_ff i16 ‰, torque_ff i16 ‰ |
| 0x0D | Set_Input_Vel | vel f32, torque_ff f32 |
| 0x0E | Set_Input_Torque | torque f32 Nm (rotor) |
| 0x0F | Set_Limits | vel f32 rev/s, current f32 A |
| 0x14/0x15/0x17/0x1C/0x1D | Get Iq / temps / bus V-I / torques / powers | 2×f32 (RTR) |
| 0x16 / 0x18 / 0x1F | Reboot / Clear_Errors / Save_Configuration | 8×00 |
