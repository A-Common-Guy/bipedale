# sandro_hardware

ROS 2 Jazzy `ros2_control` plugins:

- `sandro_hardware/GIM6010System`: six GIM6010 drives on raw SocketCAN.
- `sandro_hardware/BNO085Sensor`: BNO085 orientation, gyro and acceleration over
  SHTP on SPI (Raspberry Pi) or UART.

The CAN plugin requires exactly six joints. Each joint requires `node_id`, `sign`
(`-1` or `1`), `ratio`, `offset_rev`, and the positive parameters
`mit_max_position`, `mit_max_velocity`, `mit_max_kp`, `mit_max_kd`,
`mit_max_torque`. It exports position/velocity/effort states, an effort command,
and optional `feedback_age`, `online`, `iq`, `temperature`, `error` and
`dropped_frames` states. Declare each wanted interface in the `<ros2_control>`
description. The receiver thread stages every frame; `read()` copies the staged
values into the exported states once per cycle, so a joint's position, velocity
and `feedback_age` always belong to the same sample.

The drives run in MIT mode: every cycle each receives one frame with
`position`, `velocity`, `kp`, `kd` and `torque`, and computes
`kp (q_ref - q) + kd (v_ref - v) + torque` in its own loop, synchronous with
the frame and free of bus latency. The `effort` command interface is the
torque field and is mandatory. `position`, `velocity`, `kp` and `kd` are
optional command interfaces (declare them per joint); while a controller
leaves them unset (NaN) or at zero the drive is a pure torque source, as
before. When a stiffness is commanded the plugin pulls the position reference
toward the measured position so that the torque the drive will compute from
the latest state stays within `hard_effort_limit_nm` and the drive's MIT torque
limit (the bound SandroControl applied); gains are clipped to the drive's
`mit_max_kp` / `mit_max_kd`. A controller that claims `kp`/`kd` must zero them
on deactivation, which the whole-body controller does.

The drive-side PD acts in the drive's own MIT position frame. At activation,
after closed-loop entry and while the gains are still zero, the plugin
compares each stiffenable joint's MIT reply position with the encoder
broadcast converted to the wire frame, logs the bias per drive, adds a small
bias to every position reference, and refuses to arm beyond
`mit_frame_bias_tolerance_rad` (default 0.2): a wrong drive zero would
otherwise pull the joint against the torque limit while the controller sees a
small error. A saturated reply (multi-turn wheel) is skipped.

`state_source` is `encoder` (default: position and velocity from the 0x09
broadcast) or `mit`: velocity, and position, from the reply to every MIT
command. The reply position is converted to the encoder's logical frame (bias
and `offset_rev` removed, taken on the MIT span nearest the broadcast), and is
used only for a joint whose bias was measured at this activation (with `mit`
every joint is checked, not only those with `kp`) and inside 90 % of the MIT
range; otherwise the broadcast keeps the position. While replies arrive the
broadcast does not overwrite them; before the first reply (inactive) and
`stale_state_s` after the last, the broadcast is the state again.
`sandro.launch.py` passes `encoder_period_ms` 5 with `encoder` and 20 with
`mit` unless set (`JOINT_TIMING`), and in MuJoCo the matching joint timing.

Faults are reported in one throttled line per second, `DRIVE FAULT: <joint>
(node N): no encoder feedback | no heartbeat | axis error 0xCODE ...`, with an
`all drives healthy again` line on recovery; the IMU plugin does the same with
`IMU FAULT`. A shutdown from the active state (Ctrl-C on the controller
manager) runs the deactivation sequence (zero torque, idle, deadman off); if
the process is killed before that, the drives' communication deadman stops
them within `deadman_ms`.

Lifecycle and bus traffic:

- configure: opens the socket, starts the receiver, sends one idle request
  (`Set_Axis_State` 1) per drive. Nothing is energised.
- inactive: `read()` reports joint states, `online` and `feedback_age` from
  whatever the drives broadcast; `write()` sends nothing. A fault is not an
  error while inactive.
- activate: clears operator-induced errors, configures broadcasts, reads the
  MIT limits, enters closed loop with zero-gain zero-torque frames, measures
  the MIT frame bias, arms the deadman last. Every command interface is reset
  first (effort 0, the others NaN), so stale gains from before an error cannot
  reach the drives. A verified SDO write waits for the drive's read-back
  instead of a fixed 20 ms delay.
- deactivate and error: zero gains and torque, idle, and the deadman off as
  an unverified write, all as non-blocking frames on the calling thread (the
  error path runs in the control loop). Nothing waits for a reply; a drive
  that misses a frame keeps its deadman armed and idles by itself within
  `deadman_ms`, latching `can_watchdog`, which arming clears. The error path
  ends unconfigured with the socket still open; the next configure (arming
  from unconfigured) reopens it, as cleanup and shutdown close it.

The control loop never blocks on the bus: MIT frames go out with
`MSG_DONTWAIT`. A frame refused because the send buffer or interface queue is
full (`EAGAIN`, `ENOBUFS`) is dropped and counted in the joint's
`dropped_frames`; `write()` fails only after more than 5 consecutive cycles
with a drop, or at once on any other send error. A non-finite effort command
withholds that joint's frame and fails the cycle, but the other joints still
get theirs. The socket's send buffer is limited (`can_send_buffer_bytes`,
default 16384, which the kernel doubles to about 25 to 40 frames, i.e. a few
cycles) so torque frames cannot queue behind a stalled bus and go out stale;
the effective size is logged at configure, and 0 keeps the kernel default.
Configuration writes block for buffer space for at most 200 ms.

System parameters are `can_interface`, `encoder_period_ms`, `iq_period_ms`,
`heartbeat_period_ms`, `deadman_ms`, `stale_state_s`, `heartbeat_stale_s`,
`activation_timeout_s`, `hard_effort_limit_nm`, optional `state_source`,
`mit_frame_bias_tolerance_rad` and `can_send_buffer_bytes`,
and the numeric firmware endpoint IDs `encoder_rate_endpoint`,
`iq_rate_endpoint`, `heartbeat_rate_endpoint`, `deadman_endpoint`. Endpoint IDs
must come from the exact drive firmware's `endpoints_v3.json`; they are
deliberately not guessed. The original wiring is nodes 1/2/3 for right
inner/outer/wheel and 5/4/6 for left inner/outer/wheel, with signs
`+/-/+`, `+/-/-` respectively.

The BNO plugin requires one sensor and the parameters `transport` (`spi` or
`uart`, default `uart`), `read_timeout_ms`, `report_interval_us` and
`online_timeout_s`, plus optional `activation_timeout_s` (default 1.0), how
long activation waits for the first of every report. SPI adds `spi_device`,
`spi_speed_hz`, `gpio_chip`, `int_gpio` and `reset_gpio`; UART adds `port`,
`baudrate`, `tx_byte_gap_us` and `max_backlog_bytes`. Its ten standard state
interfaces are `orientation.{x,y,z,w}`, `angular_velocity.{x,y,z}`, and
`linear_acceleration.{x,y,z}`. Optional diagnostics are `age` and `online`:
`age` is that of the oldest of the three reports, and `online` requires it
below `online_timeout_s` (otherwise the orientation reads NaN).
Quaternion ordering is ROS `x,y,z,w`.

Over SPI the plugin follows `deploy/pi/tools/bno085_spi_probe.py`: SPI mode 3, the
chip's WAKE pin tied high, so it only accepts a write while it asserts INT.
Activation pulses RST, reads one boot packet, then enables the game rotation
vector, calibrated gyro and accelerometer while INT is still asserted;
deactivation holds the chip in reset. A receiver thread reads every packet
INT announces (header, then the whole packet), decodes it outside the lock
shared with `read()` and only copies the values under it. On an SPI/GPIO
error, or when one of the three reports has been missing for
max(0.5 s, 3 x `online_timeout_s`) (a chip that reset itself comes back with
every report disabled), it pulses RST and re-enables the reports, logging
each attempt, with a pause from 0.1 s doubling to 2 s between failed ones.
The UART path has no such recovery. The SPI path has not yet run
on the robot; check `imu_update_hz` in `compare_runs.py` on the first
recording. Over the FTDI adapter the samples reached the controller in bursts
about 33 ms apart (30 Hz, all four 2026-09-20 recordings) despite the 5 ms
report interval.

All limits are explicit parameters and are validated. Commands are clipped only
at the configured MIT torque limit. Activation first verifies fresh feedback,
configures broadcasts and MIT mode, reconciles at measured position with zero
gains, enters closed loop, then arms the CAN deadman last. Deactivation sends
zero torque, idles every drive, and disables the deadman (see Lifecycle above).
