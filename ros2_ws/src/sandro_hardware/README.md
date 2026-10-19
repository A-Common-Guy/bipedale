# sandro_hardware

ROS 2 Jazzy `ros2_control` plugins:

- `sandro_hardware/GIM6010System`: six GIM6010 drives on raw SocketCAN.
- `sandro_hardware/BNO085Sensor`: BNO085 orientation, gyro and acceleration over
  SHTP on SPI (Raspberry Pi) or UART.

The CAN plugin requires exactly six joints. Each joint requires `node_id`, `sign`
(`-1` or `1`), `ratio`, `offset_rev`, and the positive parameters
`mit_max_position`, `mit_max_velocity`, `mit_max_kp`, `mit_max_kd`,
`mit_max_torque`. It exports position/velocity/effort states, an effort command,
and optional `feedback_age`, `online`, `iq`, `temperature`, and `error` states.
Declare each wanted interface in the `<ros2_control>` description.

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

Faults are reported in one throttled line per second, `DRIVE FAULT: <joint>
(node N): no encoder feedback | no heartbeat | axis error 0xCODE ...`, with an
`all drives healthy again` line on recovery; the IMU plugin does the same with
`IMU FAULT`. A shutdown from the active state (Ctrl-C on the controller
manager) runs the deactivation sequence (zero torque, idle, deadman off); if
the process is killed before that, the drives' communication deadman stops
them within `deadman_ms`.

System parameters are `can_interface`, `encoder_period_ms`, `iq_period_ms`,
`heartbeat_period_ms`, `deadman_ms`, `stale_state_s`, `activation_timeout_s`,
and the numeric firmware endpoint IDs `encoder_rate_endpoint`,
`iq_rate_endpoint`, `heartbeat_rate_endpoint`, `deadman_endpoint`. Endpoint IDs
must come from the exact drive firmware's `endpoints_v3.json`; they are
deliberately not guessed. The original wiring is nodes 1/2/3 for right
inner/outer/wheel and 5/4/6 for left inner/outer/wheel, with signs
`+/-/+`, `+/-/-` respectively.

The BNO plugin requires one sensor and the parameters `transport` (`spi` or
`uart`, default `uart`), `read_timeout_ms`, `report_interval_us` and
`online_timeout_s`. SPI adds `spi_device`, `spi_speed_hz`, `gpio_chip`,
`int_gpio` and `reset_gpio`; UART adds `port`, `baudrate`, `tx_byte_gap_us`
and `max_backlog_bytes`. Its ten standard state interfaces are
`orientation.{x,y,z,w}`, `angular_velocity.{x,y,z}`, and
`linear_acceleration.{x,y,z}`. Optional diagnostics are `age` and `online`.
Quaternion ordering is ROS `x,y,z,w`.

Over SPI the plugin follows `raspi_tests/bno085_spi.py`: SPI mode 3, the
chip's WAKE pin tied high, so it only accepts a write while it asserts INT.
Activation pulses RST, reads one boot packet, then enables the game rotation
vector, calibrated gyro and accelerometer while INT is still asserted;
deactivation holds the chip in reset. A receiver thread reads every packet
INT announces (header, then the whole packet). The SPI path has not yet run
on the robot; check `imu_update_hz` in `compare_runs.py` on the first
recording. Over the FTDI adapter the samples reached the controller in bursts
about 33 ms apart (30 Hz, all four 2026-09-20 recordings) despite the 5 ms
report interval.

All limits are explicit parameters and are validated. Commands are clipped only
at the configured MIT torque limit. Activation first verifies fresh feedback,
configures broadcasts and MIT mode, reconciles at measured position with zero
gains, enters closed loop, then arms the CAN deadman last. Deactivation sends
zero torque, idles every drive, and disables the deadman.
