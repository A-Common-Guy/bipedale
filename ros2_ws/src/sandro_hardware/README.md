# sandro_hardware

ROS 2 Jazzy `ros2_control` plugins:

- `sandro_hardware/GIM6010System`: six GIM6010 drives on raw SocketCAN.
- `sandro_hardware/BNO085Sensor`: BNO085 UART-SHTP orientation, gyro and acceleration.

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

System parameters are `can_interface`, `encoder_period_ms`, `iq_period_ms`,
`heartbeat_period_ms`, `deadman_ms`, `stale_state_s`, `activation_timeout_s`,
and the numeric firmware endpoint IDs `encoder_rate_endpoint`,
`iq_rate_endpoint`, `heartbeat_rate_endpoint`, `deadman_endpoint`. Endpoint IDs
must come from the exact drive firmware's `endpoints_v3.json`; they are
deliberately not guessed. The original wiring is nodes 1/2/3 for right
inner/outer/wheel and 5/4/6 for left inner/outer/wheel, with signs
`+/-/+`, `+/-/-` respectively.

The BNO plugin requires one sensor and parameters `port`, `baudrate`,
`read_timeout_ms`, `report_interval_us`, `online_timeout_s`, `tx_byte_gap_us`,
and `max_backlog_bytes`. Its ten standard state interfaces are
`orientation.{x,y,z,w}`, `angular_velocity.{x,y,z}`, and
`linear_acceleration.{x,y,z}`. Optional diagnostics are `age` and `online`.
Quaternion ordering is ROS `x,y,z,w`.

All limits are explicit parameters and are validated. Commands are clipped only
at the configured MIT torque limit. Activation first verifies fresh feedback,
configures broadcasts and MIT mode, reconciles at measured position with zero
gains, enters closed loop, then arms the CAN deadman last. Deactivation sends
zero torque, idles every drive, and disables the deadman.
