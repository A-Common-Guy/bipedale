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
