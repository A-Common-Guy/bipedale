# Hardware commissioning

Physical bringup is intentionally non-arming. Python/ROS safety checks do not
replace firmware limits, a physical E-stop, supports, or an independent power
cut-off.

The convenience CAN E-stop is:

```bash
ros2 service call /gim6010_estop/trigger std_srvs/srv/Trigger
```

It is software over the same CAN bus and is not a safety-rated substitute for
the physical power cut-off. With teleop running, pressing L2 and R2 together on
the gamepad (`joy_estop`, configured in `config/teleop.yaml`) deactivates every
command-owning controller and calls this same service; it is equally software
only, and nothing on the gamepad re-arms the drives.

After either E-stop the drives latch `ESTOP_REQUESTED` (axis error `0x4000`,
reported together with the CAN watchdog code as `0x104000` = 1064960).
Arming the hardware component again is the deliberate recovery: activation
disables the watchdog, clears exactly these two operator-induced bits, and
logs what it cleared; any other fault (encoder, over-temperature, motor)
still refuses activation and must be inspected. So after an E-stop simply
relaunch with `arm_hardware:=true`, or if the launch is still running:

```bash
ros2 control set_hardware_component_state SandroSystem active
ros2 control switch_controllers --activate whole_body_controller
```

For inspection without re-arming, `ros2 service call /gim6010_estop/clear_errors
std_srvs/srv/Trigger` sends `Clear_Errors` to every drive, as does
`cansend can0 <(node<<5)|0x18>#0000000000000000` per node.

Normal disarm uses the managed hardware lifecycle:

```bash
ros2 control set_hardware_component_state SandroSystem inactive
```

1. Mechanically lift and secure the robot; remove wheel/leg load.
2. Configure `can0` on the host and confirm nodes 1–6 using passive CAN
   inspection. Do not expose all `/dev` devices to the container.
3. With drive power disabled, verify the configured wiring:
   right inner/outer/wheel = 1/2/3; left inner/outer/wheel = 5/4/6.
4. Move each axis by hand. Verify logical signs, output-shaft radians, velocity,
   and the ±2.0944 rad leg range.
5. Establish the joint zero. The convention is: each upper link (the 80 mm
   crank on the motor shaft) pointing straight down toward the wheel with the
   body level, for all four leg motors. Two equivalent ways:
   - **Drive-level user zero (preferred on the GIM6010-8).** Its dual
     encoders keep `encoder.config.index_offset` across power cycles
     (`gim6010/zero_load_encoder.py`, manual 3.4.3). Set it in the
     upper-link-down pose per motor and leave `offset_rev: 0.0`.
   - **Host offset.** Leave the drive zero alone and put the rotor reading of
     the upper-link-down pose (`raw_position_rev` on `/dynamic_joint_states`)
     into that joint's `offset_rev` in `sandro.ros2_control.xacro`.

   Verify either way: in the standing pose `/joint_states` must read about
   0.41 / 0.22 rad (right inner / outer) and 0.22 / 0.41 rad (left inner /
   outer), the same values the simulation shows. Then power-cycle in that pose
   and read again. The drive's power-loss memory covers ±4 rotor turns
   (±180° of the output) around the *encoder's* absolute zero, and the user
   zero only shifts the numbers inside that window; if a motor is powered on
   beyond the window edge it reports 8 turns (2π) off. So also swing each
   upper link slowly from straight down to fully open (about 60°) and back
   while echoing `/joint_states`: the angle must be continuous with no 2π
   jump, and a wheel turned several times by hand must count past 2π without
   jumping. The whole-body controller refuses to engage while a leg reading is
   outside the five-bar workspace, but the wheel-only balancers do not check.
6. Read and record the drive's five MIT encoding capabilities. They must match
   the values used to encode frames; do not reduce them as a tuning mechanism.
   The independent host boundary is 10 N·m, while the initial controller policy
   boundaries are 3 N·m per leg joint and 6 N·m per wheel. Verify the firmware
   300 ms communication deadman. The leg PD runs inside the drives (MIT kp/kd),
   so `mit_max_kp` must be at least the controller's `tasks.legs.drive_kp_nm_rad`
   (12) and `mit_max_kd` at least `drive_kd_nm_s_rad` (0.5); the plugin clips
   to the drive's values and logs them at activation.
   At every activation the plugin also prints, per leg drive, `MIT position
   ... encoder frame ... bias ... rad`: the drive's own MIT position against
   the encoder broadcast, measured with zero gains. A few hundredths of a
   radian is normal and is corrected; beyond `mit_frame_bias_tolerance_rad`
   (0.2) activation fails, because a stiffness acting in a frame with a
   different zero would hold that joint at the wrong angle (the fifth
   hardware run showed exactly this on one leg: a stiff, asymmetric stance
   that did not follow height commands). Re-zero the drive and repeat.
   Stop the launch with a single Ctrl-C and wait: the controller manager
   deactivates the drives (zero torque, idle, deadman off) before exiting,
   and a second Ctrl-C kills it mid-way and leaves the drives to the deadman.
   rviz2 is given 2 s before SIGTERM so the whole launch is down in a few
   seconds.
7. Test one motor, then one paired leg, with the physical cut-off in reach.
   Preview first, then explicitly activate:

   ```bash
   ros2 launch sandro_bringup commissioning.launch.py \
     target_type:=motor target:=right_inner activate:=false
   ros2 launch sandro_bringup commissioning.launch.py \
     target_type:=motor target:=right_inner activate:=true
   ```

   The selected controller accepts Nm values on
   `/commission_right_motor_inner/commands` as
   `std_msgs/msg/Float64MultiArray`, clamps at the documented 0.5 N·m
   commissioning boundary, and zeros after 0.1 s without a message. A one-axis
   command is:

   ```bash
   ros2 topic pub -r 20 /commission_right_motor_inner/commands \
     std_msgs/msg/Float64MultiArray "{data: [0.1]}"
   ```

8. The plugin reads joint position and velocity from the drives' 0x09
   broadcast (`encoder_period_ms`, 5 ms). `state_source:=mit` takes velocity,
   and leg position, from the reply to every MIT command instead, synchronous
   with the 300 Hz loop; the broadcast then only keeps the wheels' multi-turn
   position and `encoder_period_ms:=20` frees about 15 % of the bus.
9. Inspect `/joint_states` and `/dynamic_joint_states`; the latter includes raw
   requested/applied effort and saturation count. Verify disable, E-stop, stale
   feedback, CAN removal, IMU removal, process
   termination, overspeed, and command timeout before ground contact.
10. Commission the IMU frame (x forward, y left, z up), then balance signs
    while tethered. Tune station keeping before velocity or yaw. A roll
    mounting bias shows up as unequal leg lengths with a level IMU reading;
    put the IMU's roll at a truly level body into `tasks.roll.imu_offset_rad`.

The commissioning controller claims only the selected motor or leg interfaces,
so ros2_control prevents simultaneous activation of balance/RL/WBC command
owners.

## Drive velocity estimate check

The leg damping and the wheel velocity feedback use each drive's reported
velocity. Compare it with the derivative of the reported position in a
`drive_test.py --record` file: the slope of velocity against position
derivative should be about 0.85 for every drive (the 5 ms broadcast and the
15 Hz filtering account for the rest) with a one-sample lag. In the third
hardware run the left inner drive reported 0.46 while the other five reported
0.84 to 0.87; check that drive's velocity estimator and encoder configuration
when one drive stands out like this.
