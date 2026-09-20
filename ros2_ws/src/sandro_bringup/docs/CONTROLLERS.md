# Extending controllers

Controllers are ordinary `ros2_control` plugins. There is no private controller
protocol and no required Sandro base class.

1. Derive from `controller_interface::ControllerInterface` and export with
   pluginlib.
2. Claim the six effort commands and only the standard state interfaces needed.
   Each joint also offers `position`, `velocity`, `kp` and `kd` command
   interfaces, the other fields of the drives' MIT frame: a controller that
   claims them makes the drive compute `kp (q_ref - q) + kd (v_ref - v)` on
   top of the effort in its own loop (the whole-body controller does this for
   the legs). Unset or zero, the drive is a torque source. A controller that
   claims `kp`/`kd` must zero them on deactivation.
3. Receive `geometry_msgs/TwistStamped` through a realtime buffer when needed.
4. Declare gains, limits, timeouts, and units as ROS parameters. Validate a
   complete update before applying it; never silently replace a requested value.
5. Add the type and parameters to `config/controllers.yaml`, then launch with
   `controller:=<configured_name>`.

`DrivingBalancerController` is the reference for live parameter updates,
requested/effective telemetry, saturation reasons, stale-command handling, and
zero-on-deactivation. `TimedEffortController` is only for lifted commissioning:
it clamps at `effort_limit_nm`, requires continuous messages, and zeros after
`command_timeout_s`.

Switch controllers with the standard manager:

```bash
ros2 control switch_controllers \
  --deactivate driving_balancer --activate my_controller
```

RL and whole-body controllers can use the same interfaces directly; see
`WHOLE_BODY_CONTROLLER.md` for the leg-exploiting balancer that ships here. Keep
final policy limits in each controller; the separate 10 Nm hardware boundary is
fault containment, not a tuning value.

Both backends present one joint convention (zero-down five-bar angles,
forward-positive wheels; `sandro_description` README). A controller must not
carry per-backend sign or zero tables.

`tools/drive_test.py` publishes a scripted `/cmd_vel` sequence and prints any
balancer's `~/diagnostics` as a table, so a change can be judged from numbers
in a headless simulation.
