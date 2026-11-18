# Policy trials on the robot

Recordings of the panel's Sysid page, "Record a policy trial"
(`ros2_ws/src/sandro_bringup/tools/policy_record.py`): the operator tries the
deployed policies one after the other, the recording follows and logs which
controller owned the drives at every moment. They are the robot's answer to a
set of policies, kept here for whoever trains the next ones.

To add one: download **Recording** from the Sysid page's Results, untar it
here, and write what was seen and heard into `notes.md` inside the run (the
recording cannot hear the buzz or see a wobble the gyro misses).

    tar -xf ~/Downloads/policies_20261119-101500.tar -C recordings/policy_trials/

## What a run holds

| File | Content |
|---|---|
| `plan.json` | `events`: `{"kind": "controller", "name", "t"}` at every change of the controller that owns the drives (`name` null: none, as after an E-stop), between a `start` and an `end`; `t` is the wall clock, the clock of the bag's header stamps. `controllers`: every parameter of each controller that ran, read from its node (`policy_file`, `policy_fingerprint`, the wheel filters, gains). `version`: the commit the robot ran. |
| `summary.json` | `stretches`: one row per stay of a controller, in order. `from_s`, `seconds`, the policy file and fingerprint, then the stay cut into 2 s windows: `standing` and `driving` with their seconds and, per band (5-15 Hz, 15-45 Hz) and gyro axis, the `typical` (median window) and `worst` `[RMS rad/s, strongest Hz]`; `driving.top_speed_m_s`; `down_s`, the time tilted past 0.5 rad (fallen or resting on the body, as a stand-up policy before it got up; the robot rests at about 0.68 rad). `verdict`: the same in one line per stretch. |
| `bag/` | ros2 bag, mcap, zstd: `/joint_states` and `/imu_broadcaster/imu` (300 Hz), `/health_broadcaster/dynamic_joint_states`, `/cmd_vel` (the command, `linear.y` the hop request), `/joy`, `/terrain_profile` when the camera runs, every `~/diagnostics` (the policy controllers', the WBC's, the height profile's), `/controller_manager/activity`. |
| `notes.md` | The operator's observations, written by hand. |

A window is `driving` above 0.05 m/s or 0.2 rad/s of turn, `down` above
0.5 rad of tilt, `standing` otherwise. Standing is quiet below 0.06 rad/s in
every band and axis (the WBC stays at 0.03-0.04).

## Reading one

    python3 ros2_ws/src/sandro_bringup/tools/policy_record.py --analyse recordings/policy_trials/<run>

rewrites `summary.json` from the bag (needs numpy and either ROS 2 or
`pip install mcap-ros2-support`). The yaw and roll loops of a policy that
buzzes here are checked against the measured plant with
`recordings/yaw_loop_margin.py`.
