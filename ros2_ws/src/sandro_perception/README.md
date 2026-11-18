# sandro_perception

Sandro's eyes. A RealSense D435 sits on the front bracket (81 mm ahead of the
IMU, level, looking straight ahead) and, for now, on the Raspberry Pi's USB 3
port. This package turns its depth images into the one thing the balance
policies will read: the **terrain profile**, the floor height at eight points
ahead of each wheel.

Nothing here commands the robot and nothing in the control path reads the
profile yet. The control loop keeps its priority: depth runs at 424 x 240 and
15 Hz, an image costs about 8 ms on the Pi 4, and the 300 Hz topics are
sampled at 50 Hz without waking an executor.

## The profile

`/terrain_profile` (`std_msgs/Float32MultiArray`, 25 Hz), 32 values:

| values | meaning |
| --- | --- |
| 0-7 | right track: floor height (m) at 0.10, 0.20, 0.30, 0.45, 0.60, 0.80, 1.05, 1.35 m ahead of the right axle, relative to the floor the right wheel stands on |
| 8-15 | the same for the left track |
| 16-31 | 1 where the value above is known, 0 where it is not (and the value is 0) |

A 3 cm plank 0.45 m ahead of the right wheel reads +0.03 at value 3; once the
wheel is on it, the floor beyond reads -0.03. Heights are clipped to
-0.15..+0.25 m, so a wall ahead reads +0.25. The layout is a contract with
the policies trained on it (`config/perception.yaml`).

`height_profile.py` (numpy only, no ROS) is the definition:

- The camera sees the floor only from about 0.7 m ahead of the wheels (0.34 m
  up, level, 58 degrees of vertical view; about 0.4 m with the bracket tilted
  20 degrees down). What it saw is kept in a 4 cm grid fixed to the floor, a
  2.5 m window that travels with the robot on wheel odometry and the IMU's
  heading, so a plank seen at 0.8 m is still known when the wheel reaches it.
- Floor never seen (at start, after a turn on the spot) is **unknown**, as is
  floor not seen for 8 s. A policy must then behave as a blind one.
- Each image is placed with the body pose at the image's own stamp, 30-60 ms
  back: a body pitching at 1 rad/s would otherwise misplace the floor at
  1.3 m by 5 cm.
- The floor under a wheel is the map's cell there once it is known, so the
  profile stays right while the robot climbs what it has seen.

`test/test_height_profile.py` renders depth from the mount over a floor with
a plank and drives the robot across kinematically: flat reads 0 within 5 mm,
the plank 3 cm within 8 mm through the blind zone, with a rocking body and
60 ms late images.

## Running it

```bash
# beside a running stack (nothing is armed or commanded by this)
ros2 launch sandro_perception camera.launch.py \
    extra_params:=$(ros2 pkg prefix sandro_bringup)/share/sandro_bringup/config/sandro_real_overrides.yaml
# or with the stack
ros2 launch sandro_bringup sandro.launch.py hardware:=real camera:=true ...

ros2 topic echo /height_profile/diagnostics      # depth_hz, process_ms, points_known, floor fit
ros2 topic echo /terrain_profile
```

## Checking the mount

Stand the robot on a flat floor with a clear 2 m ahead. `~/diagnostics`
then gives the plane through the floor it sees 0.6-1.6 m ahead:

- `floor_pitch_deg`: the mount looks this much further down than
  `mount.pitch_rad` says. Add it (in radians) to `mount.pitch_rad`.
- `floor_roll_deg`: likewise `mount.roll_rad`, or the IMU's roll offset.
- `floor_offset_mm`: the floor appears this much too high: the mount's
  height, or the wheel radius.

Half a degree of pitch is 1 cm at 1.3 m, as much as a third of a plank: do
this once after every change to the bracket.

## Not yet

- The policies do not read the profile (sandro_rl: the observation, the
  simulated camera, training; then RlPolicyController).
- The camera is not in the simulation model or the URDF.
- Obstacles are only "high floor": nothing tells a person from a wall.
