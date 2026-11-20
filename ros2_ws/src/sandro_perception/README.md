# sandro_perception

Sandro's eyes. A RealSense D435 sits on the front bracket (its depth origin
114 mm ahead of the IMU, looking 20 degrees down since 2026-11-18: the CAD
camera, `config/perception.yaml`) and, for now, on the Raspberry Pi's USB 3
port. This package turns its depth images into the one thing the balance
policies will read: the **terrain profile**, the floor height at eight points
ahead of each wheel.

Nothing here commands the robot. The camera policies (`rl_agile_step` and
its successors) read the profile; every other controller ignores it. The
control loop keeps its priority: depth runs at 424 x 240 and
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

- The camera sees the floor only from 0.3-0.45 m ahead of the wheels (0.25-0.35 m
  up, 20 degrees down, 58 degrees of vertical view; 0.55-0.75 m with the level
  bracket), and floor behind an obstacle not at all. What it saw is kept in a 4 cm grid fixed to the floor, a
  2.5 m window that travels with the robot on wheel odometry and the IMU's
  heading, so a plank seen at 0.8 m is still known when the wheel reaches it.
- Floor never seen (at start, after a turn on the spot) is **unknown**, as is
  floor not seen for 8 s. A policy must then behave as a blind one.
- Each image is placed with the body pose at the image's own stamp, 30-60 ms
  back: a body pitching at 1 rad/s would otherwise misplace the floor at
  1.3 m by 5 cm.
- The floor under a wheel is the map there once it is known, so the profile
  stays right while the robot climbs what it has seen: the lowest cell within
  5 cm before and behind the contact point. Taken from the one cell under the
  axle, a wheel 5 cm short of a step read as standing on it and the step
  vanished from the profile just before the hop (2026-11-19).
- A cell reads the top of what is in it (its points' mean plus 1.5 standard
  deviations, at most the highest point), not the mean: the face of a 15 cm
  block averaged 7 cm, a 3 cm edge 2.
- A cell becomes known, or changes by more than 3 cm, only when two images in
  a row say so: one bad image no longer leaves an obstacle in the part of the
  map the camera cannot see again. The robot's recording of 2026-11-19 has
  such cells, 16 cm at 0.2-0.3 m ahead of a standing robot for up to 1.5 s.

`test/test_height_profile.py` renders depth from the mount over a floor with
a plank and drives the robot across kinematically: flat reads 0 within 5 mm,
the plank 3 cm within 8 mm through the blind zone, with a rocking body and
60 ms late images. `sandro_rl`'s evaluator runs the same file in the policy's
loop on images rendered in MuJoCo (`--sim depth=1`, `sandro_rl/depth_camera.py`).

## What the camera saw, afterwards

`~/depth_preview` publishes every fifth image at a quarter of the resolution
(106 x 60, about 40 kB/s; `preview.hz`, `preview.stride`), and
`policy_record.py` records it. `recordings/replay_profile.py <run>` runs
`height_profile.py` on it again, beside the profile the robot published:
why a profile read what it read, and what a change to the map would have made
of the same images.

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

- No deployed policy reads the profile. `sandro_rl` trains on a simulated one
  (`configs/agile_perceptive.yaml`, sandro_rl/docs/DESIGN.md 14) and
  `RlPolicyController` takes it for policies exported with it, but the first
  policies gain nothing from the camera.
- The camera is in the simulation model (`d435_depth`, a MuJoCo camera with
  the depth stream's field of view, which nothing renders yet) and in the URDF
  (`camera_link`, `camera_depth_optical_frame`, the frames realsense2_camera
  stamps), but this node takes its mount from `mount.*`, not from TF.
  `sandro_description`'s tests keep the three the same.
- Obstacles are only "high floor": nothing tells a person from a wall, and a
  wall reads high only in the cell of its visible face, unknown behind
  (not +0.25 as said above for everything beyond it).
- From the 2026-11-18 review, unchecked on the robot: the cost while
  balancing (pin the camera and this node off the control loop's core), the
  depth stamps' accuracy, the odometry's scale, coverage at 1.35 m.
