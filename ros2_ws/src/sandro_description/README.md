# sandro_description

ROS 2 Jazzy description resources for Sandro.

## Models

- `urdf/sandro.urdf.xacro` is the `robot_state_publisher` model. It exposes the
  six stable actuated joint names and `imu_link`.
- `urdf/sandro.mujoco.urdf.xacro` adds
  `mujoco_ros2_control/MujocoSystem`.
- `urdf/sandro.real.urdf.xacro` adds the GIM6010 system and BNO085 sensor from
  `sandro_hardware`.
- `mjcf/scene_floating.xml` and `mjcf/robot_full_floating.xml` are byte-for-byte
  copies of the canonical Onshape export. Their referenced STL files are kept
  below `mjcf/assets`, preserving MuJoCo's relative `meshdir`.
- `mjcf/scene_ros2.xml` and `mjcf/robot_ros2.xml` are generated from the export
  by `tools/sync_model.py` for `mujoco_ros2_control`: unit-gain effort
  actuators, zeroed keyframe `ctrl`, and the six actuated hinges
  re-parametrised into the logical joint convention described below.

## Joint convention

Every controller, the calibrated physical drives (`sign` and `offset_rev` in
`sandro_hardware`) and the simulated hinges share one convention:

- Leg joints are zero when the upper link hangs straight down toward the wheel
  and positive when the link opens away from the leg centre. This is the frame
  of the planar five-bar kinematics in `sandro_controllers` and of the legacy
  `SandroControl` runtime.
- Wheel joints are positive when the robot rolls forward, on both sides.
- The inner/outer motor names are mirrored between the sides: `right_motor_inner`
  is the rear motor of the right leg while `left_motor_inner` is the front motor
  of the left leg. Planar leg +x therefore points forward on the right and
  backward on the left (`forward_sign` +1 / -1 in the whole-body controller).

The raw Onshape export does not follow this convention: its leg hinge zeros are
unrelated between sides and its left wheel hinge points the opposite way, so
equal efforts would spin the robot instead of driving it. `tools/sync_model.py`
holds the per-joint `(sign, zero)` table `JOINT_FRAMES` and applies it while
generating `robot_ros2.xml` through a flipped hinge axis and a joint `ref`, and
converts the `stand` keyframe accordingly. Body poses are unchanged.

`tools/derive_joint_frames.py` recomputes that table from the export geometry
(it needs the `mujoco` Python package, which the ROS image does not ship) and
`--check` compares it with the committed one. Rerun it after every CAD export.

The URDF is intentionally a schematic tree. A URDF cannot express each
five-bar linkage's closed loop, so its lower links end as unconnected visual
branches and the wheel joints attach directly to `base_link`. It has no
inertials and must not be used as a physics model. The MJCF retains the
exported inertias, equality constraints, contacts, and sensors.

## Updating the canonical model

After changing `onshape_export/robot`, run:

```bash
ros2_ws/src/sandro_description/tools/sync_model.py sync
ros2_ws/src/sandro_description/tools/sync_model.py check
```

`check` also verifies the six-joint controller contract and IMU site. Both
commands are deterministic; unchanged files are not rewritten. Then rerun
`tools/derive_joint_frames.py --check` from an environment with `mujoco`, and
update `JOINT_FRAMES` if the export moved a joint frame.
