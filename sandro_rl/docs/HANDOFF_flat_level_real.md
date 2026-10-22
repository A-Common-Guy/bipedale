# Handoff: retrain flat_level against wheel drive-train lag

Status 2026-10-22: done on the GPU machine. `policies/flat_level_real` meets
the acceptance below, and every other policy was refitted to the same lag
(docs/DESIGN.md section 8, "Wheel drive-train lag"). `export_ros` has been
run for `flat_level_real:rl_flat_level` and the refitted
`agile_terrain:rl_agile_terrain`. Remaining: `deploy/pi/pi build` and a
launch with wheel kd 0.4 and `wheel_filter_hz: 0`, on the owner's decision.

Status 2026-10-21. Next step needs a GPU (MuJoCo Warp); the laptop that did
the hardware work has a 4 GB RTX 500 and cannot train.

## Problem

`flat_level` balances in simulation but on the robot the wheels oscillate
fast back and forth (operator cut it). Recordings:
`recordings/rl_flat_vibration_2026-10-21/` (stock) and
`recordings/rl_flat_A_kd025/` (wheel kd 0.25, `wheel_filter_hz` 4).

## Findings

- 7.4-7.5 Hz limit cycle in the wheel actions: both wheels in phase, actions
  +-0.7, 2.3 Nm RMS per wheel (0.3-0.5 Nm in simulation).
- Lowering wheel kd 0.4 -> 0.25 did not move the frequency: the loop is
  delay-limited, not gain-limited.
- Body pitch acceleration lags the reported wheel torque by 24-25 ms.
- The IMU lags the encoders by only 9 ms (rocking test with the left wheel
  blocked, `recordings/imu_rock3_left_2026-10-21.csv`, gain -0.94,
  correlation 1.00; script `recordings/imu_delay_one_wheel.py`). So the lag
  is in the wheel drive train (backlash / compliance), not in sensing.
- Training only randomised 0.5-3 ms of command delay, and the env's
  `command_delay` cannot exceed one policy step (10 ms).
- Policy inference on the Pi is 330-569 us worst case: not the cause.
- `evaluate.py --sim wheel_lag=...` reproduces the ringing with the current
  policy: 10 ms -> 11.3 Hz, 20 ms -> 10 Hz, 30 ms -> 8.1 Hz (about 2 Nm
  summed). The robot is near 30-40 ms.

Analysis scripts: `recordings/analyse_vibration.py`, `check_imu.py`,
`imu_delay.py`, `imu_delay_one_wheel.py`.

## What changed in code

- `sandro_rl/env.py`: per-episode `wheel_lag` sampled from
  `randomization.drives.wheel_lag_s` (nominal: `drives.nominal_wheel_lag_s`,
  default 0). Applied as a first-order lag on the wheel actuator torque at
  the physics rate (`info["drive_tau"]` carries the state). Rewards and
  constraints still see the commanded torque. Critic observation unchanged,
  so `--resume` from `policies/flat_level` works.
  **Never executed yet** (written on a machine without a GPU): run a short
  smoke training first.
- `sandro_rl/evaluate.py`: `--sim wheel_lag=<seconds>` for CPU evaluation.
- `configs/flat_level_real.yaml`: inherits `flat_level.yaml`,
  `wheel_lag_s: [0.005, 0.05]`, stronger `action_rate` (-0.08) and
  `action_accel` (-0.04).
- ROS side (already on the robot): `RlPolicyController` has
  `action.wheel_filter_hz` (first-order filter on wheel actions only, 0 =
  off) and a hot handover between policies; new gamepad teleop
  (`joy_teleop.py`) with soft policy switch and pad LED showing the policy.

Known: 3 `sandro_rl` tests fail without a GPU; they fail identically without
these changes.

## To do on the GPU machine

```bash
cd sandro_rl
# 1. smoke test the lag code path
uv run python -m sandro_rl.train configs/flat_level_real.yaml --out runs/smoke \
    --resume policies/flat_level --set train.iterations=10
# 2. full fine-tune
uv run python -m sandro_rl.train configs/flat_level_real.yaml \
    --out runs/flat_level_real --resume policies/flat_level
# 3. check it no longer rings with lag (compare with policies/flat_level)
for lag in 0 0.02 0.035 0.05; do
  uv run python -m sandro_rl.evaluate runs/flat_level_real --sim wheel_lag=$lag
  uv run python -m sandro_rl.evaluate policies/flat_level  --sim wheel_lag=$lag
done
```

If resuming does not converge, train from scratch with the same config.

Acceptance: at `wheel_lag=0.035` no sustained 7-12 Hz oscillation in the
wheel actions, wheel torque RMS close to the lag-free case, tracking and
upright metrics not worse than `policies/flat_level` at lag 0.

## Deploy (after acceptance)

1. Copy `runs/flat_level_real` to `policies/flat_level_real` (or replace
   `policies/flat_level` if it is strictly better).
2. Export both policies together (the script rewrites `rl_policies.yaml`
   from the list it is given):
   `uv run python -m sandro_rl.export_ros policies/flat_level_real:rl_flat_level policies/agile_terrain:rl_agile_terrain`
3. Commit, then on the robot laptop: `deploy/pi/pi build` and launch with
   `wheel_filter_hz: 0` first (the filter adds lag of its own and was only
   a stopgap; see `recordings/real_overrides_*.yaml`).

`agile_terrain` likely has the same problem; once `flat_level_real` works on
hardware, apply the same `wheel_lag_s` randomisation to it.
