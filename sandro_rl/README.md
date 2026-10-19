# sandro_rl

GPU-parallel reinforcement learning of locomotion policies for Sandro, the
wheeled biped with five-bar legs. The pipeline covers:

- training in MuJoCo Warp through MJX, on the same generated MJCF the ROS
  simulation runs;
- export to ONNX;
- evaluation in CPU MuJoCo with the ROS stack's full latency emulation,
  side by side with the whole-body controller (WBC).

Nothing here touches `ros2_ws/`. Nothing here runs on the robot.

- [docs/RESEARCH.md](docs/RESEARCH.md): the literature survey.
- [docs/DESIGN.md](docs/DESIGN.md): the design notes. They cover:
  - backend benchmarks;
  - observations, actions and rewards;
  - the randomisation table with sources;
  - model sync;
  - evaluation results;
  - a deployment sketch;
  - assumptions.

## Setup

These are Python wheels only, with no system changes. The CUDA runtime comes
with `jax[cuda13]`, and an NVIDIA driver with CUDA 13 support is required.
The versions are pinned in `uv.lock`.

```bash
cd sandro_rl
uv sync
```

The WBC side of the evaluation builds `sandro_sim/tools/closed_loop` and
needs Eigen 3.4 headers. `evaluate.py` uses `.cache/eigen-3.4.0` unless
`EIGEN_DIR` is set:

```bash
mkdir -p .cache && curl -L https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz | tar xz -C .cache
```

## Commands

```bash
# model contract + CPU MuJoCo vs GPU consistency (run after every sync_model.py sync)
uv run python -m sandro_rl.consistency

# tests: CPU-only, then everything (GPU)
uv run pytest -m "not gpu"
uv run pytest

# train (one command, on the GPU); writes checkpoints, train_log.csv, curves.png
uv run python -m sandro_rl.train configs/flat_tracking.yaml --out runs/flat
uv run python -m sandro_rl.train configs/agile_terrain.yaml --out runs/terrain
#   override any key:  --set train.num_envs=4096 --set seed=3
#   fine-tune:         --resume policies/flat_tracking

# export ONNX + policy.json (the deployment contract), verified with onnxruntime
uv run python -m sandro_rl.export runs/flat

# evaluate in CPU MuJoCo with latency emulation
uv run python -m sandro_rl.evaluate runs/flat --compare            # vs WBC: hardware/default/push x nominal/uart_imu/worst
uv run python -m sandro_rl.evaluate runs/flat --scenario default --profile uart_imu --video default.mp4
uv run python -m sandro_rl.evaluate runs/terrain --scenario course --course --video course.mp4   # planks + a hill
uv run python -m sandro_rl.evaluate runs/terrain --scenario expressive --video expressive.mp4  # roll and height

# traces of recordings (truth pitch, speed, yaw rate, leg height, wheel torque)
uv run python -m sandro_rl.plots out.png runs/flat/eval/policy_hardware_nominal.csv runs/flat/eval/wbc_hardware_nominal.csv

# physics throughput of the backends
uv run python -m sandro_rl.bench
```

Run training alone on the GPU. `train.py` preallocates 60 % of GPU memory
for XLA. Tests and the other tools don't preallocate.

## Measured throughput (RTX 5070 Ti)

Training is 8192 environments, the policy at 100 Hz, 12 physics steps per
policy step. Rates are averaged over the whole run, compile included, from
each policy's `train_summary.json`:

| Run | Env steps/s | Physics steps/s | Wall time |
| --- | --- | --- | --- |
| `flat_tracking`, 1000 iterations (197M steps) | 153.7k | 1.84M | 21.3 min |
| `flat_level`, 1500 iterations (295M steps) | 154.9k | 1.86M | 31.7 min |
| `agile_obstacles`, 1500 iterations, 16 planks per world | 127.2k | 1.53M | 38.7 min |
| `agile_terrain`, 1200 + 800 + 500 iterations (DESIGN.md 8), 12 planks and 4 hills per world, pushes | 111–114k | 1.33–1.37M | 36 + 24 + 15 min |

Raw physics from `sandro_rl.bench` (`docs/bench.json`):

- MJX-Warp: 2.60M physics steps/s at 8192 worlds in the `warp_staged`
  graph mode used for training, and 5.18M in the `warp` mode;
- MJX-JAX: 0.74M, on a simplified model, because it cannot collide the tyres
  with the meshes;
- CPU MuJoCo: 89.7k on one thread.

## Policies

Each folder under `policies/` contains:

- the checkpoint, with its config and model fingerprint;
- `policy.onnx` and `policy.json`;
- the training curves;
- an example video;
- `eval/`, with the CSV recordings, plots and `comparison.md`.

| Policy | What it is | Video | Result (CPU MuJoCo, latency emulation) |
| --- | --- | --- | --- |
| [`flat_tracking`](policies/flat_tracking) | First task: balance, forward speed, yaw rate, height on flat ground | `default_sequence.mp4` | Completes all 9 scenario/latency runs (the WBC falls in 4). Yaw tracking is 3× better than the WBC. Standing pitch is twice the WBC's, with a tilted posture |
| [`flat_level`](policies/flat_level) | `flat_tracking` with pitch penalised (−25) | `default_uart_imu.mp4` | Completes all 9. Stands level, calmer than the WBC (pitch RMS 0.013 against 0.023 rad). Drives with more pitch than the WBC |
| [`agile_obstacles`](policies/agile_obstacles) | Superseded. Its planks did not collide (DESIGN.md 8), so it was trained on flat ground | `agile_course.mp4` | Its course result is invalid: only the first plank was solid |
| [`agile_terrain`](policies/agile_terrain) | 1 m/s, 3 rad/s, planks up to 3 cm, hills up to 10 cm with 15° ramps, pushes and torque kicks, and a commanded body roll and height for animation | `course.mp4`, `expressive.mp4` | Crosses four planks and climbs the 8 cm hill, spins 2.5 rad/s on its plateau. Holds 0.22–0.26 rad of roll for 0.25–0.3 commanded. Completes all 9 standard runs (the WBC falls in 4). Height follows only within 0.12–0.18 m |

Results, including where each policy is worse than the WBC, are in
[docs/DESIGN.md](docs/DESIGN.md) sections 7 and 8.

## Layout

| Path | Contents |
| --- | --- |
| `configs/` | task configs; each inherits `base.yaml` and states only what differs. `ros` marks values read from the ROS stack |
| `sandro_rl/sources.py` | reads the MJCF, controllers.yaml, the robot overrides and the launch latency defaults; fingerprint |
| `sandro_rl/robot.py` | model loading and contract check |
| `sandro_rl/env.py` | batched environment: MIT drives at physics rate, latency emulation, commands, rewards, CaT constraints |
| `sandro_rl/randomize.py`, `terrain.py`, `latency.py` | domain randomisation, obstacles, sensor timing |
| `sandro_rl/ppo.py`, `networks.py`, `train.py` | PPO with symmetry loss, asymmetric critic, curriculum |
| `sandro_rl/interface.py` | numpy observation and action contract shared by evaluation and deployment |
| `sandro_rl/export.py`, `evaluate.py`, `plots.py` | ONNX export, CPU evaluation and comparison against the WBC, plots |
| `sandro_rl/consistency.py`, `bench.py` | CPU vs GPU check, backend benchmark |
| `tests/` | model consistency, observation contract, determinism |
