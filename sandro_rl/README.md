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
uv run python -m sandro_rl.evaluate policies/step_climb --scenario steps --course steps --video steps.mp4  # 3-6 cm planks, curb
uv run python -m sandro_rl.evaluate runs/terrain --scenario expressive --video expressive.mp4  # roll and height
uv run python -m sandro_rl.evaluate policies/stand_up --scenario stand_up --fallen 1.4 --video up.mp4  # limp fall, then stand up

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
| `jump`, 2500 iterations (492M steps), `agile_terrain` plus jumps at 10 Nm | 109.8k | 1.32M | 74.6 min |
| `step_climb`, 1500 iterations from `jump` on the weighed robot (checkpoint 1200 kept) | 106.5k | 1.28M | 46.1 min |
| `stand_up`, 3000 iterations (590M steps) from `flat_level`, 4096 worlds with leg collision boxes (checkpoint 2000 kept) | 103.1k | 1.24M | 95.3 min |

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
| [`flat_tracking`](policies/flat_tracking) | First task: balance, forward speed, yaw rate, height on flat ground | `default_sequence.mp4` | Completes all 9 scenario/latency runs (the WBC falls in 3). Yaw tracking is about 5× better than the WBC. Stands as calmly as the WBC (pitch RMS 0.017–0.033 rad), tilted forward by up to 0.02 rad |
| [`flat_level`](policies/flat_level) | `flat_tracking` with pitch penalised (−25) | `default_uart_imu.mp4` | Completes all 9. Stands level and as calmly as the WBC. Drives with more pitch than the WBC |
| [`flat_level_real`](policies/flat_level_real) | `flat_level` fine-tuned on 10–80 ms of wheel drive-train lag, with stronger action smoothing. Deployed as `rl_flat_level` | `eval/hardware_lag50.png` | No wheel ringing from 0 to 80 ms of lag, where `flat_level` rings at 7–10 Hz (the robot's 7.5 Hz) and falls at 80 ms. At 40 ms it tracks as `flat_level` does without lag. Completes all 9 |
| [`agile_obstacles`](policies/agile_obstacles) | Superseded. Its planks did not collide (DESIGN.md 8), so it was trained on flat ground | `agile_course.mp4` | Its course result is invalid: only the first plank was solid |
| [`agile_terrain`](policies/agile_terrain) | 1 m/s, 3 rad/s, planks up to 3 cm, hills up to 10 cm with 15° ramps, pushes and torque kicks, and a commanded body roll and height for animation | `course.mp4`, `expressive.mp4` | On the weighed robot it crosses planks up to 20 mm but not the 25 mm one: at 0.5 m/s it bounces back from it, and at 0.7 m/s or more it falls over it. Holds 0.23–0.27 rad of roll for 0.25–0.3 commanded. Completes all 9 standard runs (the WBC falls in 3). Height follows only within 0.12–0.18 m |
| [`jump`](policies/jump) | `agile_terrain` with 10 Nm on every drive and a commanded jump (2–8 cm). A base to build dynamic tasks on; not meant for the hardware yet | `jump.mp4` | Hops three times per jump command, 92–120 mm of wheel clearance, standing and at 0.5 m/s, and lands every time. Completes all 9 standard runs. The height command barely matters, and it falls twice as often as `agile_terrain` in training. Stands with the legs about 75 mm apart fore and aft, an exploit (DESIGN.md 8) |
| [`step_climb`](policies/step_climb) | `jump` on planks up to 6.5 cm and steeper ramps, on the weighed robot. Crossing planks is not commanded; commanded jumps are kept | `steps.mp4` | Crosses the 3 cm plank on 5 of 5 seeds and the 4 cm one on 2; never 5 or 6 cm, never the 6 cm curb. Stands with its legs 60 mm apart fore and aft (an exploit, docs/DESIGN.md 8), and its heading drifts after crossings, so in `steps.mp4` it passes the later planks beside them. Not exportable to ROS yet (jump inputs) |
| [`stand_up`](policies/stand_up) | Gets up from switched off: limp falls onto the folded legs, front or back, leaning or lying flat, then balances and drives as `flat_level`. Collides the leg links (boxes fitted to the CAD). 10 Nm on every drive | `stand_up_front.mp4`, `stand_up_back.mp4` | Up and staying up in 18 of 18 falls (pitches ±0.3 to ±1.4 rad) and 36 of 36 on held-out falls across the three latency profiles. Up in 0.1–1.6 s. Legs within 3 mm fore and aft once up. Completes all 9 standard runs. Needs a controller mode that engages lying down (DESIGN.md 11) |

`flat_tracking`, `agile_terrain`, `jump`, `step_climb` and `stand_up` were then fine-tuned on 10–80 ms of wheel drive-train lag, which made `flat_level` ring at 7.5 Hz on the robot. The rows above are from before that refit; its results, at 40 ms of lag, are in [docs/DESIGN.md](docs/DESIGN.md) section 8 (wheel drive-train lag) and each policy's `eval/lag_sweep.md`. The videos predate it.

`flat_tracking`, `flat_level`, `agile_terrain` and `jump` were fine-tuned on the weighed robot (4.825 kg, loaded tyre radius
0.0672 m) from their CAD-mass checkpoints; the results above are on that
model. Results, including where each policy is worse than the WBC, are in
[docs/DESIGN.md](docs/DESIGN.md) sections 7 and 8; the refit is the last
subsection of section 8.

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
| `sandro_rl/refit.py`, `scripts/refit_lag.sh`, `scripts/package_refit.sh` | fine-tune policies onto the wheel drive-train lag, then package and re-evaluate them |
| `tests/` | model consistency, observation contract, determinism |
