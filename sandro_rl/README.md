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

#   yaw loop held inside the robot's measured margin (DESIGN.md 8, train.loop_margin):
uv run python -m sandro_rl.train configs/agile_terrain_margin.yaml --resume policies/agile_terrain --out runs/agile_margin
#   the same on every measured plant at once, from agile_terrain_v3 (what rl_agile_terrain_v5 is):
uv run python -m sandro_rl.train configs/agile_terrain_v5.yaml --resume policies/agile_terrain_v3 --out runs/agile_v5
#   the same loop for any exported policy (after sandro_rl.export), before it goes on the robot:
uv run python ../recordings/yaw_loop_margin.py runs/agile_margin

# export ONNX + policy.json (the deployment contract), verified with onnxruntime
uv run python -m sandro_rl.export runs/flat

# evaluate in CPU MuJoCo with latency emulation
uv run python -m sandro_rl.evaluate runs/flat --compare            # vs WBC: hardware/default/push x nominal/spi_slow/worst
uv run python -m sandro_rl.evaluate runs/flat --scenario default --profile spi_slow --video default.mp4
uv run python -m sandro_rl.evaluate runs/terrain --scenario course --course --video course.mp4   # planks + a hill
uv run python -m sandro_rl.evaluate policies/step_climb --scenario steps --course steps --video steps.mp4  # 3-6 cm planks, curb
uv run python -m sandro_rl.evaluate runs/terrain --scenario expressive --video expressive.mp4  # roll and height
uv run python -m sandro_rl.evaluate policies/stand_up --scenario stand_up --fallen 1.4 --video up.mp4  # limp fall, then stand up
uv run python -m sandro_rl.evaluate policies/fun_show --scenario routine:show:0.5 --video show.mp4  # a routine of the panel's Drive page at 50 % speed
uv run python scripts/routine_runs.py   # every routine with its policy: falls, wheel speed, floor used (docs/demo_reel.mp4, ../docs/DEMO.md)

# obstacle avoidance: goals among posts and walls, seen by a simulated planar range scanner
uv run python -m sandro_rl.train configs/avoid.yaml --out runs/avoid
uv run python -m sandro_rl.evaluate runs/avoid --scenario avoid --course slalom --video slalom.mp4   # or --course wall
# flow policies (FPO++) instead of the Gaussian: the same tasks, train.algorithm: fpo
uv run python -m sandro_rl.train configs/flat_tracking_fpo.yaml --out runs/flat_fpo
uv run python -m sandro_rl.train configs/avoid_fpo.yaml --out runs/avoid_fpo

# debugging without a GPU: MJX's JAX backend on the CPU (tyre-floor contacts only, DESIGN.md 12)
uv run python -m sandro_rl.train configs/avoid.yaml --out /tmp/avoid --set env.impl=jax --set env.graph_mode=null \
    --set train.num_envs=8 --set train.iterations=3

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
| [`flat_tracking_ppo`](policies/flat_tracking_ppo) | `flat_tracking` retrained on the 2026-11 body (1500 iterations, 41 min): the PPO baseline for FPO and its teacher | | Completes all 12 runs; pitch RMS 0.021–0.027 rad |
| [`flat_level_sysid`](policies/flat_level_sysid), [`flat_level_fast_sysid`](policies/flat_level_fast_sysid) | `flat_level_real` and `flat_level_fast` on the identified yaw plant: a heavy rotor (15–35 × the armature) behind a soft, damped drive train, the tyres' torsion and the wheels' friction low (DESIGN.md, the 8–10 Hz yaw mode). Controllers `rl_flat_level_sysid`, `rl_flat_level_fast_sysid`, off the gamepad switch and the panel | | On the fitted plant 0.001–0.025 rad/s of 5–15 Hz yaw where the earlier policies ring at 0.16–0.66, and the best tracking (yaw error 0.014–0.018 rad/s). On the robot (2026-11-16) slower than before and still buzzing |
| [`flat_tracking_fpo`](policies/flat_tracking_fpo) | `flat_tracking` as a flow policy trained with FPO++ (`flat_tracking_fpo_warm.yaml`, then `_finetune.yaml`; DESIGN.md 13). Not runnable by `RlPolicyController` yet | | Completes all 12 runs; pitch RMS 0.015–0.019 rad (steadier than PPO), speed and yaw tracking about as good except under pushes |
| [`flat_level`](policies/flat_level) | `flat_tracking` with pitch penalised (−25) | `default_uart_imu.mp4` | Completes all 9. Stands level and as calmly as the WBC. Drives with more pitch than the WBC |
| [`flat_level_real`](policies/flat_level_real) | `flat_level` fine-tuned on 10–80 ms of wheel drive-train lag, with stronger action smoothing. Deployed as `rl_flat_level` | `eval/hardware_lag50.png` | No wheel ringing from 0 to 80 ms of lag, where `flat_level` rings at 7–10 Hz (the robot's 7.5 Hz) and falls at 80 ms. At 40 ms it tracks as `flat_level` does without lag. Completes all 9 |
| [`agile_obstacles`](policies/agile_obstacles) | Superseded. Its planks did not collide (DESIGN.md 8), so it was trained on flat ground | `agile_course.mp4` | Its course result is invalid: only the first plank was solid |
| [`fun_split`](policies/fun_split), [`fun_fast`](policies/fun_fast), [`fun_one_leg`](policies/fun_one_leg) | For fun (DESIGN.md 16): v6 in a split stance, v6 up to 2 m/s, and standing on the left wheel for 2 s on the hop input. Deployed as `rl_fun_split`, `rl_fun_fast`, `rl_fun_one_leg`, not yet run on the robot | | Split 11 cm, course 27 of 27. Fast: 1.7 m/s, no fall in 18 runs, but 32 rad/s at the wheels against the drives' default limit of 19.5. One leg: 87 of 88 requests, 2.06 s on one wheel, 1 fall in 18 runs (`scripts/one_leg_runs.py`); [`fun_one_leg_long`](policies/fun_one_leg_long) (`rl_fun_one_leg_long`): 7.0–8.0 s in 54 of 54 requests, no fall in 18 runs, but not a clean stand: with the leg links colliding it is on the left tyre alone for 83 % of the hold, hops for 5 % and leans on a leg link for 11 % |
| [`fun_show`](policies/fun_show) | For a demo (DESIGN.md 18): `jump` with spins up to 5.5 rad/s, roll to 0.4 rad at 2 rad/s, the height at 0.4 m/s and one hop per request. Deployed as `rl_fun_show` (panel list), not yet run on the robot | `fun_show.mp4` | `scripts/show_runs.py`, 18 runs: no fall; 5.0 rad/s reached on the fitted plant, 2.4–2.6 on the MJCF's stiffer tyre torsion; stops within 1 s; roll 0.35 rad; height 0.10–0.18 m; hops 36 of 36 (5–7 cm), also in a spin; wheels below 19.1 rad/s outside the hops; gate 0.32 |
| [`fun_stride`](policies/fun_stride) | For a demo (DESIGN.md 19): `agile_terrain_v6` with the roll stick as the legs' fore-aft split (up to 16 cm, at 0.6 m/s), so sweeping it strides. Body level, height 0.11–0.15 m. Deployed as `rl_fun_stride` (panel list), not yet run on the robot | `fun_stride.mp4` (side), `fun_stride_front.mp4` | `scripts/stride_runs.py`, 18 runs: no fall; 15 cm held of 16 asked, 7 of 8; strides of ±11 cm at 0.5 Hz and ±8 cm at 1 Hz standing, driving and turning; roll within 0.02 rad; the heading wiggles by up to 1 rad/s in a stride; only the tyres touch; wheels below 12.2 rad/s; gate 0.23, no crossing of the whole loop. Falls on the plank course at 0.8 m/s (6 of 6, at the 2 or the 2.5 cm plank): flat floors |
| [`fun_bow`](policies/fun_bow) | For a demo (DESIGN.md 20): `fun_stride`'s lineage with the roll stick as the body's lean (`commands.lean`), up to 0.6 rad forward and back, the legs carrying the axles under the centre of mass; sweeping it nods. Height 0.11–0.15 m, 0.6 m/s. Deployed as `rl_fun_bow` (panel list), not yet run on the robot | `fun_bow.mp4` (side) | `scripts/bow_runs.py`, 18 runs: no fall; 0.56 rad held forward and 0.60 back of 0.6 asked, standing, at 0.4 m/s (0.34 m/s kept) and turning; nods of ±0.36 rad at 0.5 Hz and ±0.21 at 1 Hz; roll within 0.01 rad; the axle creeps at 0.02 m/s holding a lean; only the tyres touch, knees 11 cm up; leg torque up to 4.9 Nm; wheels below 10 rad/s on the fitted plant, 24 on the MJCF with the slow SPI IMU in a lean back while driving; gate 0.30 and 0.33; push, default and hardware 18 of 18 |
| [`fun_hop_high`](policies/fun_hop_high) | For a demo (DESIGN.md 21): `fun_show` with every hop request paid up to 25 cm of wheel clearance (`commands.jump.target_m`), the legs' action range 1.2 rad and their speed held under the drives' limit (`constraints.leg_speed`). Hops in a row, one press each. Deployed as `rl_fun_hop_high` (panel list), not yet run on the robot | `fun_hop_high.mp4`, `_slow.mp4` (side) | `scripts/hop_runs.py`, 18 runs: no fall; 108 of 108 hops, 11.2–13.5 cm under the lower wheel (12.4 median; 11.7 on the fitted plant, 13.2 on the MJCF) where `fun_show` gives 5–7; 0.24 s in the air, down at 1.2–1.8 m/s, no bounce; only the tyres touch, knees 7.6 cm up; legs at 9.0 Nm and up to 17.4 rad/s (limit 19.5), wheels to 24 rad/s in the air; the height command is followed 1–2.5 cm low (0.078–0.164 m for 0.09–0.19); gate 0.17, no crossing; push, default and hardware 18 of 18 |
| [`fun_one_leg_v4`](policies/fun_one_leg_v4) | For a demo (DESIGN.md 23): the one-wheel stand as a balance. From `fun_show`; tilt allowed to 0.85 rad (the weight is over the tyre at 0.6–0.8), the stand paid more the longer it lasts without a tap or a hop (`commands.jump.streak_s`). Deployed as `rl_fun_one_leg_v4` (panel list), not yet run on the robot | `fun_one_leg_v4.mp4`, `_side.mp4` | `scripts/one_leg_runs.py --hold 8`: 54 of 54 requests, no fall in 18 runs, 93 % on the left tyre alone, leg links at least 6 mm off the floor. On the fitted plant 27 of 27 holds without one tap or hop (7 s), turning on the spot at 1.6 rad/s; on the stock model's grippier tyre none, 7 hops or taps per hold (`fun_one_leg_long`: 15, and never 3 s without one). Roll 0.8 rad, 0.2 from the robot's fall angle |
| `fun_fast_v2` (configs only, not deployed) | DESIGN.md 24: driving fast with the wheel drives' velocity limiter (19.5 rad/s, 1.31 m/s at the tyre) modelled as ODrive's torque-mode one, in training (`action.wheel_speed_limit`) and in the evaluator (`--sim wheel_speed_limit=19.5`). An assumption about the drives, not a measurement | | `scripts/fast_runs.py`, 18 runs with the limiter: 0.82–0.86 m/s with 1 fall, or 0.95–0.98 m/s (`fun_fast_v2c`) falling at every 12 N shove; no faster than what is deployed. **Under the same limiter `fun_fast` falls in 18 of 18 runs when asked for 1.5 or 2 m/s (keep it at 1.0: `speed_fraction` 0.5), and `agile_terrain_v6` in 3 of 18 when shoved at full stick** |
| [`fun_flip_strong`](policies/fun_flip_strong) | **Simulation only, not deployed, and not the robot's drives** (DESIGN.md 22): a backflip on the hop input (`commands.jump.flip`), asked for by the owner. With the real drives no flip was found (`configs/fun_flip.yaml`: 1.4–1.5 rad turned of the 6.3, on its back in 18 of 18; `fun_flip_free.yaml`). This is the attempt with the leg drives at 18 Nm, half their damping and no speed limit. Not a flip yet: it does not land | `fun_flip_strong.mp4`, `_slow.mp4` (side) | `scripts/hop_runs.py --flip`, 18 runs: 27–31 cm up, 5.3–5.8 rad of the 6.28 turned (5.1–5.5 of it in 0.36 s in the air), then down on the torso or a leg in 18 of 18, never upright. The legs turn at up to 53 rad/s (the drives' limit is 19.5) at up to 10 Nm; touchdown at 2.5–3.3 m/s. With the drives' 9 Nm and their damping, without the speed limit: 13 cm and 1.2–2.0 rad |
| [`stand_up_v2`](policies/stand_up_v2) | `stand_up` retrained on slippery floors (the floor's friction randomised, not only the tyres'), weak drives and the measured left-against-right plants (DESIGN.md, stand_up_v2). Iteration 500 of 1000. Deployed as `rl_stand_up_v2` next to `rl_stand_up`, not yet run on the robot | | `scripts/stand_up_runs.py`: 180 of 180, and 300 of 300 on the hard grid (`stand_up` 285, 46 of 60 on a slippery floor). No loop crossing standing (`stand_up` 0.81) |
| [`agile_step`](policies/agile_step) | Three answers to the terrain in the camera's profile: drive over edges below 2 cm, hop onto steps of 2–6 cm, stop before anything taller; hops on request too (`/cmd_vel` `linear.y`, L1 + cross). From `jump` (DESIGN.md 15). Deployed as `rl_agile_step`, not yet run on the robot | | `scripts/three_runs.py`, 18 runs: onto a 3 cm step 18, 4 cm 14, 5 cm 11 (camera off 0; blind v6 18, 14, 7 with 1–5 falls); never touches the 15 cm block (v6 meets it at 0.6–0.7 m/s). Requested hops 96 of 96, 6–8 cm. Gate 0.34 |
| [`agile_step_v2`](policies/agile_step_v2) | `agile_step` retrained on the errors the robot's own perception makes, after that perception ran in the evaluator's loop (`--sim depth=1`) and three faults in its map were fixed (DESIGN.md 17). Deployed as `rl_agile_step_v2` next to `rl_agile_step` (panel only), not yet run on the robot | | `scripts/three_runs.py POLICY:depth=1`, 18 runs: never touches the 15 cm block (`agile_step` under the same perception meets it at up to 0.37 m/s in 9 runs); stops in 12, turns past it in 3. Onto a 3 cm step 18 (driven, not hopped), 5 cm 8 with 1 fall (`agile_step` 9 with 4). Requested hops 30 of 30. Gate 0.34 |
| [`agile_perceptive`](policies/agile_perceptive) | `agile_terrain_v6` with the depth camera's terrain profile as an observation (`sandro_perception`, DESIGN.md 14), planks to 5 cm. Not deployed | | Reads the profile but gains nothing from it: the course at 0.5 m/s in 15 of 27 runs with the camera and 15 without, against the blind v6's 27. Kept as the reference for the controller's profile input |
| [`agile_terrain_v6`](policies/agile_terrain_v6) | `agile_terrain_v5` with a margin of 0.35 on the measured plants (DESIGN.md 2026-11-18). Deployed as `rl_agile_terrain_v6` | | No loop crossing left standing on the measured plants (v5 0.37–0.42, and 0.60 at 26 Hz through the legs behind the 3 Hz wheel filter; v6 0.36 there). Tracking as v5; the course at 0.5 m/s in 27 of 27 runs with and without the filter, at a picked checkpoint, on the model without the depth camera; 11 of 27 with its 99 g (DESIGN.md 28). Not yet on the robot |
| [`agile_terrain_v7`](policies/agile_terrain_v7) | `agile_terrain_v6` fine-tuned with the wheel drives' velocity limiter in the loop (DESIGN.md 26; the limiter is an assumption, so its limit and gain are varied and it is absent in 30 % of the episodes). Deployed as `rl_agile_terrain_v7`, panel list only; v6 stays the default. Not run on the robot | | `scripts/fast_runs.py`, 18 runs at full stick with the limiter at 19.5 rad/s: no fall when shoved (v6: 3); none without the limiter; with the limiter at 17.5 both fall in 18 of 18 (v7 at 0.8 of the stick: none). Gate without the wheel filter 0.23, no whole-loop crossing (v6 0.22); behind the 3 Hz filter 0.27 and 0.41 (v6 0.20 and 0.36). Course at 0.5 m/s 20 of 27 (v6: 11 of 27 on the same model, the robot with the depth camera; DESIGN.md 28) |
| [`agile_terrain_v5`](policies/agile_terrain_v5) | `agile_terrain_v3` fine-tuned with its yaw and roll loops held inside the margin of the robot's measured plants (`train.loop_margin`, DESIGN.md 2026-11-17). Deployed as `rl_agile_terrain_v5`, without the wheel filter | | Loop gain at the yaw and roll crossings 0.37–0.42 (`agile_terrain` 0.89–1.04, v3 0.55–0.82; 1 oscillates), no roll crossing through the legs at 14 Hz. On the fitted plant yaw error 0.035–0.043 rad/s; the course at 0.5 m/s in 27 of 27 runs, though that varies between checkpoints (18 of 27 with the depth camera's mass, DESIGN.md 28). On the robot (2026-11-18) far quieter than the earlier policies, run with the 3 Hz wheel filter |
| [`agile_terrain_v3`](policies/agile_terrain_v3) | `agile_terrain` fine-tuned on the 2026-11 findings: MIT joint state, the identified wheel drive train, low tyre torsion and wheel friction, heavy leg rotors (DESIGN.md 2026-11-17). Deployed as `rl_agile_terrain_v3` | | On the fitted plant yaw error 0.024–0.028 rad/s against 0.059–0.082 for `agile_terrain` with its 3 Hz filter, speed error lower, 0.003–0.004 rad/s of 9 Hz yaw against 0.016–0.018; the course at 0.5 m/s in 27 of 27 runs (15 of 18), and still 27 with the depth camera's mass (DESIGN.md 28). Pitch RMS a little higher |
| [`agile_terrain`](policies/agile_terrain) | 1 m/s, 3 rad/s, planks up to 3 cm, hills up to 10 cm with 15° ramps, pushes and torque kicks, and a commanded body roll and height for animation | `course.mp4`, `expressive.mp4` | On the weighed robot it crosses planks up to 20 mm but not the 25 mm one: at 0.5 m/s it bounces back from it, and at 0.7 m/s or more it falls over it. Holds 0.23–0.27 rad of roll for 0.25–0.3 commanded. Completes all 9 standard runs (the WBC falls in 3). Height follows only within 0.12–0.18 m |
| [`jump`](policies/jump) | `agile_terrain` with 10 Nm on every drive and a commanded jump (2–8 cm). A base to build dynamic tasks on; not meant for the hardware yet | `jump.mp4` | Hops three times per jump command, 92–120 mm of wheel clearance, standing and at 0.5 m/s, and lands every time. Completes all 9 standard runs. The height command barely matters, and it falls twice as often as `agile_terrain` in training. Stands with the legs about 75 mm apart fore and aft, an exploit (DESIGN.md 8) |
| [`step_climb`](policies/step_climb) | `jump` on planks up to 6.5 cm and steeper ramps, on the weighed robot. Crossing planks is not commanded; commanded jumps are kept | `steps.mp4` | Crosses the 3 cm plank on 5 of 5 seeds and the 4 cm one on 2; never 5 or 6 cm, never the 6 cm curb. Stands with its legs 60 mm apart fore and aft (an exploit, docs/DESIGN.md 8), and its heading drifts after crossings, so in `steps.mp4` it passes the later planks beside them. Not exportable to ROS yet (jump inputs) |
| [`stand_up`](policies/stand_up) | Gets up from switched off: limp falls onto the folded legs, front or back, leaning or lying flat, then balances and drives as `flat_level`. Collides the leg links (boxes fitted to the CAD). The drives' 9 Nm envelope (10 Nm before 2026-11) | `stand_up_front.mp4`, `stand_up_back.mp4` | Up and staying up in 18 of 18 falls (pitches ±0.3 to ±1.4 rad) and 36 of 36 on held-out falls across the three latency profiles. Up in 0.1–1.6 s. Legs within 3 mm fore and aft once up. Completes all 9 standard runs. Refit on the 2026-11 body: up from 120 of 120 falls (pitches ±0.3 to ±1.4 rad, three latency profiles, 40 and 80 ms of wheel lag) in 0.1–1.8 s. Deployed as `rl_stand_up` (`safety.start_lying`: engages limp where the robot lies, fall cut-off once upright) |

Two task configs have no trained policy yet (DESIGN.md sections 12 and 13):

| Config | What it is | Status |
| --- | --- | --- |
| `avoid` | Drive to goals 1.5–4 m away among posts and short walls taller than the robot, at a commanded cruise speed up to 1.2 m/s, through a simulated 16-beam planar range scanner. Touching a pillar is a failure (after Agile But Safe, He et al. 2024) | Code and CPU tests only. Sandro has no range sensor; not exportable to ROS |
| `flat_tracking_fpo`, `avoid_fpo` | The same tasks with a flow-matching policy trained by FPO++ (Yi et al. 2026) instead of the Gaussian | Code and CPU tests only. On a toy problem it keeps both of two equally good actions, where the Gaussian leans to one; with one right answer it learns slower. Not exportable to ROS |

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
| `sandro_rl/env.py` | batched environment: MIT drives at physics rate, latency emulation, commands (speeds or goals), range scanner, rewards, CaT constraints |
| `sandro_rl/randomize.py`, `terrain.py`, `latency.py` | domain randomisation, obstacles (planks, hills, pillars) with analytic ray casts and clearances, sensor timing |
| `sandro_rl/ppo.py`, `networks.py`, `train.py` | PPO with symmetry loss, asymmetric critic, curriculum |
| `sandro_rl/fpo.py` | flow-matching policies trained with FPO++, and their ONNX export |
| `sandro_rl/interface.py` | numpy observation and action contract shared by evaluation and deployment, goal odometry |
| `sandro_rl/export.py`, `evaluate.py`, `plots.py` | ONNX export, CPU evaluation and comparison against the WBC, plots |
| `sandro_rl/consistency.py`, `bench.py` | CPU vs GPU check, backend benchmark |
| `sandro_rl/refit.py`, `scripts/refit_lag.sh`, `scripts/package_refit.sh` | fine-tune policies onto the wheel drive-train lag, then package and re-evaluate them |
| `tests/` | model consistency, observation contract, determinism, rays against `mj_ray`, the avoid task, FPO++ |
