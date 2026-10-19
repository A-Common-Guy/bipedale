<!-- Literature survey for sandro_rl. Every claim carries its source link; the
survey was compiled on 2026-10-16 from arXiv/DOI records and the official
MuJoCo, MJX and MuJoCo Warp documentation. How the recommendations were
applied, and where this project departs from them, is in DESIGN.md. -->

# Literature survey: GPU-parallel RL and sim-to-real for Sandro, a small wheeled biped

**Bottom line:**
- Every MuJoCo feature Sandro uses is supported by both MJX and MuJoCo Warp: condim 4, connect equality constraints, frictionloss, armature and cylinder–plane contact.
- MJX's JAX backend can't collide the cylinder tyres with heightfields, meshes or boxes. Anything beyond a flat plane needs MuJoCo Warp, or a different tyre primitive.
- The wheel-legged RL work that reached hardware converges on one recipe:
  - leg position targets tracked by PD in the drive, plus wheel velocity targets;
  - an asymmetric critic that is given base velocity;
  - randomisation of 0–20 ms of delay.

Everything was checked against arXiv IDs or DOIs (titles and authors through the arXiv API) and the official docs. Venues are given only where I confirmed them. The last section lists the leads I could not verify.

---

## 1. RL for wheel-legged and wheeled-bipedal robots

- **Ascento, model-based baseline.** Hierarchical whole-body control with closed-form dynamics of the kinematic loops, and LQR as a motion task for the non-minimum-phase balance. [Klemm 2020, RA-L 5(2)](https://arxiv.org/abs/2005.11431). *Hardware: yes, no RL.* The original platform paper: [Klemm 2019, ICRA](https://arxiv.org/abs/2005.11435).
- **Ascento RL (ICRA 2024).** [Chamorro 2024](https://arxiv.org/abs/2402.06143), [DOI](https://doi.org/10.1109/icra57147.2024.10610069). *Hardware: yes, 15 cm steps climbed blind.*
  - **Training:** PPO from rsl_rl in Isaac Gym, 4096 envs, game-inspired curriculum.
  - **Actions:** "for the articulated joints … goal positions to be tracked by a low-level PD controller … target angular velocities for the wheeled joints."
  - **Actor observations:** angular velocity (±20 % noise), projected gravity, joint positions and velocities, last action, commands.
  - **Critic only (privileged):** linear velocity, base height, terrain heights, wheel contact forces, friction.
  - **Randomisation:** friction (too much grip produced an exploit that didn't transfer), pushes of 0–0.5 m/s. Delay was "particularly important": a 50 % chance of using the previous step's state, roughly 20 ms.
  - The thesis version notes that the current Ascento replaced the leg linkage with direct-drive hip and knee joints, so its RL robot no longer has kinematic loops. [Chamorro thesis 2024](https://publications.polymtl.ca/57991/1/2024_SimonChamorro.pdf)
- **ANYmal on wheels (Science Robotics 2024).** [Lee 2024, Sci. Robot. 9(89)](https://arxiv.org/abs/2405.01792), [DOI](https://doi.org/10.1126/scirobotics.adi9641). *Hardware: yes, kilometre-scale missions in Zurich and Seville.*
  - **Actions:** "16-dimensional … joint position commands (12 joints) and wheel velocity commands (4)" sent to the actuators' own PD loops. The engineered gait pattern generator (CPG) was dropped.
  - **Training:** a privileged teacher, then a recurrent (GRU) student distilled with DAgger.
  - **Actuators:** leg joints are simulated with an actuator network. Wheels use a learned map from velocity command and velocity history to current, plus randomised viscous and stick friction that the critic sees.
- **Multi-style skills on the wheeled ANYmal.** [Vollenweider 2022, arXiv 2203.14912](https://arxiv.org/abs/2203.14912). Uses multiple adversarial motion priors, trained in Isaac Gym with 4096 envs. *Hardware: yes.*
- **Whole-body MPC baseline.** [Bjelonic 2021, IROS](https://arxiv.org/abs/2010.06322), [DOI](https://doi.org/10.1109/iros51168.2021.9636371). A kinodynamic MPC that treats wheels as moving contacts. *Hardware: yes, not RL.*
- **LimX Tron1, wheel-foot variant: CTBC.** [Li 2025, arXiv 2509.02986](https://arxiv.org/abs/2509.02986). *Hardware: yes, 20 cm stairs on Tron1 and 7.5 cm on the Cowarobot R0.*
  - **Training:** PPO with an asymmetric critic and an MLP state estimator, in Isaac Gym, cross-checked in MuJoCo, deployed at 50 Hz.
  - **Actions:** 8-dimensional; leg position targets on PD, wheel velocity targets.
  - **Rewards:** include action rate (−0.01) and second-difference action smoothness (−0.005).
  - **Randomisation:**
    - PD gains: Kp and Kd scaled ×0.8–1.2;
    - motor torque, inertia and link mass parameters: ×0.8–1.2;
    - friction 0.2–1.6, restitution 0–1;
    - payload −0.5 to 2 kg, centre-of-mass shift ±3 cm;
    - IMU offset;
    - "Step delay [0, 20] ms".
  - LimX publishes an open Isaac Lab and rsl_rl training stack for the wheel-foot Tron1 at 4096 envs ([tron1-rl-isaaclab](https://github.com/limxdynamics/tron1-rl-isaaclab)). It is a repository, not a paper.
- **Unitree Go2-W.**
  - Long-distance navigation: [Matsuzawa 2026, arXiv 2606.21387](https://arxiv.org/abs/2606.21387). Proprioceptive PPO with an asymmetric actor-critic, leg angle targets plus wheel speed targets, an "action latency" observation, and action-rate and action-curvature penalties. *Hardware: yes, a 2.8 km course.*
  - Fall recovery: [Deng 2025, arXiv 2506.05516](https://arxiv.org/abs/2506.05516). *Hardware: yes, zero-shot on Go2-W.*
- **Unitree B2-W.** I found only a simulation-only community repository ([b2w-rl](https://github.com/lauramquiros/b2w-rl)). *No verified hardware RL paper.*
- **DIABLO.** The IROS 2024 paper is LQR-based, not RL. [Liu 2024](https://arxiv.org/abs/2407.21500). The only RL artefact is a community Isaac Gym repository ([DDTRobot](https://github.com/DDTRobot/Col_DIABLO_Issac_RL)). *No verified hardware RL.*
- **Rhea (custom wheeled biped).** PPO plus a curriculum handled 49 mm ledges, against 21 mm for a linear controller. [Sun, "Sim2Biped" (course report)](https://selenasun.com/papers/cs224r.pdf). *Hardware: yes, but not peer-reviewed.*

## 2. Sim-to-real techniques

### Randomisation and latency

- **Tan 2018.** Improved the simulator with "an accurate actuator model and simulating latency", and randomised the physics. [arXiv 1804.10332](https://arxiv.org/abs/1804.10332). *Hardware: yes, Minitaur.*
- **Peng 2018.** Dynamics randomisation for transfer. [Peng 2018, ICRA](https://arxiv.org/abs/1710.06537). *Hardware: yes, an arm doing object pushing.*
- **Xie 2021 (explicit latency study).** "policies trained without randomization fail when the latency exceeds 17 ms … randomized latency of up to 20 ms … can handle latency up to 32 ms, both in simulation and on the physical robot." They also found that broad dynamics randomisation makes policies conservative, so randomise only what matters. [Xie 2021, ICRA](https://arxiv.org/abs/2011.02404). *Hardware: yes, Laikago.*
- **Isaac Lab's delayed PD actuator.** Buffers "desired actions by a configurable number of simulation steps". [Isaac Lab 2025](https://arxiv.org/abs/2511.04831).
- **MuJoCo Playground.** The Franka task imposes "action and observation stochastic delays". [Zakka 2025](https://arxiv.org/abs/2502.08844). *Hardware: yes.*

### Actuator modelling

- **Hwangbo 2019.** An actuator network trained on real data, used for ANYmal agility and fall recovery. [Hwangbo 2019, Sci. Robot.](https://arxiv.org/abs/1901.08652). *Hardware: yes.*
- **Rudin 2021.** The actuator network limited the simulation step to no smaller than 5 ms. [Rudin 2021](https://arxiv.org/abs/2109.11978).

### Privileged information and state estimation

- **Asymmetric actor-critic.** The critic sees the full state, the actor sees partial observations. [Pinto 2017](https://arxiv.org/abs/1710.06542). *Hardware: yes.*
- **Teacher–student distillation.** [Lee 2020, Sci. Robot.](https://arxiv.org/abs/2010.11251). *Hardware: yes, ANYmal, zero-shot in the wild.*
- **RMA.** A base policy plus an adaptation module, trained entirely in simulation. [Kumar 2021](https://arxiv.org/abs/2107.04034). *Hardware: yes, A1.*
- **DreamWaQ.** An implicit terrain and velocity estimator. [Nahrendra 2023](https://arxiv.org/abs/2301.10602). *Hardware: yes.*
- **Concurrent estimator.** A network estimates base linear velocity, foot height and contact probability alongside the policy. [Ji 2022, RA-L](https://arxiv.org/abs/2202.05481). *Hardware: yes, up to 3.75 m/s.*

### Curriculum

- **Game-inspired terrain curriculum.** [Rudin 2021](https://arxiv.org/abs/2109.11978). *Hardware: yes.* Also used on Ascento.

### Symmetry

- **Augmentation versus mirror loss.** Gives a theoretical basis for adding mirrored samples to on-policy training, with faster convergence. [Mittal 2024](https://arxiv.org/abs/2403.04359). *Simulation focus.* It is implemented in rsl_rl's PPO ([rsl_rl](https://github.com/leggedrobotics/rsl_rl)).
- **Original mirror loss.** [Yu 2018, ACM TOG](https://arxiv.org/abs/1801.08093). *Simulation only.*

### Constraints

- **CaT.** A constraint violation triggers a stochastic termination, with minimal changes to PPO. [Chane-Sane 2024](https://arxiv.org/abs/2403.18765). *Hardware: yes, Solo.*
- **Rewards plus constraints.** Tuning only a single reward coefficient. [Kim 2024, T-RO 40](https://arxiv.org/abs/2308.12517). *Hardware: yes.*
- **P3O.** [Zhang 2022](https://arxiv.org/abs/2205.11814).
- **Constrained-RL comparison on robots.** Includes Klemm and Bjelonic as co-authors. [Lee 2023](https://arxiv.org/abs/2309.15430). *Hardware: yes.*

### Action parametrisation

- **Peng & van de Panne 2017.** Higher-level actions (PD targets) beat torques for learning speed and robustness. [SCA 2017](https://arxiv.org/abs/1611.01055). *Simulation only.*
- **Torque-output RL.** "directly predicts joint torques at a high frequency", and was more robust to disturbances. [Chen 2022](https://arxiv.org/abs/2203.05194). *Hardware: yes.*
- **Learned impedance.** The policy outputs gains and desired position. [Bogdanovic 2020](https://arxiv.org/abs/1907.07500). *Hardware: yes, a real system.*

### Reward design

- **legged_gym reference config.**
  - Tracking reward is "exp(-error^2/sigma)" with σ = 0.25.
  - Weights: lin-vel tracking 1.0, ang-vel 0.5, lin_vel_z −2, ang_vel_xy −0.05, torques −1e-5, dof_acc −2.5e-7, action_rate −0.01.
  - Negative totals are clipped to zero.
  - Source: [legged_gym config](https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/base/legged_robot_config.py).
- **Playground** uses the same exponential kernels plus orientation, pose and termination terms. [Zakka 2025, App. B](https://arxiv.org/abs/2502.08844).

### Smoothness

- **CAPS.** Temporal and spatial smoothness regularisation; about 80 % power reduction on a real quadrotor. [Mysore 2021](https://arxiv.org/abs/2012.06644). *Hardware: yes.*
- **Lipschitz-constrained policies.** A gradient penalty that "replaces the need for smoothing rewards or low-pass filters". [Chen 2024](https://arxiv.org/abs/2410.11825). *Hardware: yes, humanoids.*

## 3. Massively parallel GPU training

- **Rudin 2021 (CoRL 2021, PMLR 2022).** [arXiv 2109.11978](https://arxiv.org/abs/2109.11978). *Hardware: yes, ANYmal.*
  - Flat terrain in under 4 minutes, rough terrain in 20 minutes, on an RTX A6000.
  - The deployed policy used 4096 robots; batch 98304 (4096×24), minibatch 24576, 5 epochs, clip 0.2, entropy 0.01, γ 0.99, λ 0.95, adaptive learning rate with KL target 0.01, 1500 updates.
  - Policy at 50 Hz with 4 simulation steps of 5 ms each.
  - 2048–4096 robots with batches of about 100k–200k samples were the best trade-off.
- **MuJoCo Playground.** [Zakka 2025](https://arxiv.org/abs/2502.08844).
  - Zero-shot transfer on Go1, Berkeley Humanoid, G1, Booster T1, LEAP hand and Franka.
  - Most environments train "in under 10 minutes on a single GPU". Go1 flat terrain trains within 5 minutes on 2× RTX 4090.
  - Brax PPO throughput on an A100: Go1JoystickFlat 417k steps/s, G1Joystick 106k steps/s.
  - Locomotion actions are a PD-tracked position target, q_default + k·a, with an asymmetric critic; policies run at 50 Hz under ros2_control.
  - Stated limitations: JIT compilation takes 1–3 minutes, and contact cost "scale[s] like the number of possible contacts".
  - The README says it now trains with "both the MuJoCo MJX JAX implementation, as well as the MuJoCo Warp implementation" (`--impl warp`, with `train-jax-ppo` or `train-rsl-ppo`). [Playground README](https://github.com/google-deepmind/mujoco_playground)
- **MuJoCo Warp release status.**
  - "MuJoCo Warp is now officially released" in MuJoCo 3.5.0, Feb 12, 2026. The MJX Warp backend is "beta". [changelog](https://mujoco.readthedocs.io/en/3.5.0/_sources/changelog.rst.txt)
  - Current release is v3.16.0, Oct 14, 2026. [releases](https://github.com/google-deepmind/mujoco_warp/releases)
  - MJX docs report 3.35M steps/s on Humanoid with pure Warp and 2.96M through the JAX bridge; the GPU used is not stated. [MJX docs](https://mujoco.readthedocs.io/en/latest/mjx.html)
- **mjlab.** Isaac Lab's manager-based API running on MuJoCo Warp, PyTorch and rsl_rl. [Zakka 2026](https://arxiv.org/abs/2601.22074).
- **Brax.** [Freeman 2021](https://arxiv.org/abs/2106.13281). Its old generalized physics pipeline is "no longer maintained"; Brax now depends on MJX.
- **rsl_rl.** [Schwarke 2025](https://arxiv.org/abs/2509.10771). PPO, student–teacher distillation, symmetry, RND, multi-GPU. Used by Isaac Lab, legged_gym, mjlab and Playground.
- **Isaac Lab.**
  - Orbit: [Mittal 2023, RA-L](https://arxiv.org/abs/2301.04195).
  - Isaac Lab: [Mittal 2025](https://arxiv.org/abs/2511.04831). PhysX gained "closed-loop kinematic chains", though loops "require a higher solver iteration count" (Digit). It has delayed-PD actuators and multi-rate sensors.
  - Isaac Gym: [Makoviychuk 2021](https://arxiv.org/abs/2108.10470).

## 4. Verified facts about the simulators

**MJX-JAX** ([feature parity table](https://mujoco.readthedocs.io/en/latest/mjx.html#feature-parity)):
- **Condim:** "1, 3, 4, 6 (1 is not supported with ELLIPTIC)". **condim 4 is supported.**
- **Equality:** "CONNECT, WELD, JOINT, TENDON". **Connect is supported.**
- **Constraint:** "EQUALITY, LIMIT_JOINT, CONTACT_FRICTIONLESS, CONTACT_PYRAMIDAL, CONTACT_ELLIPTIC, FRICTION_DOF, FRICTION_TENDON". Frictionloss (FRICTION_DOF) is supported.
- **Armature:** not listed in the docs. The source has `dof_armature: jax.Array` in `mjx.Model` ([types.py](https://github.com/google-deepmind/mujoco/blob/main/mjx/mujoco/mjx/_src/types.py)). **Supported, and randomisable at runtime.**
- **Geoms:** "PLANE, HFIELD, SPHERE, CAPSULE, BOX, MESH are fully implemented. ELLIPSOID and CYLINDER are implemented but only collide with other primitives".
- **Unsupported pairs:** "Collisions between (SPHERE, BOX, MESH, HFIELD) and CYLINDER."
- **Cylinder–plane:** the source maps `(PLANE, CYLINDER): plane_cylinder` and `(PLANE, MESH): plane_convex` ([collision_driver.py](https://github.com/google-deepmind/mujoco/blob/main/mjx/mujoco/mjx/_src/collision_driver.py)). **Cylinder–plane and mesh–plane are supported.**
  - The source at HEAD also maps sphere–cylinder, which contradicts the docs footnote; `put_model` raises on unsupported pairs, so test it.
  - **Cylinder tyre on a heightfield is not supported.**
- **Solver:** CG or Newton only (no PGS, no noslip).
- **Mesh cost:** convex meshes should have "roughly 200 vertices or less" against primitives, and fewer than 32 for mesh–mesh.

**MuJoCo Warp** ([README](https://github.com/google-deepmind/mujoco_warp), [docs](https://mujoco.readthedocs.io/en/latest/mjwarp/index.html)):
- "MuJoCo Warp supports the same features as MuJoCo with the following exceptions":
  - the IMPLICITFAST midpoint integrator;
  - "PGS not yet supported";
  - "Actuator / Sensors: PLUGIN types not yet supported";
  - Flex is experimental.
- MJX parity table, MJX-Warp column: Equality "All", Condim "All", Geom "All".
- Collider table: "Plane | Cylinder: primitive 4", "Plane | Mesh: primitive 4", "HField | Cylinder: HFieldCCD 4".
- "MJX-Warp also fully supports mesh collisions."
- **Sizing:** you must set `naconmax`, `njmax` and the Warp graph mode.
- **No differentiability.**
- **Margin limitation:** non-zero margin with box/mesh multi-CCD raises an error.
- **Sandro-specific:** the plugin exception applies only to MJCF engine plugins. Going by the repo's own descriptions, `MitDrivePlugin` is a mujoco_ros2_control (host-side) plugin, not one of these. Either way, the MIT drive law has to be re-implemented in the training environment and run every physics substep.

## 5. Recommendation for Sandro

**Adopt:**
1. **Action space: position targets plus wheel velocity targets with fixed or randomised kp and kd, sent as MIT-mode frames.** This is what Ascento, CTBC, Lee 2024 and Go2-W did, and it keeps the stiff loop inside the drive, which bypasses the bus and IMU latency. Optionally add learned gains on the legs following Bogdanovic 2020.
2. **Act on the 4 leg motors and 2 wheels only.** Keep the passive five-bar joints as connect constraints, which both backends support.
3. **Asymmetric critic plus an estimated-velocity head.** The critic gets base velocity, height, contact forces, friction and the delay values. The actor gets IMU orientation and gyro, joint positions and velocities, and the last action. Add a concurrent estimator (Ji 2022) for base velocity, because there is no velocity sensor. This is simpler than teacher–student, and Ascento shows it is enough for a wheeled biped.
4. **Per-channel latency and rate randomisation.**
   - IMU delay 0–40 ms, which covers the measured 1–35 ms.
   - Sample-and-hold at a randomised 30–200 Hz.
   - Encoder and command delays at a few ms.
   - Optionally include the delay in critic observations.
   - Evidence: Xie 2021 (randomising to 20 ms gave tolerance to 32 ms), Ascento (about 20 ms), CTBC (0–20 ms).
5. **Narrow, identified randomisation, with a small sweep of the remaining parameters.** Randomise friction, frictionloss, armature, Kp/Kd ×0.8–1.2, payload and centre of mass, and IMU bias. Keep tyre friction moderate (Ascento's grip exploit), and avoid over-broad randomisation (Xie 2021).
6. **Rewards and smoothness.** legged_gym-style exponential tracking terms with an orientation term; action-rate and action-curvature penalties, or Lipschitz-constrained policies (LCP); left-right mirror augmentation; CaT for joint, torque and five-bar-singularity limits.
7. **Simulator.** Use MuJoCo Warp (through Playground `--impl warp` or mjlab) as soon as anything goes beyond a flat plane. MJX-JAX is fine for flat-ground cylinder–plane balance work.

**Reject or defer:**
- Torque-output policies (Chen 2022): they need a high-rate loop through the latency path and throw away the drive-side PD.
- Actuator networks: there is no torque sensor. Use identified friction and armature first.
- RMA-style adaptation and terrain curricula: defer until you move to rough terrain.
- Brax's legacy physics pipeline: unmaintained.

## 6. Leads I could not verify

- **No 2025 or 2026 Ascento RL or jumping paper** turned up in any search. The only Ascento RL paper is Chamorro 2024.
- **No paper titled "Learning agile locomotion on wheeled bipeds"** was found.
- **No wheeled-biped RL jumping with hardware results was found.** The CAC 2025 "stage-wise reward shaping" jumping paper ([DOI](https://doi.org/10.1109/cac67268.2025.11486681)) exists, but I couldn't confirm whether it reached hardware. RL jumping on hardware exists for Cassie, a non-wheeled biped ([Li 2023](https://arxiv.org/abs/2302.09450)).
- **No peer-reviewed RL papers for DIABLO, Unitree B2-W or LimX W1.** The CCC 2023 wheeled-biped balance RL paper is cited but I didn't check it.
- **Venues not confirmed** (arXiv IDs only): Mittal 2024, CaT, Chen 2022, CAPS and LCP.
