"""Whole-body drive controller for the composed wheeled biped.

Uses every actuator the robot has. Architecture (each element was validated
against the numerically linearized full model -- mjd_transitionFD with the
exact discrete control law closed around it -- and in simulation):

  wheels    Ground SPEED is controlled THROUGH the lean, never directly:
            velocity-mode wheels cannot host two speed authorities (a
            direct drive loop and the balance loop deadlock, each blocking
            the other). The wheels always follow the measured ground speed
            plus a balance correction that chases the contact point under
            the COM; velocity/position error is turned into a demanded
            acceleration and hence a lean reference (lean = a/g). Balance
            gains scale with sqrt(g*H_com), so they re-schedule
            automatically with ride height. Yaw rate is differential wheel
            speed: kinematic feedforward from the track width + gyro
            feedback.
  leg motors  pose control through per-leg IK on the four-bar linkage:
            ride height, roll (differential leg height, and turns bank
            automatically like a bicycle), torso pitch attitude (fore-aft
            wheel shift), and acceleration assist -- the legs pre-shift the
            wheels backward by du*tan(lean_ref), creating the acceleration
            lean geometrically so the wheels never have to move backward to
            initiate forward motion (the classic non-minimum-phase
            conflict). All leg targets derive from COMMANDS + stand-pose
            constants; feeding measured quantities (COM position, attitude)
            back into the legs at bandwidth is unstable, so the only
            feedback there is a slow integral attitude trim.

  shaping   commands are shaped for an organic, sporty feel: a saturated
            approach law with lean-unwind anticipation (brake early by the
            speed still to be gained while the lean unwinds), acceleration
            authority capped to what the leg workspace can create at the
            current ride height (a = g*x/du), and a reference governor so
            the targets never outrun the robot. Measured 0->1.35 m/s in
            0.9 s, full 1.5 -> -1.5 reversal in 0.9 s, yaw step to 90% in
            0.14 s, and it survives 40 s of random aggressive commands.

The controller reads only what the real robot will have (IMU quat/gyro +
motor encoders) and writes only what the CAN bus carries (leg MIT-mode angle
targets, wheel speed targets). The kinematic/dynamic model it consults
internally is the same MJCF the firmware would carry as a URDF. The
estimator compensates the odometry for LEG MOTION (the encoders measure
wheel-vs-carrier, and the legs both rotate the carrier and translate the
wheels under the body) -- without this, fast leg action corrupts the
velocity estimate and the whole controller destabilizes.

Run:
    python drive_controller.py                 # viewer + keyboard teleop
    python drive_controller.py --test all      # headless checks (see --help)

Teleop keys (viewer window focused):
    W/S  forward speed +/-      A/D  yaw rate +/-       X    all-stop
    R/F  ride height +/-        T/G  torso pitch +/-    Q/E  roll offset +/-
"""
from __future__ import annotations

import argparse
import time

import mujoco
import numpy as np

from controller_demo import Robot, CONTROL_HZ, SIDES
from stand_pose import (leg_solve, jid, LEG_JOINTS_ALL, PASSIVE_JOINTS, HERE,
                        collidable_min_z)

ROBOT_XML = HERE / "robot" / "robot_full.xml"           # fixed base: internal model
FLOATING_XML = HERE / "robot" / "robot_full_floating.xml"

G = 9.81

# command shaping: the acceleration intent follows a saturated approach law
#   a_int = clip(K_ARR_V * (v_tgt - v_pred), +-caps)
# where v_pred = v_ref + g*lean_ff*ARR_LEAD anticipates the speed still to
# be gained while the lean unwinds (the measured lean lags the feedforward
# by roughly ARR_LEAD through the leg PDs and the balance dynamics).
# Anticipating through the controller's own smooth lean_ff state -- never
# through the measured speed -- keeps sensor noise out of the leg targets.
ACCEL_MAX = 3.5        # m/s^2 forward (further capped by the leg workspace)
K_ARR_V = 6.0          # 1/s, approach-law pole
ARR_LEAD = 0.4         # s, arrival anticipation horizon
YAW_ACCEL_MAX = 12.0   # rad/s^2
HEIGHT_RATE = 0.12     # m/s
PITCH_RATE = 1.0       # rad/s
ROLL_RATE = 1.5        # rad/s
LAT_ACC_MAX = 4.5      # m/s^2: yaw rate is capped so v*psidot stays below this

# wheel cascade gains (see the module docstring for the architecture)
BAL_KP_SCALE = 2.5     # (m/s per rad) per sqrt(g * COM height)
BAL_KD_FRACT = 0.15    # s: lean-rate gain as a fraction of the lean gain
KP_POS = 1.0           # m/s^2 of demanded acceleration per m of position error
KD_POS = 0.6           # s^-1: velocity-error weight in the demanded accel
                       # (multiplied by g inside: 0.6 -> ~6 m/s^2 per m/s)
GOV_BAND = 0.5         # m/s: reference governor band (v_ref stays this close
                       # to the measured speed)
LEAN_FF_TAU = 0.10     # s, low-pass on the acceleration lean feedforward
FRIC_FF = 0.0          # rolling-friction lean feedforward: OFF -- it fixes
                       # the cruise droop but biases the lean the wrong way
                       # during braking (measured: doubles the overshoot);
                       # the position integral below covers the droop instead
SREF_WINDUP = 1.0      # m, position-reference anti-windup around measured s
                       # (large enough that the position term can hold the
                       # lean needed against rolling friction at cruise)
VEL_FILT_HZ = 25.0     # low-pass on velocity estimates (contact impulses can
                       # spike a single qvel sample; real drives filter too)
WHEEL_ACC_MAX = 400.0  # rad/s^2 slew limit on wheel speed commands

YAW_KP = 0.4           # rad/s error -> extra differential wheel speed (rad/s)
ATT_KI = 0.03          # m/s fore-aft trim per rad of pitch-attitude error (slow!)
ATT_TRIM_MAX = 0.03    # m
X_FORE_RATE = 0.8      # m/s, fore-aft wheel shift slew
LEG_TRIM_KI = 3.0      # 1/s, integral trim on leg PD targets (gravity sag)
LEG_TRIM_MAX = 0.25    # rad
SETTLE_TIME = 0.6      # s before balance engages (gain ramp)
FALL_LEAN = 0.7        # rad, beyond this: cut torques


class LegModel:
    """Internal kinematic/dynamic model: IK for the four-bar legs and
    pendulum parameters at the current pose. Two MjData: one tracking the
    measured state (estimation), one tracking commanded targets (IK)."""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
        self.d_meas = mujoco.MjData(self.model)
        self.d_ik = mujoco.MjData(self.model)

        # torso-frame reference directions from the floating model's stand
        # keyframe: its base quat maps CAD/torso axes onto (fwd, axle, up)
        mf = mujoco.MjModel.from_xml_path(str(FLOATING_XML))
        kid = mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_KEY, "stand")
        qk = mf.key_qpos[kid]
        R0 = np.empty(9)
        mujoco.mju_quat2Mat(R0, qk[3:7])
        R0 = R0.reshape(3, 3)
        self.fwd_t = R0.T @ [1.0, 0, 0]   # forward, in torso frame
        self.axle_t = R0.T @ [0, 1.0, 0]  # wheel axle (+left), in torso frame
        self.up_t = R0.T @ [0, 0, 1.0]    # up at spawn, in torso frame

        # seed both internal datas with the stand pose (per-joint by name)
        for j in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            jf = mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_JOINT, name)
            for d in (self.d_meas, self.d_ik):
                d.qpos[self.model.jnt_qposadr[j]] = qk[mf.jnt_qposadr[jf]]
        mujoco.mj_kinematics(self.model, self.d_ik)

        self.wj = {s: jid(self.model, f"{s}_wheel") for s in SIDES}
        self.wbody = {s: int(self.model.jnt_bodyid[self.wj[s]]) for s in SIDES}
        n = self.d_ik.xaxis[self.wj[SIDES[0]]].copy()
        n /= np.linalg.norm(n)
        if n @ self.axle_t < 0:   # sign convention: axle points left (+y at spawn)
            n = -n
        self.n_t = n              # common hinge axis (planar mechanism)
        self.P = np.eye(3) - np.outer(self.n_t, self.n_t)
        self.pivot = {s: np.mean([self.d_ik.xanchor[jid(self.model, f"{s}_{m}")]
                                  for m in ("motor_inner", "motor_outer")], axis=0)
                      for s in SIDES}

        # wheel radius from the tire meshes, track width from the anchors
        df = mujoco.MjData(mf)
        df.qpos[:] = qk
        mujoco.mj_forward(mf, df)
        anchors = {s: df.xanchor[mujoco.mj_name2id(mf, mujoco.mjtObj.mjOBJ_JOINT,
                                                   f"{s}_wheel")].copy() for s in SIDES}
        minz = collidable_min_z(mf, df)
        self.wheel_radius = float(np.mean([a[2] for a in anchors.values()]) - minz)
        # at the stand keyframe the axle is world y, so track = lateral spacing
        self.track = float(abs((anchors["left"] - anchors["right"]) @ [0, 1.0, 0]))
        self.m_total = float(self.model.body_mass[1:].sum())

        # stand-pose height / fore-aft, used as command defaults
        self.h0, self.x0 = {}, {}
        for s in SIDES:
            v = self.pivot[s] - self.d_ik.xanchor[self.wj[s]]
            self.h0[s] = float(v @ self.up_t)
            self.x0[s] = float(-v @ self.fwd_t)
        self.height_range = self._probe_height_range()
        self._workspace = self._probe_workspace()
        # floor the commandable height where the fore-aft workspace stays
        # usable: near the fully folded pose the travel collapses to almost
        # nothing, lean authority vanishes, and the balance law chases a
        # lean the legs cannot create (velocity runs away, then falls)
        hs, los, his = self._workspace
        usable = np.minimum(-los, his) >= 0.04
        if not usable.all():
            k = int(np.argmax(usable))          # first usable sample
            lo_ok = hs[k] if k == 0 else float(np.interp(
                0.04, [min(-los[k-1], his[k-1]), min(-los[k], his[k])],
                [hs[k-1], hs[k]]))
            self.height_range = (max(self.height_range[0], lo_ok + 0.005),
                                 self.height_range[1])

    # ------------------------------------------------------------------- IK
    def ik(self, side, x_fore, height):
        """Motor angles placing the wheel center at (x_fore forward, height
        below the pivot midpoint) in the torso frame. Warm-started; returns
        (inner, outer, residual)."""
        target = self.pivot[side] + x_fore * self.fwd_t - height * self.up_t
        res = leg_solve(self.model, self.d_ik, side, LEG_JOINTS_ALL,
                        target=target, n_axis=self.n_t, max_iter=20)
        q = self.d_ik.qpos
        return (float(q[self.model.jnt_qposadr[jid(self.model, f"{side}_motor_inner")]]),
                float(q[self.model.jnt_qposadr[jid(self.model, f"{side}_motor_outer")]]),
                res)

    def _probe_height_range(self, margin=0.015):
        """Reachable ride height at x_fore = x0, probed with the IK data and
        restored afterwards."""
        saved = self.d_ik.qpos.copy()
        lo = hi = np.mean(list(self.h0.values()))
        for direction in (+1, -1):
            h = np.mean(list(self.h0.values()))
            while True:
                h2 = h + direction * 0.005
                ok = all(self.ik(s, self.x0[s], h2)[2] < 1e-8 for s in SIDES)
                if not ok or h2 < 0.03 or h2 > 0.5:
                    break
                h = h2
            if direction > 0:
                hi = h
            else:
                lo = h
            self.d_ik.qpos[:] = saved
            mujoco.mj_kinematics(self.model, self.d_ik)
        self.d_ik.qpos[:] = saved
        return (lo + margin, hi - margin)

    def _probe_workspace(self, n=7, margin=0.006):
        """Fore-aft wheel travel [x_lo, x_hi] as a function of ride height.
        The workspace shrinks dramatically near full leg extension, and it
        caps how much lean -- hence acceleration -- the legs can create."""
        saved = self.d_ik.qpos.copy()
        x0 = float(np.mean(list(self.x0.values())))
        hs = np.linspace(self.height_range[0], self.height_range[1], n)
        los, his = [], []
        for h in hs:
            for s in SIDES:            # recenter the warm start at this height
                self.ik(s, x0, h)
            lims = []
            for direction in (+1, -1):
                x = x0
                while True:
                    x2 = x + direction * 0.005
                    ok = all(self.ik(s, x2, h)[2] < 1e-8 for s in SIDES)
                    if not ok or abs(x2 - x0) > 0.25:
                        break
                    x = x2
                lims.append(x)
                for s in SIDES:
                    self.ik(s, x0, h)
            his.append(lims[0] - margin)
            los.append(lims[1] + margin)
        self.d_ik.qpos[:] = saved
        mujoco.mj_kinematics(self.model, self.d_ik)
        return hs, np.array(los), np.array(his)

    def x_range(self, h):
        """Interpolated fore-aft travel bounds at ride height h."""
        hs, los, his = self._workspace
        return (float(np.interp(h, hs, los)), float(np.interp(h, hs, his)))

    # ------------------------------------------------------- estimation side
    def set_measured(self, leg_angles):
        """Update the measurement-side data from encoder readings and re-close
        the passive loops (warm-started, converges in 1-2 Newton steps)."""
        for name, val in leg_angles.items():
            self.d_meas.qpos[self.model.jnt_qposadr[jid(self.model, name)]] = val
        for s in SIDES:
            leg_solve(self.model, self.d_meas, s, PASSIVE_JOINTS, max_iter=10)
        mujoco.mj_kinematics(self.model, self.d_meas)

    def geometry(self):
        """(axle midpoint, COM of the pendulum part, wheel centers) in the
        torso frame at the measured pose, plus pendulum parameters."""
        m = self.model
        d = self.d_meas
        centers = {s: d.xanchor[self.wj[s]].copy() for s in SIDES}
        axle_mid = np.mean(list(centers.values()), axis=0)
        wheel_bodies = set(self.wbody.values())
        m_b = 0.0
        c_b = np.zeros(3)
        for b in range(1, m.nbody):
            if b in wheel_bodies:
                continue
            m_b += m.body_mass[b]
            c_b += m.body_mass[b] * d.xipos[b]
        c_b /= m_b
        I_b = 0.0
        for b in range(1, m.nbody):
            if b in wheel_bodies:
                continue
            Rb = d.ximat[b].reshape(3, 3)
            Ib = Rb @ np.diag(m.body_inertia[b]) @ Rb.T
            r = self.P @ (d.xipos[b] - c_b)
            I_b += float(self.n_t @ Ib @ self.n_t) + m.body_mass[b] * float(r @ r)
        lever = self.P @ (c_b - axle_mid)
        return dict(axle_mid=axle_mid, com=c_b, centers=centers,
                    m_b=m_b, l=float(np.linalg.norm(lever)), I_b=I_b,
                    lever=lever)



class Estimator:
    """Turns IMU + encoders into balance-relevant quantities."""

    def __init__(self, robot: Robot, legs: LegModel):
        self.robot = robot
        self.legs = legs
        self._v_filt = None
        self._lr_filt = None
        self._thc_prev = None   # carrier angle about the axle, vs torso
        self._xw_prev = None    # wheel-center fore-position in torso frame
        self._lg_prev = None    # geometric (leg-induced) part of the lean
        self._thc_rate = 0.0
        self._xw_rate = 0.0
        self._lg_rate = 0.0
        self._xw0 = None
        # wheel forward sign: positive spin about the joint axis moves the
        # robot along axis x z-hat; align that with the torso forward
        d = robot.data
        a_r = d.xaxis[jid(robot.model, f"{SIDES[0]}_wheel")].copy()
        Rq = np.empty(9)
        mujoco.mju_quat2Mat(Rq, robot.sensor("imu_quat"))
        f_w = Rq.reshape(3, 3) @ legs.fwd_t
        self.fwd_sign = 1.0 if (np.cross(a_r, [0, 0, 1.0]) @ f_w) > 0 else -1.0
        self.update()

    def R(self):
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, self.robot.sensor("imu_quat"))
        return R.reshape(3, 3)

    def update(self):
        legs = self.legs
        self.legs.set_measured(self.robot.leg_angles())
        geo = legs.geometry()
        R = self.R()
        n_w = R @ legs.n_t
        u_w = R @ legs.up_t
        f_w = R @ legs.fwd_t
        z = np.array([0.0, 0, 1.0])
        fh = f_w - z * (f_w @ z)
        fh /= max(np.linalg.norm(fh), 1e-9)
        # attitude: pitch + = tipped forward, roll + = leaning left (+y down)
        self.pitch = float(np.arctan2(u_w @ fh, u_w @ z))
        self.roll = float(-np.arcsin(np.clip(n_w @ z, -1, 1)))
        gyro = self.robot.sensor("imu_gyro")
        self.pitch_rate = float(gyro @ legs.n_t)
        self.yaw_rate = float(gyro @ (R.T @ z))
        # balance lean: COM relative to the contact point, about the axle
        v_w = R @ (geo["com"] - geo["axle_mid"])           # axle mid -> COM
        d = -(z - n_w * (n_w @ z))
        d /= max(np.linalg.norm(d), 1e-9)                  # axle mid -> contact
        p = v_w - legs.wheel_radius * d                    # contact -> COM
        self.lean = float(np.arctan2(p @ fh, p @ z))
        self.pend = geo
        dt = self.robot.decimation * self.robot.model.opt.timestep
        a = dt / (dt + 1.0 / (2 * np.pi * VEL_FILT_HZ))
        # leg-motion terms: the encoders measure wheel-vs-CARRIER, and the
        # legs both rotate the carrier and translate the wheels under the
        # body. Without these terms fast leg action (stiff leg PDs, sporty
        # fore-aft shifts) whipsaws the odometry. Carrier angle about the
        # axle and wheel fore-position come from the measured-pose internal
        # model; their rates are filtered finite differences.
        th_c = 0.0
        for s_ in SIDES:
            carrier = int(legs.model.body_parentid[legs.wbody[s_]])
            Rc = legs.d_meas.xmat[carrier].reshape(3, 3)
            af = Rc @ legs.fwd_t
            th_c += 0.5 * float(np.arctan2(-(af @ legs.up_t), af @ legs.fwd_t))
        x_w = float(np.mean([geo["centers"][s_] @ legs.fwd_t for s_ in SIDES]))
        lg = self.lean - self.pitch     # leg-induced (geometric) lean part
        if self._thc_prev is not None:
            self._thc_rate += a * ((th_c - self._thc_prev) / dt - self._thc_rate)
            self._xw_rate += a * ((x_w - self._xw_prev) / dt - self._xw_rate)
            self._lg_rate += a * ((lg - self._lg_prev) / dt - self._lg_rate)
        if self._xw0 is None:
            self._xw0 = x_w
        self._thc_prev, self._xw_prev, self._lg_prev = th_c, x_w, lg
        # odometry: wheel-vs-carrier encoders + body attitude + leg motion
        r = legs.wheel_radius
        raw_pos = [self.robot.wheel_dir[s] * self.robot.sensor(f"{s}_wheel_pos")[0]
                   for s in SIDES]
        raw_vel = [self.robot.wheel_dir[s] * self.robot.sensor(f"{s}_wheel_vel")[0]
                   for s in SIDES]
        self.s = r * (self.fwd_sign * float(np.mean(raw_pos)) + self.pitch + th_c) \
            - (x_w - self._xw0)
        v_raw = r * (self.fwd_sign * float(np.mean(raw_vel)) + self.pitch_rate
                     + self._thc_rate) - self._xw_rate
        # low-pass velocity and lean rate: a contact impulse can spike one
        # qvel/gyro sample, and unfiltered spikes go straight into the wheels
        self._v_filt = v_raw if self._v_filt is None else \
            self._v_filt + a * (v_raw - self._v_filt)
        lr_raw = self.pitch_rate + self._lg_rate
        self._lr_filt = lr_raw if self._lr_filt is None else \
            self._lr_filt + a * (lr_raw - self._lr_filt)
        self.v = self._v_filt
        self.lean_rate = self._lr_filt
        self.wheel_speeds = self.fwd_sign * np.array(raw_vel)  # forward convention


class Commands:
    """User-facing command state with slew limiting."""

    def __init__(self, legs: LegModel):
        self.v_ref = self.v_tgt = 0.0
        self.yaw_ref = self.yaw_tgt = 0.0
        h0 = float(np.mean(list(legs.h0.values())))
        self.h_ref = self.h_tgt = h0
        self.pitch_ref = self.pitch_tgt = 0.0
        self.roll_off = self.roll_tgt = 0.0
        self.h_range = legs.height_range
        self.accel = 0.0

    def slew(self, dt, a_ref, v_meas=None):
        """Advance the references. a_ref is the S-curve acceleration
        reference computed by the controller; it doubles as the lean
        feedforward source, so it stays alive while the governor pauses the
        reference ramp."""
        self.accel = a_ref
        rate = max(abs(a_ref), 0.3)   # floor: v_ref must still converge
        dv = np.clip(self.v_tgt - self.v_ref, -rate * dt, rate * dt)
        # reference governor: never let the speed reference outrun the robot
        # by more than the outer loop's authority
        if v_meas is not None:
            lo, hi = v_meas - GOV_BAND, v_meas + GOV_BAND
            dv = np.clip(dv, min(0.0, lo - self.v_ref), max(0.0, hi - self.v_ref))
        self.v_ref += dv
        self.yaw_ref += np.clip(self.yaw_tgt - self.yaw_ref,
                                -YAW_ACCEL_MAX * dt, YAW_ACCEL_MAX * dt)
        self.h_tgt = float(np.clip(self.h_tgt, *self.h_range))
        self.h_ref += np.clip(self.h_tgt - self.h_ref, -HEIGHT_RATE * dt, HEIGHT_RATE * dt)
        self.pitch_ref += np.clip(self.pitch_tgt - self.pitch_ref,
                                  -PITCH_RATE * dt, PITCH_RATE * dt)
        self.roll_off += np.clip(self.roll_tgt - self.roll_off,
                                 -ROLL_RATE * dt, ROLL_RATE * dt)
        # cap lateral acceleration in turns
        if abs(self.v_ref) > 0.1 and abs(self.v_ref * self.yaw_ref) > LAT_ACC_MAX:
            self.yaw_ref = np.sign(self.yaw_ref) * LAT_ACC_MAX / abs(self.v_ref)


class WholeBodyController:
    def __init__(self, robot: Robot):
        self.robot = robot
        # the ACTUAL control period: Robot rounds the decimation, so it can
        # differ from 1/CONTROL_HZ -- every integrator here must use this
        self.dt = robot.decimation * robot.model.opt.timestep
        self.legs = LegModel()
        self.est = Estimator(robot, self.legs)
        self.cmd = Commands(self.legs)
        self.w_max = float(robot.model.actuator_ctrlrange[
            robot._act["right_wheel"], 1])
        # rolling-resistance feedforward constants (identifiable on hardware)
        wdof = robot.model.jnt_dofadr[jid(robot.model, "right_wheel")]
        r0 = self.legs.wheel_radius
        self.a_fric = 2 * float(robot.model.dof_frictionloss[wdof]) \
            / (r0 * self.legs.m_total)
        self.a_visc = 2 * float(robot.model.dof_damping[wdof]) \
            / (r0 * r0 * self.legs.m_total)
        self.bal_kp = self.bal_kd = None
        self.l_design = None
        self.s_ref = None
        self.w_last = (0.0, 0.0)
        self.lean_ff = 0.0      # acceleration lean feedforward
        self.x0 = float(np.mean(list(self.legs.x0.values())))
        self.x_fore = self.x0
        self.x_trim = 0.0
        self.pivot_mid = np.mean([self.legs.pivot[s] for s in SIDES], axis=0)
        # command-derived fore-aft geometry, captured once at the stand pose:
        # COM fore-offset from the pivots and COM height above the pivots.
        # These must NOT track the measured COM -- wheels following the
        # measured COM while the balance chases the contact under the COM is
        # positive feedback (verified unstable).
        self.est.update()
        geo0 = self.est.pend
        self.f_c0 = float((geo0["com"] - self.pivot_mid) @ self.legs.fwd_t)
        w_mid0 = np.mean([geo0["centers"][s] for s in SIDES], axis=0)
        h0 = float(np.mean(list(self.legs.h0.values())))
        self.c0 = float((geo0["com"] - w_mid0) @ self.legs.up_t) - h0
        self.trim = {f"{s}_{m}": 0.0 for s in SIDES
                     for m in ("motor_inner", "motor_outer")}
        self.fallen = False
        self.t = 0.0

    # ---------------------------------------------------------------- gains
    def refresh_gains(self):
        """Schedule the balance gains with the current COM height: the speed
        a contact-chasing balancer needs scales with the pendulum's natural
        speed sqrt(g*H)."""
        p = self.est.pend
        if self.l_design is None or abs(p["l"] - self.l_design) > 0.02 * self.l_design:
            H = p["l"] + self.legs.wheel_radius
            self.bal_kp = BAL_KP_SCALE * np.sqrt(G * H)
            self.bal_kd = BAL_KD_FRACT * self.bal_kp
            self.l_design = p["l"]

    # ----------------------------------------------------------------- tick
    def tick(self):
        dt = self.dt
        est, cmd, legs = self.est, self.cmd, self.legs
        est.update()
        # acceleration authority = what the LEG WORKSPACE can create at this
        # ride height (a = g*x/du is exact for the leg-shift lean mechanism);
        # asking for more saturates the legs and the real lean overshoots
        du = cmd.h_ref + self.c0
        xlo, xhi = legs.x_range(cmd.h_ref)
        x_neutral = self.f_c0 + du * np.tan(cmd.pitch_ref) + self.x_trim
        a_cap_pos = min(ACCEL_MAX, max(0.2, G * (x_neutral - xlo) / du))
        a_cap_neg = min(ACCEL_MAX, max(0.2, G * (xhi - x_neutral) / du))
        # acceleration intent: approach law with lean-unwind anticipation
        v_pred = cmd.v_ref + G * self.lean_ff * ARR_LEAD
        a_int = float(np.clip(K_ARR_V * (cmd.v_tgt - v_pred),
                              -a_cap_neg, a_cap_pos))
        cmd.slew(dt, a_int, est.v)
        self.refresh_gains()
        self.t += dt

        if self.s_ref is None:
            self.s_ref = est.s
        if abs(est.lean) > FALL_LEAN:
            self.fallen = True
        ramp = float(np.clip((self.t - SETTLE_TIME) / 0.5, 0.0, 1.0))
        if self.fallen:
            ramp = 0.0

        # ---- wheels: speed is controlled THROUGH the lean ---------------
        # Velocity-mode wheels cannot host two speed authorities: a direct
        # drive loop and the balance loop deadlock (each blocks the other --
        # verified at length in sim). Instead the wheels always follow the
        # MEASURED ground speed plus the balance correction only, and the
        # outer loop turns velocity/position error into a lean reference:
        # lean_ref = a_demand/g. The legs pre-create that lean geometrically
        # (see the fore-aft block below), so the wheels never need to move
        # backward to initiate acceleration.
        self.s_ref += cmd.v_ref * dt
        self.s_ref = float(np.clip(self.s_ref, est.s - SREF_WINDUP,
                                   est.s + SREF_WINDUP))
        r = legs.wheel_radius
        # rolling-resistance feedforward: without it the robot needs a
        # persistent velocity error (droop) just to hold a steady lean
        a_roll = FRIC_FF * (self.a_fric * np.tanh(cmd.v_ref / 0.15)
                            + self.a_visc * cmd.v_ref)
        a_dem = float(np.clip(cmd.accel + a_roll
                              + KP_POS * (self.s_ref - est.s)
                              + KD_POS * G * (cmd.v_ref - est.v),
                              -a_cap_neg, a_cap_pos))
        # the low-pass keeps sensor noise in the feedback terms away from
        # the legs and matches roughly what the legs can track
        self.lean_ff += (dt / LEAN_FF_TAU) * (a_dem / G - self.lean_ff)
        lean_ff = self.lean_ff
        lean_err = est.lean - lean_ff
        v_bal = self.bal_kp * lean_err + self.bal_kd * est.lean_rate
        # differential: kinematic feedforward + gyro feedback (left turn = +yaw
        # = right wheel on the outside, so the right wheel runs faster)
        dw = (cmd.yaw_ref * legs.track / (2 * r)
              + YAW_KP * (cmd.yaw_ref - est.yaw_rate) / r) * ramp
        w_base = ramp * (est.v + v_bal) / r
        # slew-limit the commands: a single spiky sample must not become a
        # full-power wheel kick
        w_r = float(np.clip(w_base + dw, self.w_last[0] - WHEEL_ACC_MAX * dt,
                            self.w_last[0] + WHEEL_ACC_MAX * dt))
        w_l = float(np.clip(w_base - dw, self.w_last[1] - WHEEL_ACC_MAX * dt,
                            self.w_last[1] + WHEEL_ACC_MAX * dt))
        w_r = float(np.clip(w_r, -self.w_max, self.w_max))
        w_l = float(np.clip(w_l, -self.w_max, self.w_max))
        self.w_last = (w_r, w_l)
        # set_wheel_speed's convention matches the raw right wheel; convert
        self.robot.set_wheel_speed(est.fwd_sign * w_r, est.fwd_sign * w_l)

        # ---- legs: height, roll (banked turns), pitch attitude ----------
        bank = np.arctan(cmd.v_ref * cmd.yaw_ref / G) + cmd.roll_off
        dh = legs.track * np.tan(bank) if abs(bank) < 0.5 else 0.0
        # fore-aft placement from COMMANDS only (stand-pose constants +
        # commanded height/pitch/acceleration); feedback is just a slow
        # integral trim -- anything faster through the balance dynamics is
        # unstable, and tracking the measured COM is positive feedback.
        # The -tan(lean_ff) term makes the LEGS create the acceleration
        # lean: shifting the wheels back geometrically leans the body with
        # NO wheel motion, resolving the non-minimum-phase conflict between
        # the drive loop (wheels forward) and lean creation (wheels back).
        # freeze the attitude trim while accelerating: during acceleration
        # the measured pitch legitimately differs from pitch_ref, and the
        # integrator would eat the acceleration leg-shift
        if abs(lean_ff) < 0.02:
            self.x_trim = float(np.clip(
                self.x_trim + ATT_KI * (cmd.pitch_ref - est.pitch) * dt * ramp,
                -ATT_TRIM_MAX, ATT_TRIM_MAX))
        x_des = self.f_c0 + du * (np.tan(cmd.pitch_ref) - np.tan(lean_ff)) \
            + self.x_trim
        x_des = float(np.clip(x_des, xlo, xhi))
        self.x_fore += float(np.clip(x_des - self.x_fore,
                                     -X_FORE_RATE * dt, X_FORE_RATE * dt))
        heights = {"right": cmd.h_ref + dh / 2, "left": cmd.h_ref - dh / 2}
        for s in SIDES:
            h = float(np.clip(heights[s], *legs.height_range))
            inner, outer, res = legs.ik(s, self.x_fore, h)
            if res > 1e-6:      # unreachable: keep the previous targets
                continue
            for m, q_des in (("motor_inner", inner), ("motor_outer", outer)):
                name = f"{s}_{m}"
                q_meas = float(self.robot.sensor(f"{name}_pos")[0])
                self.trim[name] = float(np.clip(
                    self.trim[name] + LEG_TRIM_KI * (q_des - q_meas) * dt,
                    -LEG_TRIM_MAX, LEG_TRIM_MAX)) * ramp
                self.robot.set_leg_target(s, m, q_des + self.trim[name])

    # ------------------------------------------------------------------ HUD
    def status(self):
        est, cmd = self.est, self.cmd
        return (f"t={self.t:6.2f}s v={est.v:+5.2f}/{cmd.v_ref:+5.2f} m/s "
                f"yaw={est.yaw_rate:+5.2f}/{cmd.yaw_ref:+5.2f} rad/s "
                f"h={cmd.h_ref:.3f} m lean={np.degrees(est.lean):+6.2f} "
                f"pitch={np.degrees(est.pitch):+6.2f} roll={np.degrees(est.roll):+6.2f} deg"
                + ("  ** FALLEN **" if self.fallen else ""))


# --------------------------------------------------------------------- tests
def run_test(name, seconds=None):
    robot = Robot()
    ctl = WholeBodyController(robot)
    cmd = ctl.cmd
    T = {"station": 12, "drive": 14, "spin": 10, "arc": 14,
         "height": 20, "pitch": 16, "mixed": 24}.get(name, 12)
    T = seconds or T

    def script(t):
        if name == "station":
            pass
        elif name == "drive":
            cmd.v_tgt = 1.2 if 1 < t < 9 else 0.0
        elif name == "spin":
            cmd.yaw_tgt = 1.5 if 1 < t < 8 else 0.0
        elif name == "arc":
            cmd.v_tgt = 1.0 if 1 < t else 0.0
            cmd.yaw_tgt = 0.8 if 3 < t < 12 else 0.0
        elif name == "height":
            lo, hi = cmd.h_range
            cmd.h_tgt = hi if 2 < t < 8 else (lo if 8 <= t < 15 else (lo + hi) / 2)
        elif name == "pitch":
            cmd.pitch_tgt = 0.15 if 2 < t < 8 else (-0.15 if 8 <= t < 14 else 0.0)
        elif name == "mixed":
            cmd.v_tgt = 1.0 if 2 < t < 20 else 0.0
            cmd.yaw_tgt = 0.6 if 6 < t < 12 else (-0.6 if 12 <= t < 18 else 0.0)
            lo, hi = cmd.h_range
            cmd.h_tgt = lo if 8 < t < 16 else (lo + hi) / 2

    peak_lean, v_err_hold, yaw_err_hold, h_err_hold, pitch_err_hold = 0, [], [], [], []
    n = int(T / ctl.dt)
    hud_every = max(1, int(1.0 / ctl.dt))
    for i in range(n):
        t = i * ctl.dt
        script(t)
        ctl.tick()
        robot.step_control_period()
        peak_lean = max(peak_lean, abs(ctl.est.lean))
        if abs(cmd.v_ref - cmd.v_tgt) < 1e-6 and abs(cmd.v_tgt) > 0:
            v_err_hold.append(ctl.est.v - cmd.v_ref)
        if abs(cmd.yaw_ref - cmd.yaw_tgt) < 1e-6 and abs(cmd.yaw_tgt) > 0:
            yaw_err_hold.append(ctl.est.yaw_rate - cmd.yaw_ref)
        if abs(cmd.h_ref - cmd.h_tgt) < 1e-6:
            geo = ctl.est.pend
            h_meas = float(np.mean([(ctl.legs.pivot[s] - geo["centers"][s])
                                    @ ctl.legs.up_t for s in SIDES]))
            h_err_hold.append(h_meas - cmd.h_ref)
        if abs(cmd.pitch_ref - cmd.pitch_tgt) < 1e-6 and abs(cmd.pitch_tgt) > 0:
            pitch_err_hold.append(ctl.est.pitch - cmd.pitch_ref)
        if i % hud_every == 0:
            print(ctl.status())
    print(ctl.status())

    def stat(v):
        return f"mean {np.mean(v):+.4f}  rms {np.sqrt(np.mean(np.square(v))):.4f}" if v else "n/a"
    print(f"\n== {name}: peak |lean| = {np.degrees(peak_lean):.2f} deg, "
          f"fallen = {ctl.fallen}")
    print(f"   v error while holding:     {stat(v_err_hold)}")
    print(f"   yaw error while holding:   {stat(yaw_err_hold)}")
    print(f"   height error while holding:{stat(h_err_hold)}")
    print(f"   pitch error while holding: {stat(pitch_err_hold)}")
    return not ctl.fallen


# -------------------------------------------------------------------- teleop
def teleop():
    import mujoco.viewer
    robot = Robot()
    ctl = WholeBodyController(robot)
    cmd = ctl.cmd

    def key(code):
        c = chr(code).upper() if 0 <= code < 256 else ""
        if c == "W":
            cmd.v_tgt += 0.4
        elif c == "S":
            cmd.v_tgt -= 0.4
        elif c == "A":
            cmd.yaw_tgt += 0.5
        elif c == "D":
            cmd.yaw_tgt -= 0.5
        elif c == "R":
            cmd.h_tgt += 0.03
        elif c == "F":
            cmd.h_tgt -= 0.03
        elif c == "T":
            cmd.pitch_tgt += 0.08
        elif c == "G":
            cmd.pitch_tgt -= 0.08
        elif c == "Q":
            cmd.roll_tgt += 0.08
        elif c == "E":
            cmd.roll_tgt -= 0.08
        elif c == "X":
            cmd.v_tgt = cmd.yaw_tgt = cmd.pitch_tgt = cmd.roll_tgt = 0.0

    print(__doc__.split("Teleop keys")[1])
    with mujoco.viewer.launch_passive(robot.model, robot.data,
                                      key_callback=key) as viewer:
        last_hud = 0.0
        while viewer.is_running():
            ctl.tick()
            robot.step_control_period()
            viewer.sync()
            if ctl.t - last_hud > 0.5:
                print("\r" + ctl.status(), end="", flush=True)
                last_hud = ctl.t
            time.sleep(max(0, robot.decimation * robot.model.opt.timestep - 0.002))


def main():
    ap = argparse.ArgumentParser(description="whole-body drive controller")
    ap.add_argument("--test", choices=["station", "drive", "spin", "arc",
                                       "height", "pitch", "mixed", "all"],
                    help="headless scripted test instead of the viewer")
    ap.add_argument("--seconds", type=float, default=None)
    args = ap.parse_args()
    if args.test == "all":
        results = {n: run_test(n) for n in
                   ("station", "drive", "spin", "arc", "height", "pitch", "mixed")}
        print("\n==== summary ====")
        for n, ok in results.items():
            print(f"  {n:8s} {'PASS' if ok else 'FALL'}")
        raise SystemExit(0 if all(results.values()) else 1)
    if args.test:
        ok = run_test(args.test, args.seconds)
        raise SystemExit(0 if ok else 1)
    teleop()


if __name__ == "__main__":
    main()
