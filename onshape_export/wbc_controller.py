"""Task-space whole-body controller (inverse-dynamics QP) for the wheeled biped.

Where drive_controller.py distributes commands by hand (balance law + leg IK
position targets + trim integrators), this controller solves ONE weighted
least-squares problem per tick over the full floating-base dynamics:

    variables  z = [qdd (16), contact forces (6), closure forces (12),
                    leg torques (4)]
    hard rows  M qdd + bias = S^T tau + Jc^T f + Je^T lambda   (dynamics)
               wheel axle: vertical hold, rolling, no side slip (contact,
               with the nonholonomic centripetal term -- without it any yaw
               rate is "infeasible" and the QP refuses to turn)
               four-bar loop closures                          (closure)
    task rows  leg joints -> the SAME IK backbone the cascade uses
               torso pitch, base height -> light corrective layers
               regularization -> smallest qdd/f/tau

Legs are torque-controlled by inverting the MIT-mode PD
(ctrl = q + (tau + kv qv)/kp): gravity/contact/acceleration coupling come
out of the dynamics exactly, so the cascade's leg-sag trim integrator is
unnecessary and height/posture transitions are measurably smoother
(height-sweep peak lean 8.4 deg vs the cascade's 16.5).

The WHEELS keep the cascade's speed-command law: velocity-mode wheel drives
are not torque-transparent (a torque-level wheel loop through them whipsaws
at the control rate -- verified here and throughout the cascade's history).
Their servo torques enter the QP as known inputs.

Hard-won rules encoded here (each verified by a fall):
  - one authority per variable: banked-turn roll is fed forward through the
    IK heights, so the roll feedback task is DISABLED (W_ROLL = 0); with
    both active the robot rolls over at bank transitions.
  - the joint backbone must stay SOFT (KP_J ~ 80): stiff torque tracking
    through the PD inversion destabilizes bank transitions.
  - never feed the measured COM or attitude into leg targets at bandwidth.

The high-level shaping (approach law, lean feedforward, workspace caps,
governor) and the estimator (leg-motion-compensated odometry) are reused
from drive_controller, so both controllers see identical references and
measurements; only the low-level distribution differs.

Status vs the cascade: passes the same 7-scenario battery with better
numbers on 5 of 7 (notably smoother pose transitions), equal agility,
recovers 40 N shoves, ~1.5 ms/tick. The cascade is still more robust to
adversarial command-mashing at deep crouch; prefer it for rough teleop.

Run:
    python wbc_controller.py                 # viewer + keyboard teleop
    python wbc_controller.py --test all      # same 7-scenario battery

Teleop keys are identical to drive_controller.py.
"""
from __future__ import annotations

import argparse
import time

import mujoco
import numpy as np

from controller_demo import Robot, SIDES
from stand_pose import jid, HERE, collidable_min_z
import drive_controller as dc
from drive_controller import G

FLOATING_XML = HERE / "robot" / "robot_full_floating.xml"

# task weights (hard constraints get W_HARD). Keep the weight span modest:
# the closure rows are rank-deficient, and with a 1e8 dynamic range the
# lstsq null-space handling produces qdd ~ 1e4 numerical garbage.
W_HARD = 1e3
W_JPOS = 10.0          # leg joint tracking of the IK backbone
W_PITCH = 8.0          # corrective attitude layer on top of the backbone
W_ROLL = 0.0           # roll feedback DISABLED: the banked-turn roll is fed
                       # forward through the IK heights, and a roll feedback
                       # task fights that feedforward (one authority per
                       # variable -- verified: with both active the robot
                       # rolls over at bank transitions)
W_HEIGHT = 8.0
REG_QDD = 3e-2
REG_F = 1e-2
REG_TAU = 3e-2
LSTSQ_RCOND = 1e-9
KP_J = 80.0            # 1/s^2, joint backbone stiffness. Raising it (tested
                       # up to 700) destabilizes bank transitions: the
                       # discrete torque tracking through the MIT-mode
                       # inversion does not tolerate a stiff backbone.
KD_J = 12.0            # 1/s
TAU_RATE = 150.0       # N m / s, leg torque slew limit

KP_ATT = 60.0          # 1/s^2, attitude task stiffness
KD_ATT = 12.0          # 1/s
KP_Z = 80.0            # 1/s^2, height task stiffness
KD_Z = 15.0            # 1/s
TAU_MAX = 11.0


class FloatingModel:
    """Internal floating-base dynamics model, filled from the estimator."""

    def __init__(self, est: "dc.Estimator"):
        self.est = est
        self.m = mujoco.MjModel.from_xml_path(str(FLOATING_XML))
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.jadr = {}
        self.jdof = {}
        for j in range(m.njnt):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
                self.jadr[name] = m.jnt_qposadr[j]
                self.jdof[name] = m.jnt_dofadr[j]
        self.wheel_body = {s: int(m.jnt_bodyid[jid(m, f"{s}_wheel")]) for s in SIDES}
        self.wheel_jnt = {s: jid(m, f"{s}_wheel") for s in SIDES}
        # connect-equality site pairs (loop closures)
        self.pairs = []
        for e in range(m.neq):
            if m.eq_type[e] == mujoco.mjtEq.mjEQ_CONNECT:
                self.pairs.append((int(m.eq_obj1id[e]), int(m.eq_obj2id[e])))
        self.passive_dofs = np.array(
            [self.jdof[f"{s}_{j}"] for s in SIDES
             for j in ("passive_biella_inner", "passive_biella_outer")])
        # actuator -> joint dof, and PD constants for torque inversion
        self.act = []
        for a in range(m.nu):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
            self.act.append(dict(
                name=name, dof=self.jdof[name],
                kp=float(-m.actuator_biasprm[a, 1]),      # 0 for wheels
                kv=float(-m.actuator_biasprm[a, 2]),
                wheel=name.endswith("_wheel")))

    # ---------------------------------------------------------------- state
    def fill_state(self, robot: Robot):
        """Populate (qpos, qvel) from what the real robot can measure."""
        m, d, est = self.m, self.d, self.est
        legs = est.legs
        d.qpos[:] = 0.0
        d.qpos[3:7] = robot.sensor("imu_quat")
        for s in SIDES:
            for j in ("motor_inner", "motor_outer", "wheel"):
                d.qpos[self.jadr[f"{s}_{j}"]] = robot.sensor(f"{s}_{j}_pos")[0]
            for j in ("passive_biella_inner", "passive_biella_outer"):
                # closure-solved on the estimation model, same joint names
                d.qpos[self.jadr[f"{s}_{j}"]] = \
                    legs.d_meas.qpos[legs.model.jnt_qposadr[jid(legs.model, f"{s}_{j}")]]
        # place the tires on the ground
        mujoco.mj_kinematics(m, d)
        d.qpos[2] -= collidable_min_z(m, d)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)

        # velocities: encoders + gyro; passives from the closure rate equation
        d.qvel[:] = 0.0
        for s in SIDES:
            for j in ("motor_inner", "motor_outer", "wheel"):
                d.qvel[self.jdof[f"{s}_{j}"]] = robot.sensor(f"{s}_{j}_vel")[0]
        d.qvel[3:6] = robot.sensor("imu_gyro")          # body frame
        # base linear velocity: odometry forward speed at the axle midpoint,
        # transported to the base origin
        R = est.R()
        fh = R @ legs.fwd_t
        fh = fh - np.array([0, 0, 1.0]) * fh[2]
        fh /= max(np.linalg.norm(fh), 1e-9)
        axle_mid_w = np.mean([d.xanchor[self.wheel_jnt[s]] for s in SIDES], axis=0)
        omega_w = R @ d.qvel[3:6]
        d.qvel[0:3] = est.v * fh + np.cross(omega_w, d.xpos[1] - axle_mid_w)
        # passive joint rates: J_e qvel = 0, solve for the passive dofs
        Je = self._closure_jac()
        rest = Je @ d.qvel                                # residual w/ passives=0
        Jp = Je[:, self.passive_dofs]
        sol, *_ = np.linalg.lstsq(Jp, -rest, rcond=None)
        d.qvel[self.passive_dofs] = sol
        mujoco.mj_forward(m, d)

    def _closure_jac(self):
        m, d = self.m, self.d
        rows = []
        for a, b in self.pairs:
            Ja = np.zeros((3, m.nv))
            Jb = np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, d, Ja, None, a)
            mujoco.mj_jacSite(m, d, Jb, None, b)
            rows.append(Ja - Jb)
        return np.vstack(rows)

    def _closure_jacdot_qvel(self):
        m, d = self.m, self.d
        out = []
        for a, b in self.pairs:
            Jda = np.zeros((3, m.nv))
            Jdb = np.zeros((3, m.nv))
            mujoco.mj_jacDot(m, d, Jda, None, d.site_xpos[a], int(m.site_bodyid[a]))
            mujoco.mj_jacDot(m, d, Jdb, None, d.site_xpos[b], int(m.site_bodyid[b]))
            out.append((Jda - Jdb) @ d.qvel)
        return np.concatenate(out)

    # -------------------------------------------------------------- contact
    def contact_rows(self):
        """Per wheel, at the AXLE point (a material point, so mj_jacDot is
        exact): vertical hold, rolling (axle forward speed = r * spin), and
        no side slip. Returns (Jc, jdot_qvel) with 3 rows per wheel."""
        m, d, est = self.m, self.d, self.est
        r = est.legs.wheel_radius
        z = np.array([0.0, 0, 1.0])
        R_base = d.xmat[1].reshape(3, 3)
        rows, rhs = [], []
        for s in SIDES:
            j = self.wheel_jnt[s]
            body = self.wheel_body[s]
            carrier = int(m.body_parentid[body])
            p = d.xanchor[j].copy()
            n = d.xaxis[j].copy()
            n /= np.linalg.norm(n)
            f = np.cross(n, z)
            f /= max(np.linalg.norm(f), 1e-9)
            nh = n - z * (n @ z)
            nh /= max(np.linalg.norm(nh), 1e-9)
            Jp = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, Jp, None, p, carrier)     # axle point on carrier
            Jr = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, None, Jr, p, body)        # wheel angular
            Jdp = np.zeros((3, m.nv))
            mujoco.mj_jacDot(m, d, Jdp, None, p, carrier)
            Jdr = np.zeros((3, m.nv))
            mujoco.mj_jacDot(m, d, None, Jdr, p, body)
            v_axle = Jp @ d.qvel
            # vertical: z . a_axle = 0
            rows.append(z @ Jp)
            rhs.append(-(z @ (Jdp @ d.qvel)))
            # rolling: f . a_axle - r * n . alpha_wheel = 0
            rows.append(f @ Jp - r * (n @ Jr))
            rhs.append(-(f @ (Jdp @ d.qvel)) + r * (n @ (Jdr @ d.qvel)))
            # no side slip is NONHOLONOMIC: the lateral direction rotates
            # with the yaw, so the axle point legitimately carries the
            # centripetal acceleration omega_yaw * (f . v). Without this
            # term any yaw rate is "infeasible" and the QP refuses to turn.
            yaw_rate = float((R_base @ d.qvel[3:6]) @ z)
            rows.append(nh @ Jp)
            rhs.append(-(nh @ (Jdp @ d.qvel)) + yaw_rate * float(f @ v_axle))
        return np.vstack(rows), np.array(rhs)


class TaskSpaceWBC:
    """Whole-body controller: same command interface as drive_controller's
    cascade, torque-level task-space optimization underneath."""

    def __init__(self, robot: Robot):
        self.robot = robot
        self.dt = robot.decimation * robot.model.opt.timestep
        self.legs = dc.LegModel()
        self.est = dc.Estimator(robot, self.legs)
        self.cmd = dc.Commands(self.legs)
        self.fm = FloatingModel(self.est)
        self.est.update()

        # shaping state (mirrors drive_controller)
        self.bal_kp = self.bal_kd = None
        self.l_design = None
        self.s_ref = None
        self.lean_ff = 0.0
        self.x_trim = 0.0
        self.x_fore = float(np.mean(list(self.legs.x0.values())))
        self.tau_prev = np.zeros(4)
        self.fallen = False
        self.t = 0.0

        geo0 = self.est.pend
        self.pivot_mid = np.mean([self.legs.pivot[s] for s in SIDES], axis=0)
        self.f_c0 = float((geo0["com"] - self.pivot_mid) @ self.legs.fwd_t)
        w_mid0 = np.mean([geo0["centers"][s] for s in SIDES], axis=0)
        h0 = float(np.mean(list(self.legs.h0.values())))
        self.c0 = float((geo0["com"] - w_mid0) @ self.legs.up_t) - h0
        # height-task calibration: base z at spawn vs commanded h
        self.fm.fill_state(robot)
        self.z_off = float(self.fm.d.qpos[2]) - h0
        # hold targets for the settle phase
        self.hold_ctrl = robot.stand_ctrl.copy()

    def refresh_gains(self):
        p = self.est.pend
        if self.l_design is None or abs(p["l"] - self.l_design) > 0.02 * self.l_design:
            H = p["l"] + self.legs.wheel_radius
            self.bal_kp = dc.BAL_KP_SCALE * np.sqrt(G * H)
            self.bal_kd = dc.BAL_KD_FRACT * self.bal_kp
            self.l_design = p["l"]

    # ----------------------------------------------------------------- tick
    def tick(self):
        dt = self.dt
        est, cmd, legs = self.est, self.cmd, self.legs
        est.update()

        # ---- high-level shaping: identical to drive_controller ----------
        du = cmd.h_ref + self.c0
        xlo, xhi = legs.x_range(cmd.h_ref)
        x_neutral = self.f_c0 + du * np.tan(cmd.pitch_ref) + self.x_trim
        a_cap_pos = min(dc.ACCEL_MAX, max(0.2, G * (x_neutral - xlo) / du))
        a_cap_neg = min(dc.ACCEL_MAX, max(0.2, G * (xhi - x_neutral) / du))
        v_pred = cmd.v_ref + G * self.lean_ff * dc.ARR_LEAD
        a_int = float(np.clip(dc.K_ARR_V * (cmd.v_tgt - v_pred),
                              -a_cap_neg, a_cap_pos))
        cmd.slew(dt, a_int, est.v)
        self.refresh_gains()
        self.t += dt

        if self.s_ref is None:
            self.s_ref = est.s
        if abs(est.lean) > dc.FALL_LEAN:
            self.fallen = True
        ramp = float(np.clip((self.t - dc.SETTLE_TIME) / 0.5, 0.0, 1.0))
        if self.fallen:
            ramp = 0.0

        self.s_ref += cmd.v_ref * dt
        self.s_ref = float(np.clip(self.s_ref, est.s - dc.SREF_WINDUP,
                                   est.s + dc.SREF_WINDUP))
        a_dem = float(np.clip(cmd.accel
                              + dc.KP_POS * (self.s_ref - est.s)
                              + dc.KD_POS * G * (cmd.v_ref - est.v),
                              -a_cap_neg, a_cap_pos))
        self.lean_ff += (dt / dc.LEAN_FF_TAU) * (a_dem / G - self.lean_ff)
        lean_ff = self.lean_ff
        v_bal = self.bal_kp * (est.lean - lean_ff) + self.bal_kd * est.lean_rate
        v_des = cmd.v_ref + v_bal        # target ground speed of the wheels

        if ramp < 1.0 or self.fallen:
            # settle phase: plain keyframe hold, wheels at zero speed; keep
            # the references parked so the engage tick sees no command step
            cmd.v_ref = cmd.yaw_ref = cmd.accel = 0.0
            self.lean_ff = 0.0
            self.s_ref = est.s
            self.robot.data.ctrl[:] = self.hold_ctrl
            for s in SIDES:
                self.robot.data.ctrl[self.robot._act[f"{s}_wheel"]] = 0.0
            return

        # attitude trim (same freeze-during-accel logic as the cascade)
        if abs(lean_ff) < 0.02:
            self.x_trim = float(np.clip(
                self.x_trim + dc.ATT_KI * (cmd.pitch_ref - est.pitch) * dt,
                -dc.ATT_TRIM_MAX, dc.ATT_TRIM_MAX))

        # ---- wheels: the proven speed-command law (velocity-servo wheels
        # are not torque-transparent -- torque-level wheel control through
        # them whipsaws at the control rate, verified here and in the
        # cascade's history). The WBC below handles the LEGS.
        r_w = legs.wheel_radius
        dw = (cmd.yaw_ref * legs.track / (2 * r_w)
              + dc.YAW_KP * (cmd.yaw_ref - est.yaw_rate) / r_w)
        w_base = (est.v + v_bal) / r_w
        wheel_ctrl = {}
        wheel_tau_est = {}
        for s, sgn in (("right", +1.0), ("left", -1.0)):
            w_fwd = float(np.clip(w_base + sgn * dw, -44.0, 44.0))
            ctrl = est.fwd_sign * self.robot.wheel_dir[s] * w_fwd
            wheel_ctrl[s] = ctrl
            qv = float(self.robot.sensor(f"{s}_wheel_vel")[0])
            wheel_tau_est[s] = 1.0 * (ctrl - qv)   # kv = 1: servo torque now

        # ---- fill the internal model and build the WBC problem ----------
        fm = self.fm
        fm.fill_state(self.robot)
        m, d = fm.m, fm.d
        nv = m.nv
        M = np.zeros((nv, nv))
        mujoco.mj_fullM(m, M, d.qM)
        bias = (d.qfrc_bias - d.qfrc_passive).copy()
        # the wheel servo torques are KNOWN inputs: fold them into the bias
        for s in SIDES:
            dof = fm.jdof[f"{s}_wheel"]
            bias[dof] -= wheel_tau_est[s]

        Jc, c_rhs = fm.contact_rows()                     # 6 x nv
        Je = fm._closure_jac()                            # 12 x nv
        e_rhs = -fm._closure_jacdot_qvel()

        leg_acts = [a for a in fm.act if not a["wheel"]]
        nc, ne, nu = Jc.shape[0], Je.shape[0], len(leg_acts)
        nz = nv + nc + ne + nu
        sl_q = slice(0, nv)
        sl_f = slice(nv, nv + nc)
        sl_l = slice(nv + nc, nv + nc + ne)
        sl_t = slice(nv + nc + ne, nz)

        A_rows, b_rows, w_rows = [], [], []

        def add(Arow, brhs, w):
            A_rows.append(Arow)
            b_rows.append(brhs)
            w_rows.append(w)

        # dynamics (hard): M qdd - Jc^T f - Je^T lam - S^T tau_legs = -bias
        S = np.zeros((nu, nv))
        for i, a in enumerate(leg_acts):
            S[i, a["dof"]] = 1.0
        Adyn = np.zeros((nv, nz))
        Adyn[:, sl_q] = M
        Adyn[:, sl_f] = -Jc.T
        Adyn[:, sl_l] = -Je.T
        Adyn[:, sl_t] = -S.T
        for i in range(nv):
            add(Adyn[i], -bias[i], W_HARD)
        # contact + closure (hard)
        for i in range(nc):
            row = np.zeros(nz)
            row[sl_q] = Jc[i]
            add(row, c_rhs[i], W_HARD)
        for i in range(ne):
            row = np.zeros(nz)
            row[sl_q] = Je[i]
            add(row, e_rhs[i], W_HARD)

        # ---- tasks -------------------------------------------------------
        R = est.R()
        z_w = np.array([0.0, 0, 1.0])
        n_w = R @ legs.n_t
        f_h = R @ legs.fwd_t
        f_h = f_h - z_w * (f_h @ z_w)
        f_h /= max(np.linalg.norm(f_h), 1e-9)
        n_h = n_w - z_w * (n_w @ z_w)
        n_h /= max(np.linalg.norm(n_h), 1e-9)

        Jp_b = np.zeros((3, nv))
        Jr_b = np.zeros((3, nv))
        mujoco.mj_jac(m, d, Jp_b, Jr_b, d.xpos[1], 1)
        Jdp_b = np.zeros((3, nv))
        Jdr_b = np.zeros((3, nv))
        mujoco.mj_jacDot(m, d, Jdp_b, Jdr_b, d.xpos[1], 1)

        # leg joint backbone: track the SAME IK solution the cascade uses
        # (command-derived fore-aft incl. the acceleration lean shift, ride
        # height, and banked-turn differential heights). The QP wraps this
        # kinematic backbone in dynamics-consistent torques -- gravity and
        # acceleration coupling come out exactly, no trim integrators.
        x_des = self.f_c0 + du * (np.tan(cmd.pitch_ref) - np.tan(lean_ff)) \
            + self.x_trim
        xlo2, xhi2 = legs.x_range(cmd.h_ref)
        x_des = float(np.clip(x_des, xlo2, xhi2))
        self.x_fore += float(np.clip(x_des - self.x_fore,
                                     -dc.X_FORE_RATE * dt, dc.X_FORE_RATE * dt))
        bank = np.arctan(cmd.v_ref * cmd.yaw_ref / G) + cmd.roll_off
        dh = legs.track * np.tan(bank) if abs(bank) < 0.5 else 0.0
        heights = {"right": cmd.h_ref + dh / 2, "left": cmd.h_ref - dh / 2}
        for s in SIDES:
            h_s = float(np.clip(heights[s], *legs.height_range))
            inner, outer, res = legs.ik(s, self.x_fore, h_s)
            if res > 1e-6:
                continue
            for jn, q_des in ((f"{s}_motor_inner", inner),
                              (f"{s}_motor_outer", outer)):
                q_m = float(self.robot.sensor(f"{jn}_pos")[0])
                qv_m = float(self.robot.sensor(f"{jn}_vel")[0])
                row = np.zeros(nz)
                row[sl_q.start + fm.jdof[jn]] = 1.0
                add(row, KP_J * (q_des - q_m) - KD_J * qv_m, W_JPOS)

        # torso attitude: pitch about the horizontal axle axis
        gyro = self.robot.sensor("imu_gyro")
        pitch_rate = float(gyro @ legs.n_t)
        roll_rate = float(gyro @ legs.fwd_t)
        att_pitch = KP_ATT * (cmd.pitch_ref - est.pitch) - KD_ATT * pitch_rate
        row = np.zeros(nz)
        row[sl_q] = n_h @ Jr_b
        add(row, att_pitch - n_h @ (Jdr_b @ d.qvel), W_PITCH)
        # roll feedback (disabled by default, see W_ROLL)
        if W_ROLL > 0.0:
            att_roll = KP_ATT * (bank - est.roll) - KD_ATT * roll_rate
            row = np.zeros(nz)
            row[sl_q] = f_h @ Jr_b
            add(row, att_roll - f_h @ (Jdr_b @ d.qvel), W_ROLL)
        # base height
        z_des = self.z_off + cmd.h_ref
        z_acc = KP_Z * (z_des - float(d.qpos[2])) - KD_Z * float(d.qvel[2])
        row = np.zeros(nz)
        row[sl_q] = z_w @ Jp_b
        add(row, z_acc - z_w @ (Jdp_b @ d.qvel), W_HEIGHT)

        # regularization
        for i in range(nv):
            row = np.zeros(nz)
            row[sl_q.start + i] = 1.0
            add(row, 0.0, REG_QDD)
        for i in range(nc + ne):
            row = np.zeros(nz)
            row[sl_f.start + i] = 1.0
            add(row, 0.0, REG_F)
        for i in range(nu):
            row = np.zeros(nz)
            row[sl_t.start + i] = 1.0
            add(row, 0.0, REG_TAU)

        A = np.vstack(A_rows) * np.asarray(w_rows)[:, None]
        b = np.asarray(b_rows) * np.asarray(w_rows)
        zsol, *_ = np.linalg.lstsq(A, b, rcond=LSTSQ_RCOND)
        tau = np.clip(zsol[sl_t], -TAU_MAX, TAU_MAX)
        # robustness: reject a numerically bad solve (the rank-deficient
        # closure rows occasionally produce a garbage direction) and slew-
        # limit torques so one bad tick can never be a torque bomb
        if (not np.all(np.isfinite(zsol))
                or float(np.max(np.abs(zsol[sl_q]))) > 500.0):
            tau = self.tau_prev
        tau = np.clip(tau, self.tau_prev - TAU_RATE * dt,
                      self.tau_prev + TAU_RATE * dt)
        self.tau_prev = tau

        # ---- commands: wheels at speed, legs at inverted-PD torque ------
        for s in SIDES:
            self.robot.data.ctrl[self.robot._act[f"{s}_wheel"]] = wheel_ctrl[s]
        for i, a in enumerate(leg_acts):
            name = a["name"]
            q = float(self.robot.sensor(f"{name}_pos")[0])
            qv = float(self.robot.sensor(f"{name}_vel")[0])
            ctrl = q + (tau[i] + a["kv"] * qv) / a["kp"]
            self.robot.data.ctrl[self.robot._act[name]] = ctrl
        self.last_tau = tau

    # ------------------------------------------------------------------ HUD
    def status(self):
        est, cmd = self.est, self.cmd
        return (f"t={self.t:6.2f}s v={est.v:+5.2f}/{cmd.v_ref:+5.2f} m/s "
                f"yaw={est.yaw_rate:+5.2f}/{cmd.yaw_ref:+5.2f} rad/s "
                f"h={cmd.h_ref:.3f} m lean={np.degrees(est.lean):+6.2f} "
                f"pitch={np.degrees(est.pitch):+6.2f} roll={np.degrees(est.roll):+6.2f} deg"
                + ("  ** FALLEN **" if self.fallen else ""))


# ------------------------------------------------------------------- tests
def run_test(name, seconds=None):
    robot = Robot()
    ctl = TaskSpaceWBC(robot)
    cmd = ctl.cmd
    T = {"station": 12, "drive": 14, "spin": 10, "arc": 14,
         "height": 20, "pitch": 16, "mixed": 24}.get(name, 12)
    T = seconds or T

    def script(t):
        if name == "drive":
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

    peak = 0.0
    v_err, yaw_err = [], []
    n = int(T / ctl.dt)
    hud = max(1, int(1.0 / ctl.dt))
    for i in range(n):
        t = i * ctl.dt
        script(t)
        ctl.tick()
        robot.step_control_period()
        peak = max(peak, abs(ctl.est.lean))
        if abs(cmd.v_ref - cmd.v_tgt) < 1e-6 and abs(cmd.v_tgt) > 0:
            v_err.append(ctl.est.v - cmd.v_ref)
        if abs(cmd.yaw_ref - cmd.yaw_tgt) < 1e-6 and abs(cmd.yaw_tgt) > 0:
            yaw_err.append(ctl.est.yaw_rate - cmd.yaw_ref)
        if i % hud == 0:
            print(ctl.status())
    print(ctl.status())

    def stat(v):
        return (f"mean {np.mean(v):+.4f} rms {np.sqrt(np.mean(np.square(v))):.4f}"
                if v else "n/a")
    print(f"\n== {name}: peak |lean| = {np.degrees(peak):.2f} deg, fallen = {ctl.fallen}")
    print(f"   v error while holding:   {stat(v_err)}")
    print(f"   yaw error while holding: {stat(yaw_err)}")
    return not ctl.fallen


def teleop():
    import mujoco.viewer
    robot = Robot()
    ctl = TaskSpaceWBC(robot)
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

    with mujoco.viewer.launch_passive(robot.model, robot.data,
                                      key_callback=key) as viewer:
        last = 0.0
        while viewer.is_running():
            ctl.tick()
            robot.step_control_period()
            viewer.sync()
            if ctl.t - last > 0.5:
                print("\r" + ctl.status(), end="", flush=True)
                last = ctl.t
            time.sleep(max(0, ctl.dt - 0.002))


def main():
    ap = argparse.ArgumentParser(description="task-space whole-body controller")
    ap.add_argument("--test", choices=["station", "drive", "spin", "arc",
                                       "height", "pitch", "mixed", "all"])
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
        raise SystemExit(0 if run_test(args.test, args.seconds) else 1)
    teleop()


if __name__ == "__main__":
    main()
