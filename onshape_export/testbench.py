"""Kinematic testbench for the closed-loop leg.

Drive the two motor joints directly (skipping dynamics); the passive
joints are solved each call from the loop-closure equality residual via
Newton iteration on site Jacobians. Useful for verifying an IK solution:
your IK gives (motor_inner, motor_outer) for a target foot pose, this
script evaluates the resulting world-frame foot position.

Run directly to see a sinusoidal sweep in mujoco.viewer.
"""
from __future__ import annotations

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

SCENE = Path(__file__).parent / "leg" / "scene.xml"

MOTOR_JOINTS = ("motor_inner", "motor_outer")
PASSIVE_JOINTS = ("passive_biella_inner", "passive_biella_outer")
WHEEL_JOINT = "wheel"
CLOSURE_PAIRS = (
    ("closing_manovelle_meeting_1", "closing_manovelle_meeting_2"),
    ("closing_manovelle_meeting_1_z", "closing_manovelle_meeting_2_z"),
)
FOOT_BODY = "motor_shaft_2_3"


class LegFK:
    def __init__(self, xml_path: str | Path = SCENE):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        m = self.model

        all_joints = MOTOR_JOINTS + PASSIVE_JOINTS + (WHEEL_JOINT,)
        self.qpos_addr = {n: m.jnt_qposadr[self._jid(n)] for n in all_joints}
        self.dof_addr = {n: m.jnt_dofadr[self._jid(n)] for n in all_joints}
        self.passive_dofs = np.array([self.dof_addr[n] for n in PASSIVE_JOINTS])
        self.site_id = {s: self._sid(s) for pair in CLOSURE_PAIRS for s in pair}
        self.foot_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, FOOT_BODY)

        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)

    def _jid(self, name: str) -> int:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)

    def _sid(self, name: str) -> int:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)

    def _closure_residual(self) -> np.ndarray:
        d = self.data
        return np.concatenate([
            d.site_xpos[self.site_id[a]] - d.site_xpos[self.site_id[b]]
            for a, b in CLOSURE_PAIRS
        ])

    def _closure_jacobian(self) -> np.ndarray:
        m, d = self.model, self.data
        rows = []
        for a, b in CLOSURE_PAIRS:
            Ja = np.zeros((3, m.nv))
            Jb = np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, d, Ja, None, self.site_id[a])
            mujoco.mj_jacSite(m, d, Jb, None, self.site_id[b])
            rows.append(Ja - Jb)
        return np.vstack(rows)

    def set_motor_angles(
        self,
        inner: float,
        outer: float,
        wheel: float | None = None,
        max_iter: int = 50,
        tol: float = 1e-10,
    ) -> float:
        """Write motor qpos, solve passive joints, return final closure error."""
        d = self.data
        d.qpos[self.qpos_addr["motor_inner"]] = inner
        d.qpos[self.qpos_addr["motor_outer"]] = outer
        if wheel is not None:
            d.qpos[self.qpos_addr["wheel"]] = wheel

        err = float("inf")
        for _ in range(max_iter):
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)
            r = self._closure_residual()
            err = float(np.linalg.norm(r))
            if err < tol:
                break
            J = self._closure_jacobian()[:, self.passive_dofs]
            dq, *_ = np.linalg.lstsq(J, -r, rcond=None)
            d.qpos[self.qpos_addr["passive_biella_inner"]] += dq[0]
            d.qpos[self.qpos_addr["passive_biella_outer"]] += dq[1]
        return err

    def foot_pos(self) -> np.ndarray:
        return self.data.xpos[self.foot_body_id].copy()

    def passive_angles(self) -> tuple[float, float]:
        return (
            float(self.data.qpos[self.qpos_addr["passive_biella_inner"]]),
            float(self.data.qpos[self.qpos_addr["passive_biella_outer"]]),
        )


def main() -> None:
    fk = LegFK()
    print(f"loaded {SCENE.name}: nq={fk.model.nq} nv={fk.model.nv} neq={fk.model.neq}")
    print("driving motor_inner / motor_outer with sinusoid; passive joints solved each frame")

    with mujoco.viewer.launch_passive(fk.model, fk.data) as viewer:
        t0 = time.time()
        last_log = 0.0
        while viewer.is_running():
            t = time.time() - t0
            inner = 0.4 * np.sin(0.5 * t)
            outer = 0.4 * np.sin(0.5 * t + 1.2)
            err = fk.set_motor_angles(inner, outer)
            if t - last_log > 0.5:
                last_log = t
                p = fk.foot_pos()
                pi, po = fk.passive_angles()
                print(
                    f"t={t:5.2f}  motor=({inner:+.3f},{outer:+.3f})  "
                    f"passive=({pi:+.3f},{po:+.3f})  "
                    f"foot=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  "
                    f"closure_err={err:.1e}"
                )
            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()
