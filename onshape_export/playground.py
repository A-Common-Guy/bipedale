"""Interactive playground: MuJoCo viewer + a Python prompt to poke the robot.

    python playground.py                # FIXED base (robot on a stand): safe
                                        # bench for testing motors and sensors
    python playground.py --floating    # free base on the floor

The viewer opens and physics runs in real time in the background; you get a
Python prompt with an `r` object:

    r.read()                         all sensors, pretty-printed
    r.leg("right", inner=0.3)        MIT-mode target [rad] for one leg motor
    r.legs(inner=0.3, outer=-0.1)    same target(s) to both legs
    r.wheels(2)                      both wheels 2 rad/s (forward convention)
    r.wheels(right=1, left=-1)       spin in place
    r.stop()                         wheel targets to 0
    r.hold()                         all targets back to the keyframe pose
    r.reset()                        teleport back to the keyframe
    r.pause() / r.resume()           freeze / unfreeze physics
    r.obs()                          sensor dict (for scripting)
    r.pitch(), r.wheel_odometry()    balance-relevant quantities (floating)
    r.ctrl()                         current actuator commands

Everything you type talks to the same MjData the viewer renders -- what you
see is what the physics does.
"""
from __future__ import annotations

import argparse
import code
import threading
import time
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).parent
SIDES = ("right", "left")
LEG_JOINTS = ("motor_inner", "motor_outer")


def quat2mat(q):
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, q)
    return R.reshape(3, 3)


class Playground:
    def __init__(self, floating=False, gui=True):
        scene = HERE / "robot" / ("scene_floating.xml" if floating else "scene_full.xml")
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.floating = floating
        self.lock = threading.Lock()
        self.running = True
        self.paused = False

        m = self.model
        self._sens = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, s):
                      (m.sensor_adr[s], m.sensor_dim[s]) for s in range(m.nsensor)}
        self._act = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a): a
                     for a in range(m.nu)}
        self.reset()
        # reference frames + wheel mounting signs (see controller_demo.py)
        R0 = quat2mat(self.sensor("imu_quat"))
        self.up_b = R0.T @ [0, 0, 1]
        self.axle_b = R0.T @ [0, 1, 0]
        ax = {s: self.data.xaxis[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT,
                                                   f"{s}_wheel")].copy() for s in SIDES}
        self.wheel_dir = {s: (1.0 if ax[s] @ ax[SIDES[0]] > 0 else -1.0) for s in SIDES}

        self._viewer = None
        if gui:
            from mujoco import viewer as mj_viewer
            self._viewer = mj_viewer.launch_passive(self.model, self.data)
        self._thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------- lifecycle
    def _physics_loop(self):
        dt = self.model.opt.timestep
        while self.running and (self._viewer is None or self._viewer.is_running()):
            t0 = time.perf_counter()
            if not self.paused:
                with self.lock:
                    mujoco.mj_step(self.model, self.data)
            if self._viewer is not None:
                self._viewer.sync()
            time.sleep(max(0.0, dt - (time.perf_counter() - t0)))

    def pause(self):
        self.paused = True
        print("physics paused")

    def resume(self):
        self.paused = False
        print("physics running")

    def reset(self):
        with getattr(self, "lock", threading.Lock()):
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
            mujoco.mj_forward(self.model, self.data)
            self.home_ctrl = self.data.ctrl.copy()

    # ---------------------------------------------------------------- inputs
    def leg(self, side, inner=None, outer=None):
        """MIT-mode angle targets [rad] for one leg's motors."""
        with self.lock:
            if inner is not None:
                self.data.ctrl[self._act[f"{side}_motor_inner"]] = inner
            if outer is not None:
                self.data.ctrl[self._act[f"{side}_motor_outer"]] = outer

    def legs(self, inner=None, outer=None):
        for s in SIDES:
            self.leg(s, inner, outer)

    def wheels(self, speed=None, right=None, left=None):
        """Velocity targets [rad/s], forward convention (mirroring handled)."""
        right = speed if right is None else right
        left = speed if left is None else left
        with self.lock:
            if right is not None:
                self.data.ctrl[self._act["right_wheel"]] = self.wheel_dir["right"] * right
            if left is not None:
                self.data.ctrl[self._act["left_wheel"]] = self.wheel_dir["left"] * left

    def stop(self):
        self.wheels(0)

    def hold(self):
        with self.lock:
            self.data.ctrl[:] = self.home_ctrl

    # --------------------------------------------------------------- outputs
    def sensor(self, name):
        adr, dim = self._sens[name]
        return self.data.sensordata[adr:adr + dim].copy()

    def obs(self):
        return {n: self.sensor(n) for n in self._sens}

    def ctrl(self):
        return {n: float(self.data.ctrl[a]) for n, a in self._act.items()}

    def pitch(self):
        R = quat2mat(self.sensor("imu_quat"))
        up_w, axle_w = R @ self.up_b, R @ self.axle_b
        return float(np.arctan2(np.cross([0, 0, 1.0], up_w) @ axle_w, up_w[2]))

    def pitch_rate(self):
        return float(self.sensor("imu_gyro") @ self.axle_b)

    def wheel_odometry(self):
        pos = float(np.mean([self.wheel_dir[s] * self.sensor(f"{s}_wheel_pos")[0]
                             for s in SIDES])) + self.pitch()
        vel = float(np.mean([self.wheel_dir[s] * self.sensor(f"{s}_wheel_vel")[0]
                             for s in SIDES])) + self.pitch_rate()
        return pos, vel

    def read(self):
        print(f"{'sensor':24s} value")
        print(f"{'imu_quat':24s} {np.round(self.sensor('imu_quat'), 4)}")
        print(f"{'imu_gyro [rad/s]':24s} {np.round(self.sensor('imu_gyro'), 4)}")
        print(f"{'imu_accel [m/s^2]':24s} {np.round(self.sensor('imu_accel'), 3)}")
        for s in SIDES:
            for j in LEG_JOINTS + ("wheel",):
                p = self.sensor(f"{s}_{j}_pos")[0]
                v = self.sensor(f"{s}_{j}_vel")[0]
                print(f"{f'{s}_{j}':24s} pos {p:+8.3f} rad   vel {v:+8.3f} rad/s")
        if self.floating:
            print(f"{'pitch':24s} {np.degrees(self.pitch()):+.2f} deg "
                  f"(rate {self.pitch_rate():+.3f} rad/s)")
            pos, vel = self.wheel_odometry()
            print(f"{'odometry (ground)':24s} pos {pos:+.2f} rad   vel {vel:+.2f} rad/s")

    def help(self):
        print(__doc__)

    def close(self):
        self.running = False
        self._thread.join(timeout=2)
        if self._viewer is not None:
            self._viewer.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--floating", action="store_true",
                    help="free base on the floor (default: fixed base bench)")
    args = ap.parse_args()

    r = Playground(floating=args.floating)
    base = "FLOATING base on floor" if args.floating else "FIXED base (bench mode)"
    banner = (f"\nRobot playground -- {base}.\n"
              f"`r` is your robot. Try: r.read()   r.legs(inner=0.3)   "
              f"r.wheels(2)   r.hold()   r.help()\n")
    try:
        code.interact(banner=banner, local={"r": r, "np": np, "mujoco": mujoco},
                      exitmsg="closing playground")
    finally:
        r.close()


if __name__ == "__main__":
    main()
