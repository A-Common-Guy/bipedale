"""Controller playground for the composed wheeled-biped model.

Demonstrates the canonical sim control loop:

  physics at 1/model.timestep Hz (500 Hz default)
      |  every DECIMATION steps
  controller at CONTROL_HZ, reading ONLY what the real robot will have
      (BNO085 quat/gyro + motor encoders) and writing ONLY what the real
      CAN bus carries (leg MIT-mode angle targets, wheel speed targets)

Run:
    python controller_demo.py                # hold pose, wheels at 0 rad/s
    python controller_demo.py --balance      # pitch PD -> wheel speed
    python controller_demo.py --balance --drive 3   # ...while driving 3 rad/s
    python controller_demo.py --headless 5   # no viewer, 5 s telemetry
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import numpy as np

SCENE = Path(__file__).parent / "robot" / "scene_floating.xml"

CONTROL_HZ = 200          # controller rate (real firmware loop rate)
LEG_JOINTS = ("motor_inner", "motor_outer")
SIDES = ("right", "left")

# inner loop: wheel speed [rad/s] = KP*(pitch - pitch_ref) + KD*pitchrate
BAL_KP = 10.0
BAL_KD = 2.0
# outer loop: pitch_ref [rad] leans the robot against wheel drift
#   pitch_ref = OUT_KV*(v_cmd - v) - OUT_KP*wheel_pos_error, clamped
OUT_KV = 0.02
OUT_KP = 0.01
REF_CLAMP = 0.06          # rad, max commanded lean
ACCEL = 1.5               # rad/s^2, slew limit on the wheel speed command
WHEEL_SIGN = +1.0         # flips if wheels are mounted mirrored to expectation
SETTLE_TIME = 1.0         # s: hold still after spawn before balance engages


def quat2mat(q):
    R = np.empty(9)
    mujoco.mju_quat2Mat(R, q)
    return R.reshape(3, 3)


class Robot:
    """The sim seen through the real robot's interface: sensors in, targets out."""

    def __init__(self, xml_path=SCENE):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.decimation = max(1, round(1 / (CONTROL_HZ * self.model.opt.timestep)))
        m = self.model
        self._sens = {}
        for s in range(m.nsensor):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, s)
            self._sens[name] = (m.sensor_adr[s], m.sensor_dim[s])
        self._act = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a): a
                     for a in range(m.nu)}
        self.reset()
        # body-frame reference axes captured at the spawn pose, so pitch is
        # measured about the wheel axle regardless of the CAD frame orientation
        R0 = quat2mat(self.sensor("imu_quat"))
        self.up_b = R0.T @ [0, 0, 1]      # body direction that points up at spawn
        self.axle_b = R0.T @ [0, 1, 0]    # body direction along the wheel axle
        # the mirrored leg mounts its wheel with the axis flipped: same-sign
        # commands would yaw in place. On the real robot this is a wiring
        # constant; here we read it from the model once.
        ax = {}
        for s in SIDES:
            j = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{s}_wheel")
            ax[s] = self.data.xaxis[j].copy()
        self.wheel_dir = {s: (1.0 if ax[s] @ ax[SIDES[0]] > 0 else -1.0) for s in SIDES}

    def reset(self):
        kid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "stand")
        mujoco.mj_resetDataKeyframe(self.model, self.data, kid)
        mujoco.mj_forward(self.model, self.data)
        self.stand_ctrl = self.data.ctrl.copy()

    # ---- observations (only what the real robot can measure) ----
    def sensor(self, name):
        adr, dim = self._sens[name]
        return self.data.sensordata[adr:adr + dim]

    def pitch(self) -> float:
        """Lean about the CURRENT axle direction (yaw-invariant): 0 upright,
        >0 falling toward the robot's own forward."""
        R = quat2mat(self.sensor("imu_quat"))
        up_w = R @ self.up_b
        axle_w = R @ self.axle_b
        return float(np.arctan2(np.cross([0, 0, 1.0], up_w) @ axle_w, up_w[2]))

    def pitch_rate(self) -> float:
        return float(self.sensor("imu_gyro") @ self.axle_b)

    def wheel_speeds(self):
        """Wheel speeds in the common forward convention."""
        return np.array([self.wheel_dir[s] * self.sensor(f"{s}_wheel_vel")[0]
                         for s in SIDES])

    def wheel_odometry(self):
        """(ground angle, ground speed), forward convention.

        Encoders measure wheel-vs-BODY rotation; rolling distance over the
        ground is wheel rotation PLUS body pitch, so add the IMU pitch. Without
        this the controller is blind to motion made by leaning."""
        pos = float(np.mean([self.wheel_dir[s] * self.sensor(f"{s}_wheel_pos")[0]
                             for s in SIDES])) + self.pitch()
        vel = float(np.mean(self.wheel_speeds())) + self.pitch_rate()
        return pos, vel

    def leg_angles(self):
        return {f"{s}_{j}": float(self.sensor(f"{s}_{j}_pos")[0])
                for s in SIDES for j in LEG_JOINTS}

    # ---- commands (only what the real CAN bus carries) ----
    def set_leg_target(self, side, joint, angle):
        self.data.ctrl[self._act[f"{side}_{joint}"]] = angle

    def set_wheel_speed(self, right, left):
        """Forward convention: positive = both wheels drive the robot forward."""
        self.data.ctrl[self._act["right_wheel"]] = self.wheel_dir["right"] * right
        self.data.ctrl[self._act["left_wheel"]] = self.wheel_dir["left"] * left

    def step_control_period(self):
        for _ in range(self.decimation):
            mujoco.mj_step(self.model, self.data)


def controller(robot: Robot, t: float, args) -> None:
    """One control tick. Replace/extend this -- everything else is plumbing."""
    # legs: hold the spawn posture (later: IK from a height/lean planner)
    for a, val in enumerate(robot.stand_ctrl):
        name = mujoco.mj_id2name(robot.model, mujoco.mjtObj.mjOBJ_ACTUATOR, a)
        if not name.endswith("_wheel"):
            robot.data.ctrl[a] = val

    if t < SETTLE_TIME:  # let the spawn transient die out first
        robot.set_wheel_speed(0, 0)
        return

    if args.balance:
        # outer loop: hold station (or the commanded speed) by leaning into drift
        pos, vel = robot.wheel_odometry()
        if not hasattr(robot, "pos_ref"):
            robot.pos_ref, robot.v_cmd = pos, 0.0
        # ramp toward the commanded speed instead of stepping to it
        robot.v_cmd += np.clip(args.drive - robot.v_cmd, -ACCEL / CONTROL_HZ,
                               ACCEL / CONTROL_HZ)
        robot.pos_ref += robot.v_cmd / CONTROL_HZ
        # lean into velocity/station error (now visible thanks to corrected odometry)
        pitch_ref = np.clip(OUT_KV * (robot.v_cmd - vel) - OUT_KP * (pos - robot.pos_ref),
                            -REF_CLAMP, REF_CLAMP)
        # wheels: speed feedforward + pitch correction
        w = robot.v_cmd + WHEEL_SIGN * (BAL_KP * (robot.pitch() - pitch_ref)
                                        + BAL_KD * robot.pitch_rate())
        robot.set_wheel_speed(w + args.turn, w - args.turn)
    else:
        if not hasattr(robot, "v_cmd"):
            robot.v_cmd = 0.0
        robot.v_cmd += np.clip(args.drive - robot.v_cmd, -ACCEL / CONTROL_HZ,
                               ACCEL / CONTROL_HZ)
        robot.set_wheel_speed(robot.v_cmd, robot.v_cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--balance", action="store_true", help="pitch PD on the wheels")
    ap.add_argument("--drive", type=float, default=0.0, help="wheel speed offset, rad/s")
    ap.add_argument("--turn", type=float, default=0.0, help="differential speed, rad/s")
    ap.add_argument("--headless", type=float, default=0.0, metavar="SECONDS",
                    help="run without viewer for SECONDS, print telemetry")
    args = ap.parse_args()

    robot = Robot()
    print(f"physics {1/robot.model.opt.timestep:.0f} Hz, control {CONTROL_HZ} Hz "
          f"(decimation {robot.decimation})")

    if args.headless:
        n = int(args.headless * CONTROL_HZ)
        for i in range(n):
            controller(robot, i / CONTROL_HZ, args)
            robot.step_control_period()
            if i % CONTROL_HZ == 0:
                print(f"t={i/CONTROL_HZ:4.1f}s pitch={np.degrees(robot.pitch()):+6.2f} deg "
                      f"rate={robot.pitch_rate():+6.3f}  wheels={np.round(robot.wheel_speeds(), 2)}")
        print(f"final pitch: {np.degrees(robot.pitch()):+.2f} deg")
        return

    import mujoco.viewer
    with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
        t0 = time.time()
        while viewer.is_running():
            t = time.time() - t0
            controller(robot, t, args)
            robot.step_control_period()
            viewer.sync()
            time.sleep(max(0, robot.decimation * robot.model.opt.timestep - 0.001))


if __name__ == "__main__":
    main()
