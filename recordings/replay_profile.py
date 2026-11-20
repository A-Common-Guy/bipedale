"""What the camera saw in a recording, through sandro_perception's height_profile.py again.

policy_record.py's bags hold /height_profile/depth_preview (every few depth images at a
quarter of the resolution), the joint states and the IMU: enough to rebuild the terrain
profile offline, with the node's settings or others, and to set it beside the
/terrain_profile the robot published. Use it to see why a profile read what it read, and to
try a change to height_profile.py on real images before it goes on the robot.

    sandro_rl/.venv/bin/python recordings/replay_profile.py recordings/policy_trials/<run>
        [--set cell_m=0.04 spread=1.5 ...]     Settings fields of height_profile.py
        [--images OUT.npz]                     the preview images, stamps and intrinsics

The preview holds a quarter of the pixels the node used, so cells far ahead get fewer
points than on the robot: the replay counts a cell from 1 point (the node needs 3 of 4
times as many). Prints, per second: the command, and for each wheel track the replayed
and the recorded profile in cm ('.' unknown).
"""
import argparse
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "sandro_perception"))
from sandro_perception.height_profile import HeightProfile, Settings   # noqa: E402

LEGS = ("right_motor_inner", "right_motor_outer", "left_motor_inner", "left_motor_outer")
STATE_HZ = 50.0


def load(bag: Path):
    """(states, images, profiles, commands) of a bag: states (t, legs[4], wheels[2], quaternion
    xyzw, pitch rate), images (t, depth m [h, w], (fx, fy, cx, cy)), profiles (t, 32 values),
    commands (t, forward, yaw). Times are the messages' stamps where they have one."""
    from mcap_ros2.reader import read_ros2_messages
    stamp = lambda h: h.stamp.sec + 1e-9 * h.stamp.nanosec
    states, images, profiles, commands, imu = [], [], [], [], None
    offset = None                                   # log time minus stamps, for the unstamped topics
    for m in read_ros2_messages(str(bag), topics=["/joint_states", "/imu_broadcaster/imu", "/terrain_profile",
                                                  "/height_profile/depth_preview", "/cmd_vel"]):
        r, topic = m.ros_msg, m.channel.topic
        log = m.log_time.timestamp() if hasattr(m.log_time, "timestamp") else m.log_time_ns * 1e-9
        if topic == "/imu_broadcaster/imu":
            q = r.orientation
            imu = ((q.x, q.y, q.z, q.w), r.angular_velocity.y)
        elif topic == "/joint_states":
            t = stamp(r.header)
            offset = log - t
            if imu is None or (states and t - states[-1][0] < 1.0 / STATE_HZ - 1e-4):
                continue
            position, velocity = dict(zip(r.name, r.position)), dict(zip(r.name, r.velocity))
            states.append((t, [position[n] for n in LEGS], (velocity["right_wheel"], velocity["left_wheel"]), *imu))
        elif topic == "/height_profile/depth_preview":
            depth = np.frombuffer(bytes(r.data), dtype="<u2").reshape(r.height, r.step // 2)[:, :r.width] * 1e-3
            images.append((stamp(r.header), depth.astype(np.float32), tuple(float(v) for v in r.header.frame_id.split())))
        elif offset is not None and topic == "/terrain_profile":
            profiles.append((log - offset, np.asarray(r.data, float)))
        elif offset is not None:
            commands.append((log - offset, r.twist.linear.x, r.twist.angular.z))
    return states, images, profiles, commands


def replay(states, images, settings: Settings):
    """[(t, heights [2, k], known [2, k])] at each state sample, the images added when the
    state has passed their stamp."""
    profile, out, pending = HeightProfile(settings), [], list(images)
    for t, legs, wheels, quaternion, pitch_rate in states:
        profile.update_state(t, dict(zip(LEGS, legs)), wheels, quaternion, pitch_rate)
        while pending and pending[0][0] <= t:
            stamp, depth, k = pending.pop(0)
            profile.add_depth(stamp, depth, *k)
        out.append((t, *profile.profile()))
    return out


def row(values, known):
    return " ".join(f"{100 * v:4.0f}" if k else "   ." for v, k in zip(values, known))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", type=Path, help="a policy_record.py run folder, or an .mcap file")
    parser.add_argument("--set", nargs="*", default=[], metavar="FIELD=VALUE")
    parser.add_argument("--images", type=Path)
    args = parser.parse_args()
    bag = args.run if args.run.suffix == ".mcap" else next((args.run / "bag").glob("*.mcap"))
    states, images, profiles, commands = load(bag)
    if not images:
        print(f"{bag}: no /height_profile/depth_preview (recorded before the node published it, or camera off)")
        return 1
    settings = replace(Settings(), stride=1, min_points=1)
    for item in args.set:
        name, value = item.split("=")
        settings = replace(settings, **{name: type(getattr(settings, name))(float(value))})
    if args.images:
        np.savez_compressed(args.images, stamps=[t for t, *_ in images], depth=np.stack([d for _, d, _ in images]),
                            intrinsics=images[0][2])
    replayed = replay(states, images, settings)
    t0 = states[0][0]
    n = len(settings.layout.distances_m)
    print(f"{len(images)} images, {len(states)} states over {states[-1][0] - t0:.0f} s; per track: "
          f"{' '.join(f'{d:g}' for d in settings.layout.distances_m)} m ahead")
    print("   t   cmd m/s rad/s | right: replayed / recorded | left: replayed / recorded")
    recorded_t = np.array([t for t, _ in profiles]) if profiles else np.zeros(0)
    command_t = np.array([t for t, *_ in commands]) if commands else np.zeros(0)
    for t, heights, known in replayed[::int(STATE_HZ)]:
        line = f"{t - t0:5.0f}"
        if len(command_t):
            c = commands[max(0, int(np.searchsorted(command_t, t)) - 1)]
            line += f"  {c[1]:5.2f} {c[2]:5.2f}"
        recorded = profiles[min(len(profiles) - 1, int(np.searchsorted(recorded_t, t)))][1] if profiles else None
        for w in range(2):
            line += " | " + row(heights[w], known[w])
            if recorded is not None:
                line += "  /  " + row(recorded[w * n:(w + 1) * n], recorded[(2 + w) * n:(3 + w) * n] > 0.5)
        print(line)
    return 0


if __name__ == "__main__":
    sys.exit(main())
