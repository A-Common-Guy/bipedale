"""The 2026-11-12 sysid chirps as frequency-response tables (gyro, rolling, legs against wheel torque)."""
import json, sys
import numpy as np
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2] / "ros2_ws/src/sandro_bringup/tools"))
import sysid_yaw as s
d = str(__import__("pathlib").Path(__file__).resolve().parent / "sysid_20261112-225514")
plan = json.load(open(d + "/plan.json"))
joints, gyro, diag = s.read_bag(d + "/bag")

chirps = [e for e in plan["events"] if e["kind"] == "chirp"]
freqs = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 18, 20]
print("|H| gyro yaw rate per Nm of differential wheel torque (per wheel), phase deg; coherence")
print("f Hz  " + "  ".join(f"kp {e['kp']:g}".center(22) for e in chirps))
res = []
for e in chirps:
    _, torque, yaw, rolling, legs = s.signals(joints, gyro, e["t"], e["t"] + e["duration_s"])
    f, hy, coh = s.response(torque, yaw, segment=1024)
    _, hr, _ = s.response(torque, rolling, segment=1024)
    _, hl, _ = s.response(torque, legs, segment=1024)
    res.append((f, hy, coh, hr, hl))
for q in freqs:
    cells = []
    for f, hy, coh, hr, hl in res:
        i = np.argmin(abs(f - q))
        cells.append(f"{abs(hy[i]):5.2f} {np.degrees(np.angle(hy[i])):+5.0f} {coh[i]:4.2f}")
    print(f"{q:4d}  " + "  ".join(c.center(22) for c in cells))
print("\nrolling / gyro and legs / gyro (magnitude, phase) at each frequency, kp 12 first run")
f, hy, coh, hr, hl = res[0]
for q in freqs:
    i = np.argmin(abs(f - q))
    print(f"{q:4d} Hz  rolling/gyro {abs(hr[i] / hy[i]):5.2f} {np.degrees(np.angle(hr[i] / hy[i])):+5.0f}   "
          f"legs/gyro {abs(hl[i] / hy[i]):5.2f} {np.degrees(np.angle(hl[i] / hy[i])):+5.0f}")
