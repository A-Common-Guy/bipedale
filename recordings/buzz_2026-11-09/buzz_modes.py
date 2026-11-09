"""Strongest 2-40 Hz peaks per joint and IMU axis, and how the two sides move at 5-15 Hz.

    python3 buzz_modes.py BAG
"""
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]  # buzz_spectrum.py, here or in recordings/
from buzz_spectrum import read, welch

joints, imu = read(sys.argv[1])
tj = np.array([s[0] for s in joints]); ti = np.array([s[0] for s in imu])
def sig(name, k): return np.array([s[1][name][k] for s in joints])
def peaks(label, t, x, lo=2.0, hi=40.0, n=3):
    f, p, fs = welch(t, x, segment=1024)
    band = (f >= lo) & (f < hi)
    order = np.argsort(p[band])[::-1]
    fb = f[band]; pb = p[band]
    picked = []
    for i in order:
        if all(abs(fb[i] - q) > 1.0 for q in picked):
            picked.append(fb[i])
        if len(picked) == n: break
    df = f[1] - f[0]
    print(f"{label:30s} " + "  ".join(f"{q:5.1f} Hz ({np.sqrt(pb[fb == q][0] * df * 3):.3f})" for q in picked))
print("strongest peaks 2-40 Hz (amplitude over ~3 bins)")
for name in ["right_wheel", "left_wheel", "right_motor_inner", "right_motor_outer", "left_motor_inner", "left_motor_outer"]:
    peaks(f"{name} velocity", tj, sig(name, 0))
for name in ["right_wheel", "left_wheel"]:
    peaks(f"{name} effort", tj, sig(name, 1))
for i, a in enumerate(["accel x", "accel y", "accel z", "gyro x (roll)", "gyro y (pitch)", "gyro z (yaw)"]):
    peaks(f"imu {a}", ti, [s[1][i] for s in imu])

# Side symmetry. The left side's joints turn the opposite way (mirrored motors).
def band(x, t, lo, hi):
    f = np.fft.rfftfreq(len(x), np.median(np.diff(t)))
    X = np.fft.rfft(x - x.mean()); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, len(x))
rw, lw = band(sig("right_wheel", 0), tj, 5, 15), band(sig("left_wheel", 0), tj, 5, 15)
print(f"\nwheels 5-15 Hz: correlation right vs left velocity {np.corrcoef(rw, lw)[0, 1]:+.2f}"
      " (+1: both driving forward together, -1: yaw)")
for joint in ["motor_inner", "motor_outer"]:
    r, l = band(sig("right_" + joint, 0), tj, 5, 15), band(sig("left_" + joint, 0), tj, 5, 15)
    print(f"{joint} 5-15 Hz: correlation right vs left velocity {np.corrcoef(r, l)[0, 1]:+.2f}")
print("\nreported velocity update rate (distinct consecutive values):")
for name in ["right_wheel", "right_motor_inner"]:
    v = sig(name, 0); changes = np.count_nonzero(np.diff(v)); print(f"  {name}: {changes / (tj[-1] - tj[0]):.0f} Hz")
