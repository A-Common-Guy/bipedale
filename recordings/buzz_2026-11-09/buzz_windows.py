"""The three policies of buzz_rl_flat_level_fast_mit, each in its own window (epoch s from
the 22:20 UTC run log): 5-15 Hz RMS per joint and IMU axis, peaks, and whether the two
sides move together or opposed, calibrated on the 0.5-3 Hz forward rocking.

    python3 buzz_windows.py buzz_rl_flat_level_fast_mit
"""
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]  # buzz_spectrum.py, here or in recordings/
from buzz_spectrum import read

bag = sys.argv[1]
windows = {"rl_flat_level_fast": (1794263341.7, 1794263343.4),
           "rl_flat_level": (1794263350.0, 1794263365.9),
           "rl_agile_terrain": (1794263366.5, 1794263381.5)}
joints, imu = read(bag)
tj = np.array([s[0] for s in joints]); ti = np.array([s[0] for s in imu])
names = ["right_motor_inner", "right_motor_outer", "right_wheel", "left_motor_inner", "left_motor_outer", "left_wheel"]
V = {n: np.array([s[1][n][0] for s in joints]) for n in names}
E = {n: np.array([s[1][n][1] for s in joints]) for n in names}
G = np.array([s[1][3:6] for s in imu]); A = np.array([s[1][0:3] for s in imu])

def bandpass(x, lo, hi, fs=300.0):
    f = np.fft.rfftfreq(len(x), 1 / fs); X = np.fft.rfft(x - x.mean()); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, len(x))
def peak(x, lo=5.0, hi=15.0, fs=300.0):
    w = np.hanning(len(x)); X = np.abs(np.fft.rfft((x - x.mean()) * w)); f = np.fft.rfftfreq(len(x), 1 / fs)
    b = (f >= lo) & (f <= hi); return f[b][np.argmax(X[b])]

for policy, (t0, t1) in windows.items():
    grid = np.arange(t0, t1, 1 / 300.0)
    v = {n: np.interp(grid, tj, V[n]) for n in names}
    e = {n: np.interp(grid, tj, E[n]) for n in names}
    g = [np.interp(grid, ti, G[:, k]) for k in range(3)]
    a = [np.interp(grid, ti, A[:, k]) for k in range(3)]
    print(f"\n=== {policy}: {t1 - t0:.1f} s")
    print("5-15 Hz RMS   velocity rad/s: " + "  ".join(f"{n.replace('_motor', '')} {bandpass(v[n], 5, 15).std():.2f}" for n in names))
    print("              effort Nm:      " + "  ".join(f"{n.replace('_motor', '')} {bandpass(e[n], 5, 15).std():.2f}" for n in names))
    print(f"              gyro roll {bandpass(g[0], 5, 15).std():.3f}  pitch {bandpass(g[1], 5, 15).std():.3f}  yaw {bandpass(g[2], 5, 15).std():.3f} rad/s;"
          f"  accel x {bandpass(a[0], 5, 15).std():.2f}  y {bandpass(a[1], 5, 15).std():.2f}  z {bandpass(a[2], 5, 15).std():.2f} m/s2")
    print("peak 5-15 Hz  " + "  ".join(f"{n.replace('_motor', '')} {peak(v[n]):.1f}" for n in names)
          + f"  | gyro roll {peak(g[0]):.1f} pitch {peak(g[1]):.1f} yaw {peak(g[2]):.1f}")
    for pair in [("right_wheel", "left_wheel"), ("right_motor_inner", "left_motor_inner"), ("right_motor_outer", "left_motor_outer")]:
        r, l = v[pair[0]], v[pair[1]]
        slow = np.corrcoef(bandpass(r, 0.5, 3), bandpass(l, 0.5, 3))[0, 1]
        fast = np.corrcoef(bandpass(r, 5, 15), bandpass(l, 5, 15))[0, 1]
        s = np.sign(slow) if abs(slow) > 0.5 else 1.0
        together = bandpass(r + s * l, 5, 15).std() / 2; opposed = bandpass(r - s * l, 5, 15).std() / 2
        print(f"{pair[0].replace('right_', ''):12s} right/left correlation 0.5-3 Hz {slow:+.2f}, 5-15 Hz {fast:+.2f};"
              f"  5-15 Hz together {together:.2f}, opposed {opposed:.2f} rad/s")
    # Same side: inner against outer (height: opposite turns, fore-aft: the same).
    for side in ("right", "left"):
        c = np.corrcoef(bandpass(v[f"{side}_motor_inner"], 5, 15), bandpass(v[f"{side}_motor_outer"], 5, 15))[0, 1]
        print(f"{side} inner vs outer 5-15 Hz correlation {c:+.2f}", end="   ")
    print()
