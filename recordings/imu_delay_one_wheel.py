"""tools/imu_delay.py analysis on one blocked wheel (the other free)."""
import csv
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/sandro_bringup/tools"))
from imu_delay import band_pass, fresh, pitch_from_quaternion  # noqa: E402

path, wheel = sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "right_wheel"
rows = list(csv.DictReader(open(path)))
num = lambda R, k: np.array([float(r[k]) if r[k] else np.nan for r in R])  # noqa: E731
J = [r for r in rows if r["stream"] == "joint"]
I = [r for r in rows if r["imu_qw"]]
tj, enc = fresh(num(J, "t_s"), num(J, f"pos_{wheel}"))
ti, q = fresh(num(I, "t_s"), np.stack([num(I, k) for k in ("imu_qx", "imu_qy", "imu_qz", "imu_qw",
                                                           "imu_gy")], axis=1))
pitch, gyro = pitch_from_quaternion(*q[:, :4].T), q[:, 4]
print(f"{len(tj) / (tj[-1] - tj[0]):.0f} Hz encoder, {len(ti) / (ti[-1] - ti[0]):.0f} Hz IMU, "
      f"pitch spread {np.ptp(pitch):.2f} rad, wheel spread {np.ptp(enc[:, 0]):.2f} rad")
rate = 1000.0
t = np.arange(max(tj[0], ti[0]) + 0.5, min(tj[-1], ti[-1]) - 0.5, 1.0 / rate)
e = np.interp(t, tj, enc[:, 0])
p = np.interp(t, ti, pitch)
g = np.interp(t, ti, gyro)
fit = np.polyfit(p - p.mean(), e - e.mean(), 1)[0]
print(f"wheel angle per body pitch: {fit:+.2f} (-1 for a blocked wheel)")
e_rate = np.gradient(e, t)
for name, imu_signal, encoder_signal in (("orientation", p, -e), ("gyro", g, -e_rate)):
    a = band_pass(imu_signal, rate, 0.5, 10.0)
    b = band_pass(encoder_signal, rate, 0.5, 10.0)
    lags = np.arange(-60, 61)
    n = len(a)
    c = np.array([np.dot(a[60 + L: n - 60 + L], b[60: n - 60]) for L in lags])
    c /= np.std(a) * np.std(b) * (n - 120)
    k = int(np.argmax(c))
    print(f"{name:12s} IMU behind encoder {lags[k]:+d} ms, corr {c[k]:.2f}")
