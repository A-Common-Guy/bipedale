"""Whether the 8-12 Hz wheel motion is yaw (wheels opposed) or pitch (together).

    python3 buzz_phase.py BAG
"""
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]  # buzz_spectrum.py, here or in recordings/
from buzz_spectrum import read

joints, imu = read(sys.argv[1])
tj = np.array([s[0] for s in joints]); ti = np.array([s[0] for s in imu])
grid = np.arange(max(tj[0], ti[0]), min(tj[-1], ti[-1]), 1 / 300)
def js(name, k): return np.interp(grid, tj, [s[1][name][k] for s in joints])
def im(i): return np.interp(grid, ti, [s[1][i] for s in imu])
def band(x, lo, hi):
    f = np.fft.rfftfreq(len(x), 1 / 300); X = np.fft.rfft(x - x.mean()); X[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(X, len(x))
rw, lw = js("right_wheel", 0), js("left_wheel", 0)
for lo, hi, label in [(1.0, 3.0, "1-3 Hz (forward rocking)"), (8.0, 12.0, "8-12 Hz (the oscillation)")]:
    r, l = band(rw, lo, hi), band(lw, lo, hi)
    c = np.corrcoef(r, l)[0, 1]
    pitch, yaw = band(im(4), lo, hi), band(im(5), lo, hi)
    print(f"{label:27s} right/left wheel velocity correlation {c:+.2f};  "
          f"pitch rate RMS {pitch.std():.3f}, yaw rate RMS {yaw.std():.3f} rad/s")
# Which wheel combination follows the yaw rate at 8-12 Hz.
r, l, yaw, pitch = band(rw, 8, 12), band(lw, 8, 12), band(im(5), 8, 12), band(im(4), 8, 12)
for name, combo in [("right + left", r + l), ("right - left", r - l)]:
    print(f"8-12 Hz {name:12s}: RMS {combo.std():.3f} rad/s, correlation with yaw rate {np.corrcoef(combo, yaw)[0, 1]:+.2f}, "
          f"with pitch rate {np.corrcoef(combo, pitch)[0, 1]:+.2f}")
