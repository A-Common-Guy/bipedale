"""The 2026-11-15 right-wheel drive_train_test runs (rim blocked): the tool's own
summary, the triangles cycle by cycle, and the chirp as a frequency-response
table from commanded torque to motor angle (output-side), with the inertia
read off the roll-off, 1 / (|H| w^2), where a mass dominates."""
import csv, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "ros2_ws/src/sandro_bringup/tools"))
import drive_train_test as dtt

HERE = Path(__file__).resolve().parent
FREQS = [5, 7, 9, 11, 13, 16, 20, 25, 30, 35, 40]
MASS_LINE = (20, 40)


def load(path):
    with open(path) as f:
        head = next(csv.reader(f))
        meta = dict(zip((k.lstrip("# ") for k in head[0::2]), head[1::2]))
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    return meta, col("t_s"), col("position_rad"), col("command_nm")


def chirp_response(meta, t, q, u):
    """|H| (rad/Nm) and phase (deg) at FREQS: a sinusoid at the chirp's own
    phase fitted to torque and angle over 0.7 s around each frequency."""
    start = float(meta["period_s"]) * float(meta["cycles"]) + float(meta["sine_s"])
    duration = float(meta["chirp_s"])
    (f0, f1), out = dtt.CHIRP_HZ, []
    grid = np.arange(start + 0.6, start + duration - 0.6, 1 / 300.0)
    qg, ug = np.interp(grid, t, q), np.interp(grid, t, u)
    for fq in FREQS:
        tc = start + (fq - f0) / (f1 - f0) * duration
        w = np.abs(grid - tc) < 0.35
        tt = grid[w] - start
        phase = 2 * np.pi * (f0 * tt + 0.5 * (f1 - f0) / duration * tt * tt)
        basis = np.stack([np.sin(phase), np.cos(phase), np.ones_like(tt), tt - tt.mean()], 1)
        cu = np.linalg.lstsq(basis, ug[w], rcond=None)[0]
        cq = np.linalg.lstsq(basis, qg[w], rcond=None)[0]
        r2 = 1 - np.var(qg[w] - basis @ cq) / np.var(qg[w])
        lag = math.degrees(math.atan2(cq[1], cq[0]) - math.atan2(cu[1], cu[0]))
        out.append((fq, np.hypot(*cq[:2]) / np.hypot(*cu[:2]), (lag + 180) % 360 - 180, r2))
    return out


for name in ("right_gear.csv", "right_gear_bias.csv"):
    meta, t, q, u = load(HERE / name)
    print(f"===== {name}: {len(t) / t[-1]:.0f} Hz recorded, chirp {meta['chirp_nm']} Nm "
          f"on {meta.get('chirp_bias_nm', '0')} Nm")
    dtt.analyse(str(HERE / name))
    print("  triangles: angle at +-0.40..0.45 Nm, swing, angle where the torque crosses zero")
    for k in range(int(meta["cycles"])):
        period = float(meta["period_s"])
        m = (t >= k * period) & (t < (k + 1) * period)
        hi, lo = q[m & (u > 0.4)].mean(), q[m & (u < -0.4)].mean()
        zero = q[m][np.flatnonzero(np.diff(np.sign(u[m])) != 0)]
        print(f"    {k}: {hi:.4f} / {lo:.4f} rad, swing {(hi - lo) * 1e3:.0f} mrad, zero torque at "
              + ", ".join(f"{z:.3f}" for z in zero))
    rows = chirp_response(meta, t, q, u)
    print("  chirp: f Hz, |H| mrad/Nm, phase deg, fit R2, 1/(|H| w^2) kg m2")
    for f, h, p, r2 in rows:
        print(f"    {f:4d}  {h * 1e3:6.1f}  {p:+5.0f}  {r2:4.2f}  {1 / (h * (2 * np.pi * f) ** 2):.4f}")
    j_line = np.mean([1 / (h * (2 * np.pi * f) ** 2) for f, h, _, _ in rows if MASS_LINE[0] <= f <= MASS_LINE[1]])
    # A rotor on a spring and damper behind the pipeline's delay (command logged
    # at publication, applied the next cycle, angle from a later MIT reply):
    # H = exp(-j w tau) / (k - J w^2 + j c w). For each tau, k, J and c are a
    # linear fit of exp(-j w tau) / H, weighted by |H| (relative error).
    w = 2 * np.pi * np.array([f for f, *_ in rows])
    h = np.array([m * np.exp(1j * math.radians(p)) for _, m, p, _ in rows])
    best = None
    for tau in np.arange(0.0, 0.0151, 0.0001):
        g = np.exp(-1j * w * tau) / h
        a = np.concatenate([np.stack([np.ones_like(w), -w ** 2, 0 * w], 1),
                            np.stack([0 * w, 0 * w, w], 1)]) * np.tile(abs(h), 2)[:, None]
        b = np.concatenate([g.real, g.imag]) * np.tile(abs(h), 2)
        x, *_ = np.linalg.lstsq(a, b, rcond=None)
        err = float(np.sum((a @ x - b) ** 2))
        if best is None or err < best[0]:
            best = (err, tau, *x)
    err, tau, k, j, c = best
    print(f"  inertia from the roll-off ({MASS_LINE[0]}-{MASS_LINE[1]} Hz) {j_line:.4f} kg m2 "
          f"(MJCF armature 0.000168, whole wheel 0.00047)")
    print(f"  fit: J {j:.4f} kg m2, k {k:.1f} Nm/rad, c {c:.3f} Nm s/rad, delay {tau * 1e3:.1f} ms; "
          f"natural frequency {math.sqrt(k / j) / (2 * np.pi):.1f} Hz, zeta {c / (2 * math.sqrt(k * j)):.2f}, "
          f"relative residual {math.sqrt(err / len(w)):.2f}")
