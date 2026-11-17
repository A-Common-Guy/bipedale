#!/usr/bin/env python3
"""Bode plot and resonances of the robot's yaw, pitch or roll response, from a sysid recording.

    python3 bode.py DIR                          # the chirps of a sysid_yaw.py recording
    python3 bode.py DIR --axis yaw --png bode.png

The WBC adds a chirp r(t) (sysid.* parameters, sysid_yaw.py) to an actuator
channel while it balances, and the gyro answers on the matching axis:

    yaw    opposed wheel torque (right - left) / 2, Nm per wheel  -> gyro yaw rate
    pitch  wheel torque together (right + left) / 2, Nm per wheel -> gyro pitch rate
    roll   opposed leg torque ((RI + RO) - (LI + LO)) / 4, Nm per joint -> gyro roll rate

The robot is in closed loop: the controller's own reaction is in the measured
torque u. Dividing by it (H = Puy / Puu, as sysid_yaw.py does) is right only
while that reaction is small against the chirp; the WBC's yaw loop is
(0.15 Nm per rad/s), its balance loop in pitch is not. The chirp itself is
uncorrelated with the controller's reaction, so H = Pry / Pru, the response
to the torque the chirp caused, holds either way. Both are given; where they
part, the loop is biasing the direct one.

The chirp is rebuilt from plan.json with the WBC's formula, and its start
found by correlating it with the measured torque (plan.json logs when the
parameter was set, not when the controller started the sweep).

The resonance is read where the chirp explains the gyro (coherence 0.7 or
more) in two ways: a fit of k e^(-s tau) s^p wn^2 / (s^2 + 2 zeta wn s + wn^2)
(p per axis, --slope), which finds a mode however damped, and the peaks of
|H / s^p| with their half-power damping, which only a lightly damped mode
makes. Writes DIR/bode_<axis>.json; --png draws the plot (needs matplotlib).
"""

import argparse
import json
import math
import sys
from pathlib import Path

FS = 300.0
JOINTS = ("right_motor_inner", "right_motor_outer", "right_wheel",
          "left_motor_inner", "left_motor_outer", "left_wheel")
TAPER_S = 0.5                 # whole_body_core.cpp sysid_excitation_nm
MIN_COHERENCE = 0.7


def _effort(e, name):
    return e[:, JOINTS.index(name)]


# axis: (input, its unit, input from the efforts (n, 6), gyro column x/y/z, output, slope).
# slope p: the response away from the resonance goes as s^p. -1: a rigid body
# turned through a compliance (yaw, as sysid_yaw.yaw_model_fit; pitch above the
# balance dynamics); +1: the rate of a body held by a spring (roll on the legs,
# a guess until a roll recording says otherwise).
AXES = {
    "yaw": ("opposed wheel torque", "Nm per wheel",
            lambda e: 0.5 * (_effort(e, "right_wheel") - _effort(e, "left_wheel")), 2, "gyro yaw rate", -1),
    "pitch": ("wheel torque together", "Nm per wheel",
              lambda e: 0.5 * (_effort(e, "right_wheel") + _effort(e, "left_wheel")), 1, "gyro pitch rate", -1),
    "roll": ("opposed leg torque", "Nm per joint",
             lambda e: (_effort(e, "right_motor_inner") + _effort(e, "right_motor_outer")
                        - _effort(e, "left_motor_inner") - _effort(e, "left_motor_outer")) / 4,
             0, "gyro roll rate", 1),
}


def read_bag(path):
    """(t, efforts in JOINTS order) and (t, gyro x, y, z), by header stamp."""
    import numpy as np
    joints, gyro = [], []

    def take(topic, msg, received_ns):
        s = msg.header.stamp
        t = s.sec + s.nanosec * 1e-9 if (s.sec or s.nanosec) else received_ns * 1e-9
        if topic == "/joint_states":
            joints.append([t] + [msg.effort[msg.name.index(n)] for n in JOINTS])
        else:
            g = msg.angular_velocity
            gyro.append((t, g.x, g.y, g.z))

    wanted = {"/joint_states", "/imu_broadcaster/imu"}
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
        reader = SequentialReader()
        reader.open(StorageOptions(uri=str(path), storage_id="mcap"), ConverterOptions("", ""))
        types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, data, received = reader.read_next()
            if topic in wanted:
                take(topic, deserialize_message(data, get_message(types[topic])), received)
    except ImportError:
        from mcap_ros2.reader import read_ros2_messages
        for file in sorted(Path(path).glob("*.mcap")):
            for m in read_ros2_messages(str(file), topics=sorted(wanted)):
                take(m.channel.topic, m.ros_msg, m.log_time_ns)
    return np.array(joints), np.array(gyro)


def chirp(t, f0, f1, duration, amplitude=1.0):
    """The WBC's chirp at t seconds after its start (0 outside the sweep)."""
    import numpy as np
    t = np.asarray(t, dtype=float)
    rate = (f1 - f0) / duration
    edge = np.clip(np.minimum(t, duration - t) / TAPER_S, 0.0, 1.0)
    r = amplitude * 0.5 * (1.0 - np.cos(np.pi * edge)) * np.sin(2 * np.pi * (f0 * t + 0.5 * rate * t * t))
    return np.where((t >= 0) & (t <= duration), r, 0.0)


def align(grid, u, event, search=(-0.5, 1.0)):
    """Seconds from the event's logged time to the chirp's start: the shift that
    best correlates the rebuilt chirp with the measured input u on grid."""
    import numpy as np
    u = u - u.mean()
    best, best_offset = -1.0, 0.0
    for offset in np.arange(search[0], search[1], 1 / FS):
        r = chirp(grid - event["t"] - offset, event["f0_hz"], event["f1_hz"], event["duration_s"])
        c = abs(r @ u) / (np.linalg.norm(r) * np.linalg.norm(u) + 1e-12)
        if c > best:
            best, best_offset = c, offset
    return best_offset, best


def spectra(r, u, y, segment=1024):
    """Welch sums (Hann, half overlap) of the auto and cross spectra of r, u, y:
    sums, so several chirps add up before the ratios are taken."""
    import numpy as np
    window = np.hanning(segment)
    s = {k: 0 for k in ("rr", "ru", "ry", "uu", "uy", "yy")}
    for start in range(0, len(r) - segment + 1, segment // 2):
        R, U, Y = (np.fft.rfft((x[start:start + segment] - x[start:start + segment].mean()) * window)
                   for x in (r, u, y))
        s["rr"] = s["rr"] + abs(R) ** 2
        s["ru"] = s["ru"] + np.conj(R) * U
        s["ry"] = s["ry"] + np.conj(R) * Y
        s["uu"] = s["uu"] + abs(U) ** 2
        s["uy"] = s["uy"] + np.conj(U) * Y
        s["yy"] = s["yy"] + abs(Y) ** 2
    s["f"] = np.fft.rfftfreq(segment, 1 / FS)
    return s


def add(a, b):
    return b if a is None else {k: (a[k] if k == "f" else a[k] + b[k]) for k in a}


def frequency_response(s):
    """(f, H through the chirp Pry / Pru, H direct Puy / Puu, coherence of the gyro with the chirp)."""
    import numpy as np
    with np.errstate(divide="ignore", invalid="ignore"):   # bins the chirp never reaches
        return (s["f"], s["ry"] / s["ru"], s["uy"] / s["uu"], abs(s["ry"]) ** 2 / (s["rr"] * s["yy"]))


def trusted(f, h, coherence, band, min_coherence=MIN_COHERENCE):
    """Indices of the longest run of bins inside band where H is finite and the
    chirp explains the gyro: resonances are read only where the data is continuous."""
    import numpy as np
    idx = np.flatnonzero((f >= band[0]) & (f <= band[1]) & np.isfinite(h) & (coherence >= min_coherence))
    if len(idx) < 3:
        return idx[:0]
    return max(np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1), key=len)


def resonances(f, h, coherence, band, slope=0, prominence_db=3.0, min_coherence=MIN_COHERENCE):
    """Peaks of |H / s^slope| (the slope taken out) on the trusted bins:
    [{f_hz, gain, phase_deg, prominence_db, zeta}], strongest first. zeta from
    the half-power (-3 dB) width, nan when a side leaves the trusted bins. A
    resonance damped past zeta ~0.4 makes no peak: fit_mode finds it anyway."""
    import numpy as np
    idx = trusted(f, h, coherence, band, min_coherence)
    if len(idx) < 3:
        return []
    fk, hk = f[idx], h[idx]
    mag = 20 * np.log10(abs(hk / (2j * np.pi * fk) ** slope))
    found = []
    for i in range(1, len(mag) - 1):
        if not (mag[i] > mag[i - 1] and mag[i] >= mag[i + 1]):
            continue
        left = mag[:i + 1][::-1]
        higher = np.flatnonzero(left > mag[i])
        left_base = left[:higher[0]].min() if len(higher) else left.min()
        right = mag[i:]
        higher = np.flatnonzero(right > mag[i])
        right_base = right[:higher[0]].min() if len(higher) else right.min()
        prominence = mag[i] - max(left_base, right_base)
        if prominence < prominence_db:
            continue
        half = mag[i] - 3.0
        lo = next((k for k in range(i, -1, -1) if mag[k] <= half), None)
        hi = next((k for k in range(i, len(mag)) if mag[k] <= half), None)
        zeta = math.nan
        if lo is not None and hi is not None:
            f_lo = np.interp(half, [mag[lo], mag[lo + 1]], [fk[lo], fk[lo + 1]])
            f_hi = np.interp(half, [mag[hi], mag[hi - 1]], [fk[hi], fk[hi - 1]])
            zeta = float((f_hi - f_lo) / (2 * fk[i]))
        found.append({"f_hz": float(fk[i]), "gain": float(abs(hk[i])),
                      "phase_deg": float(np.degrees(np.angle(hk[i]))),
                      "prominence_db": float(prominence), "zeta": zeta})
    return sorted(found, key=lambda p: -p["prominence_db"])


def mode_shape(f, slope, fn, zeta, tau):
    """e^(-s tau) s^slope wn^2 / (s^2 + 2 zeta wn s + wn^2) at f (zeta may be a column)."""
    import numpy as np
    jw, wn = 2j * np.pi * np.asarray(f), 2 * np.pi * fn
    return np.exp(-jw * tau) * jw ** slope * wn ** 2 / (jw ** 2 + 2 * zeta * wn * jw + wn ** 2)


def fit_mode(f, h, coherence, band, slope, min_coherence=MIN_COHERENCE):
    """H = k mode_shape(slope, fn, zeta, tau) fitted on the trusted bins, weighted by
    the coherence (sysid_yaw.yaw_model_fit with the slope free): {fn_hz, zeta,
    delay_ms, k, fit_error} (fit_error relative), None without enough data."""
    import numpy as np
    idx = trusted(f, h, coherence, band, min_coherence)
    if len(idx) < 6:
        return None
    fk, hk, w = f[idx], h[idx], coherence[idx]
    zetas = np.arange(0.03, 0.9, 0.01)[:, None]
    best = None
    for fn in np.arange(max(2.0, fk[0] - 2.0), fk[-1] + 5.0, 0.1):
        for tau in np.arange(0.0, 0.0205, 0.001):
            shape = mode_shape(fk, slope, fn, zetas, tau)
            k = np.real(np.sum(w * np.conj(shape) * hk, axis=1)) / np.sum(w * abs(shape) ** 2, axis=1)
            err = np.sum(w * abs(hk - k[:, None] * shape) ** 2, axis=1) / np.sum(w * abs(hk) ** 2)
            i = int(np.argmin(err))
            if best is None or err[i] < best["fit_error"]:
                best = {"fn_hz": float(fn), "zeta": float(zetas[i, 0]), "delay_ms": tau * 1000,
                        "k": float(k[i]), "fit_error": float(err[i])}
    return best


def chirp_band(chirps):
    """Hz to trust: inside the sweeps, clear of their ends (as sysid_yaw.plant_band)."""
    return (max(3.0, max(e["f0_hz"] for e in chirps) + 1.0), min(e["f1_hz"] for e in chirps) - 3.0)


def analyse(directory, axis=None, png=None, slope=None, segment=1024, out=None):
    import numpy as np
    directory = Path(directory)
    plan = json.loads((directory / "plan.json").read_text())
    # Recordings from before chirps had an axis are yaw.
    every = [e for e in plan["events"] if e["kind"] == "chirp"]
    axis = axis or (every[0].get("axis", "yaw") if every else "yaw")
    chirps = [e for e in every if e.get("axis", "yaw") == axis]
    if not chirps:
        raise ValueError(f"no {axis} chirp in {directory / 'plan.json'}")
    name, unit, take_input, column, output, axis_slope = AXES[axis]
    slope = axis_slope if slope is None else slope
    joints, gyro = read_bag(directory / "bag")
    total, rows = None, []
    for e in chirps:
        # The search window and the sweep, on one 300 Hz grid.
        grid = np.arange(e["t"] - 0.5, e["t"] + 1.0 + e["duration_s"], 1 / FS)
        u = np.interp(grid, joints[:, 0], take_input(joints[:, 1:]))
        offset, match = align(grid, u, e)
        start = e["t"] + offset
        grid = np.arange(start, start + e["duration_s"], 1 / FS)
        u = np.interp(grid, joints[:, 0], take_input(joints[:, 1:]))
        y = np.interp(grid, gyro[:, 0], gyro[:, 1 + column])
        r = chirp(grid - start, e["f0_hz"], e["f1_hz"], e["duration_s"])
        s = spectra(r, u, y, segment)
        total = add(total, s)
        f, h, _, c = frequency_response(s)
        rows.append({"amplitude": e.get("amplitude_nm"), "kp": e.get("kp"), "start_offset_s": float(offset),
                     "chirp_match": float(match), "mode": fit_mode(f, h, c, chirp_band([e]), slope),
                     "resonances": resonances(f, h, c, chirp_band([e]), slope)})
    band = chirp_band(chirps)
    f, h, h_direct, coherence = frequency_response(total)
    keep = (f >= band[0]) & (f <= band[1])
    result = {
        "source": directory.name, "axis": axis, "input": f"{name}, {unit}", "output": f"{output}, rad/s",
        "slope": slope, "band_hz": list(band), "per_chirp": rows,
        "mode": fit_mode(f, h, coherence, band, slope), "resonances": resonances(f, h, coherence, band, slope),
        "f_hz": [round(float(v), 4) for v in f[keep]],
        "h": [[round(float(v.real), 5), round(float(v.imag), 5)] for v in h[keep]],
        "h_direct": [[round(float(v.real), 5), round(float(v.imag), 5)] for v in h_direct[keep]],
        "coherence": [round(float(v), 3) for v in coherence[keep]],
    }
    report(result)
    path = Path(out) if out else directory / f"bode_{axis}.json"
    path.write_text(json.dumps(result, indent=1))
    print(f"wrote {path}")
    if png:
        plot(result, png)
    return result


def report(result):
    import numpy as np
    print(f"{result['source']}: {result['output']} per {result['input']}, "
          f"{result['band_hz'][0]:g}-{result['band_hz'][1]:g} Hz")
    def mode(m):
        return (f"mode {m['fn_hz']:.1f} Hz, zeta {m['zeta']:.2f}, delay {m['delay_ms']:.0f} ms "
                f"(fit error {m['fit_error']:.2f})" if m else "no mode fitted")

    for k, row in enumerate(result["per_chirp"]):
        legs = f", leg kp {row['kp']:g}" if row.get("kp") is not None else ""
        print(f"  chirp {k + 1} ({row['amplitude']} Nm{legs}): started {row['start_offset_s'] * 1000:+.0f} ms after "
              f"the log, match {row['chirp_match']:.2f}; {mode(row['mode'])}")
    f = np.array(result["f_hz"])
    h = np.array([complex(*v) for v in result["h"]])
    hd = np.array([complex(*v) for v in result["h_direct"]])
    c = np.array(result["coherence"])
    print(f"  {'Hz':>5} {'|H|':>7} {'dB':>6} {'phase':>6} {'coh':>5} {'direct':>7} {'phase':>6}")
    for q in (3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 18, 20, 25, 30, 35, 40):
        if not result["band_hz"][0] <= q <= result["band_hz"][1]:
            continue
        i = int(np.argmin(abs(f - q)))
        print(f"  {f[i]:5.1f} {abs(h[i]):7.3f} {20 * np.log10(abs(h[i])):6.1f} {np.degrees(np.angle(h[i])):+6.0f} "
              f"{c[i]:5.2f} {abs(hd[i]):7.3f} {np.degrees(np.angle(hd[i])):+6.0f}")
    print(f"  all chirps: {mode(result['mode'])}")
    if not result["resonances"]:
        print("  no peak stands out (3 dB, slope taken out): a well-damped mode, read the fit")
    for p in result["resonances"]:
        print(f"  resonance {p['f_hz']:.1f} Hz: |H| {p['gain']:.3f} at {p['phase_deg']:+.0f} deg, "
              f"{p['prominence_db']:.1f} dB above its surroundings, zeta {p['zeta']:.2f}")


def plot(result, png):
    import numpy as np
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("no matplotlib: the plot is skipped, the table is in the JSON")
        return
    f = np.array(result["f_hz"])
    h = np.array([complex(*v) for v in result["h"]])
    hd = np.array([complex(*v) for v in result["h_direct"]])
    c = np.array(result["coherence"])
    fig, (ax_mag, ax_phase, ax_coh) = plt.subplots(3, 1, sharex=True, figsize=(8, 8),
                                                   gridspec_kw={"height_ratios": [3, 2, 1]})
    ax_mag.semilogx(f, 20 * np.log10(abs(h)), label="through the chirp (Pry / Pru)")
    ax_mag.semilogx(f, 20 * np.log10(abs(hd)), "--", alpha=0.6, label="direct (Puy / Puu)")
    ax_phase.semilogx(f, np.degrees(np.unwrap(np.angle(h))))
    ax_phase.semilogx(f, np.degrees(np.unwrap(np.angle(hd))), "--", alpha=0.6)
    ax_coh.semilogx(f, c)
    ax_coh.axhline(MIN_COHERENCE, color="grey", lw=0.8)
    m = result["mode"]
    if m:
        model = m["k"] * mode_shape(f, result["slope"], m["fn_hz"], m["zeta"], m["delay_ms"] / 1000)
        label = f"fit: {m['fn_hz']:.1f} Hz, zeta {m['zeta']:.2f}, {m['delay_ms']:.0f} ms"
        ax_mag.semilogx(f, 20 * np.log10(abs(model)), ":", color="k", label=label)
        ax_phase.semilogx(f, np.degrees(np.unwrap(np.angle(model))), ":", color="k")
        for ax in (ax_mag, ax_phase):
            ax.axvline(m["fn_hz"], color="k", lw=0.8, alpha=0.5)
    for p in result["resonances"]:
        for ax in (ax_mag, ax_phase, ax_coh):
            ax.axvline(p["f_hz"], color="tab:red", lw=0.8, alpha=0.7)
        ax_mag.annotate(f"{p['f_hz']:.1f} Hz, zeta {p['zeta']:.2f}", (p["f_hz"], 20 * np.log10(p["gain"])),
                        textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax_mag.set_ylabel(f"dB, rad/s per {result['input'].split(', ')[1].replace('Nm per', 'Nm of')}")
    ax_phase.set_ylabel("phase, deg")
    ax_coh.set_ylabel("coherence")
    ax_coh.set_xlabel("Hz")
    ax_coh.set_ylim(0, 1.05)
    ax_mag.legend(fontsize=8)
    ax_mag.set_title(f"{result['axis']}: {result['output']} per {result['input']} ({result['source']})",
                     fontsize=9)
    for ax in (ax_mag, ax_phase, ax_coh):
        ax.grid(True, which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(png, dpi=120)
    print(f"plot: {png}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("directory", help="a sysid recording (plan.json and bag/)")
    parser.add_argument("--axis", choices=sorted(AXES), help="default: the chirps' own axis (yaw before axes)")
    parser.add_argument("--png", help="draw the Bode plot into this file")
    parser.add_argument("--slope", type=int, choices=(-1, 0, 1),
                        help="the response's s^p away from the mode (default per axis: yaw, pitch -1, roll +1)")
    parser.add_argument("--segment", type=int, default=1024, help="Welch segment, samples at 300 Hz")
    parser.add_argument("--out", help="the JSON (default DIR/bode_<axis>.json)")
    args = parser.parse_args()
    analyse(args.directory, args.axis, args.png, args.slope, args.segment, args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
