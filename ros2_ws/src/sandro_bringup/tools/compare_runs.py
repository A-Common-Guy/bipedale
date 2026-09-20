#!/usr/bin/env python3
"""Compare drive_test recordings, typically one from MuJoCo and one from hardware.

    python3 tools/compare_runs.py sim.csv real.csv [--phase N]

For every command phase both runs share, prints the metrics that describe the
sim-to-real gap for a balancing robot:

  pitch_rms / pitch_p2p     body pitch about its mean (rad): oscillation level
  pitch_peak_hz             dominant pitch oscillation frequency (Hz)
  vel_err_rms               forward speed minus reference (m/s)
  yaw_err_rms               IMU yaw rate minus reference (rad/s)
  yaw_rise_s                time for the yaw rate to reach 63 % of a step command
  wheel_tau_rms             balance wheel torque (Nm, total)
  wheel_flip_frac           fraction of successive wheel velocity samples that
                            change sign (0.5 means chatter at the sample rate)
  leg_eff_rms               leg joint efforts (Nm)
  height_err                mean leg height minus its reference (m)

Only needs numpy. If matplotlib is importable and --plot is given, pitch,
speed and yaw traces of both runs are drawn per phase.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict

import numpy as np


def load(path: str) -> dict:
    """Return {"diag": {col: array}, "joint": {...}, "imu": {...}} keyed by stream."""
    streams: dict[str, dict[str, list]] = defaultdict(lambda: defaultdict(list))
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            stream = row["stream"]
            for key, value in row.items():
                if value in ("", None):
                    continue
                try:
                    streams[stream][key].append(float(value))
                except ValueError:
                    streams[stream][key].append(math.nan)
    result = {}
    for stream, columns in streams.items():
        length = max(len(v) for v in columns.values())
        result[stream] = {
            k: np.asarray(v, dtype=float) for k, v in columns.items() if len(v) == length
        }
    return result


def select(table: dict, phase: int) -> dict:
    mask = table["phase"] == phase
    return {k: v[mask] for k, v in table.items()}


def rms(values) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(values ** 2))) if values.size else math.nan


def peak_frequency(t, values) -> float:
    values = np.asarray(values, dtype=float)
    ok = np.isfinite(values) & np.isfinite(t)
    t, values = t[ok], values[ok]
    if values.size < 16:
        return math.nan
    dt = float(np.median(np.diff(t)))
    if not dt > 0:
        return math.nan
    centred = values - values.mean()
    spectrum = np.abs(np.fft.rfft(centred * np.hanning(centred.size)))
    freqs = np.fft.rfftfreq(centred.size, dt)
    spectrum[freqs < 0.3] = 0.0  # ignore drift below 0.3 Hz
    return float(freqs[int(np.argmax(spectrum))])


def rise_time(t, values, target) -> float:
    """Seconds until |values| first reaches 63 % of |target| within the phase."""
    if not np.isfinite(target) or abs(target) < 1e-6:
        return math.nan
    threshold = 0.63 * abs(target)
    for time, value in zip(t, values):
        if np.isfinite(value) and abs(value) >= threshold:
            return float(time - t[0])
    return math.inf


def metrics(run: dict, phase: int) -> dict:
    diag = select(run["diag"], phase) if "diag" in run else {}
    joint = select(run["joint"], phase) if "joint" in run else {}
    out: dict[str, float] = {}
    if diag and diag["t_s"].size:
        t = diag["t_s"]
        pitch = diag.get("diag_pitch_rad", np.array([]))
        out["pitch_rms"] = rms(pitch - np.nanmean(pitch)) if pitch.size else math.nan
        out["pitch_p2p"] = float(np.nanmax(pitch) - np.nanmin(pitch)) if pitch.size else math.nan
        out["pitch_peak_hz"] = peak_frequency(t, pitch) if pitch.size else math.nan
        v = diag.get("diag_forward_velocity_m_s")
        v_ref = diag.get("diag_velocity_reference_m_s")
        out["vel_err_rms"] = rms(v - v_ref) if v is not None and v_ref is not None else math.nan
        yaw = diag.get("diag_yaw_rate_rad_s")
        yaw_ref = diag.get("diag_yaw_rate_reference_rad_s")
        if yaw is not None and yaw_ref is not None:
            out["yaw_err_rms"] = rms(yaw - yaw_ref)
            out["yaw_rise_s"] = rise_time(t, yaw, float(np.nanmedian(yaw_ref)))
        else:
            out["yaw_err_rms"] = out["yaw_rise_s"] = math.nan
        tau = diag.get("diag_balance_torque_nm")
        out["wheel_tau_rms"] = rms(tau) if tau is not None else math.nan
        h = diag.get("diag_leg_height_m")
        h_ref = diag.get("diag_height_reference_m")
        out["height_err"] = float(np.nanmean(h - h_ref)) if h is not None and h_ref is not None else math.nan
        out["duration_s"] = float(t[-1] - t[0])
    if joint and joint["t_s"].size:
        flips = []
        for wheel in ("vel_right_wheel", "vel_left_wheel"):
            w = joint.get(wheel)
            if w is not None and w.size > 2:
                flips.append(float(np.mean(np.sign(w[1:]) * np.sign(w[:-1]) < 0)))
        out["wheel_flip_frac"] = float(np.mean(flips)) if flips else math.nan
        legs = [joint[k] for k in ("eff_right_motor_inner", "eff_right_motor_outer",
                                   "eff_left_motor_inner", "eff_left_motor_outer") if k in joint]
        out["leg_eff_rms"] = rms(np.concatenate(legs)) if legs else math.nan
    return out


def phase_commands(run: dict, phase: int) -> str:
    diag = select(run["diag"], phase)
    if not diag["t_s"].size:
        return "?"
    return (f"v={np.nanmedian(diag['cmd_forward']):+.2f} "
            f"yaw={np.nanmedian(diag['cmd_yaw']):+.2f} "
            f"h_rate={np.nanmedian(diag['cmd_height_rate']):+.2f}")


ORDER = ("duration_s", "pitch_rms", "pitch_p2p", "pitch_peak_hz", "vel_err_rms",
         "yaw_err_rms", "yaw_rise_s", "wheel_tau_rms", "wheel_flip_frac",
         "leg_eff_rms", "height_err")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("runs", nargs="+", help="drive_test --record CSV files")
    parser.add_argument("--label", action="append", help="label per run, in order")
    parser.add_argument("--phase", type=int, help="only this command phase")
    parser.add_argument("--plot", metavar="PNG", help="write trace plots (needs matplotlib)")
    args = parser.parse_args()
    runs = [load(path) for path in args.runs]
    labels = args.label or [path.rsplit("/", 1)[-1] for path in args.runs]
    if len(labels) != len(runs):
        parser.error("one --label per run")
    for run, label in zip(runs, labels):
        if "diag" not in run:
            print(f"{label}: no diagnostics rows; record with drive_test.py --record", file=sys.stderr)
            return 1
    phases = sorted(set.intersection(*(set(np.unique(r["diag"]["phase"]).astype(int)) for r in runs)))
    if args.phase is not None:
        phases = [p for p in phases if p == args.phase]
    width = max(12, max(len(l) for l in labels) + 2)
    for phase in phases:
        print(f"\nphase {phase}  ({phase_commands(runs[0], phase)})")
        print(f"{'metric':16s}" + "".join(f"{l:>{width}s}" for l in labels))
        rows = [metrics(run, phase) for run in runs]
        for key in ORDER:
            print(f"{key:16s}" + "".join(f"{row.get(key, math.nan):>{width}.4f}" for row in rows))
    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib not available; skipping plot", file=sys.stderr)
            return 0
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        for run, label in zip(runs, labels):
            d = run["diag"]
            axes[0].plot(d["t_s"], d["diag_pitch_rad"], label=label, linewidth=0.8)
            axes[1].plot(d["t_s"], d["diag_forward_velocity_m_s"], label=label, linewidth=0.8)
            axes[2].plot(d["t_s"], d["diag_yaw_rate_rad_s"], label=label, linewidth=0.8)
        axes[1].plot(runs[0]["diag"]["t_s"], runs[0]["diag"]["diag_velocity_reference_m_s"],
                     "k--", linewidth=0.6, label="reference")
        axes[2].plot(runs[0]["diag"]["t_s"], runs[0]["diag"]["diag_yaw_rate_reference_rad_s"],
                     "k--", linewidth=0.6)
        for axis, name in zip(axes, ("pitch [rad]", "forward speed [m/s]", "yaw rate [rad/s]")):
            axis.set_ylabel(name)
            axis.grid(alpha=0.3)
        axes[0].legend(loc="upper right")
        axes[2].set_xlabel("time [s]")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=130)
        print(f"wrote {args.plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
