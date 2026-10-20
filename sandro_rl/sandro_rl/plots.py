"""Traces of policy and WBC recordings side by side (evaluate.py's CSVs)."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from .evaluate import compare_runs


def traces(recordings: dict[str, Path], out: Path, title: str = "") -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # Forward speed at the axle midpoint where the recording has it (policy
    # runs); the WBC's recordings only have its own wheel-odometry estimate.
    # closed_loop's truth_speed is the torso origin's, 0.35 m from the axle,
    # and mostly shows the yaw rate while turning.
    panels = [("diag_truth_pitch_rad", "pitch, truth (rad)"),
              (("diag_truth_forward_m_s", "diag_forward_velocity_m_s"), "forward speed (m/s)"),
              ("diag_yaw_rate_rad_s", "yaw rate, IMU (rad/s)"), ("diag_leg_height_m", "leg height (m)"),
              ("diag_balance_torque_nm", "wheel torque, total (Nm)")]
    loaded = {label: compare_runs.load(str(path))["diag"] for label, path in recordings.items()}
    has_roll = any(np.isfinite(d.get("diag_roll_reference_rad", [np.nan])).any() for d in loaded.values())
    if has_roll:
        panels.insert(4, ("diag_truth_roll_rad", "roll, truth (rad)"))
    has_jump = any((np.nan_to_num(d.get("cmd_jump", [0.0])) > 0).any() for d in loaded.values())
    if has_jump:
        panels.insert(4, ("diag_truth_wheel_clearance_m", "lower wheel clearance (m)"))
    fig, axes = plt.subplots(len(panels), 1, figsize=(12, 11 + 2 * (has_roll + has_jump)), sharex=True)
    for label, d in loaded.items():
        for ax, (keys, name) in zip(axes, panels):
            keys = (keys,) if isinstance(keys, str) else keys
            for i, key in enumerate(keys):
                if key in d and np.isfinite(d[key]).any():
                    suffix = "" if i == 0 else " (own estimate)"
                    ax.plot(d["t_s"], d[key], label=label + suffix, linewidth=0.9)
                    break
    first = next(iter(loaded.values()))
    axes[1].plot(first["t_s"], first["cmd_forward"], "k--", linewidth=0.8, label="command")
    axes[2].plot(first["t_s"], first["cmd_yaw"], "k--", linewidth=0.8, label="command")
    if "diag_height_reference_m" in first:
        axes[3].plot(first["t_s"], first["diag_height_reference_m"], "k--", linewidth=0.8, label="reference")
    if has_jump:
        axes[4].plot(first["t_s"], first["cmd_jump"], "k--", linewidth=0.8, label="jump height command")
        axes[4].legend(fontsize=8)
    if has_roll:
        roll = next(d for d in loaded.values() if np.isfinite(d.get("diag_roll_reference_rad", [np.nan])).any())
        ax = axes[4 + has_jump]
        ax.plot(roll["t_s"], roll["diag_roll_reference_rad"], "k--", linewidth=0.8, label="reference")
        ax.legend(fontsize=8)
    for ax, (_, name) in zip(axes, panels):
        ax.set_ylabel(name, fontsize=8)
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[1].legend(fontsize=8)
    axes[-1].set_xlabel("time since engagement (s)")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    plt.close(fig)
    return out


if __name__ == "__main__":
    out = Path(sys.argv[1])
    traces({Path(p).stem: Path(p) for p in sys.argv[2:]}, out, out.stem)
