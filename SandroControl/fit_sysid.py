#!/usr/bin/env python3
"""Fit grouped MuJoCo actuator dynamics from SandroControl sys-ID runs."""
from __future__ import annotations

import argparse
import copy
import csv
import json
from collections import deque
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares

from config import load_config
from identified import model_sha256
from models import MitCommand, MotorState, RobotState
from mujoco_backend import MujocoBackend

HERE = Path(__file__).resolve().parent


def load_run(path: Path) -> dict:
    metadata_path = path.with_suffix(".json")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("dropped_rows", 0):
        raise ValueError(f"{path}: recorder dropped rows")
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 20:
        raise ValueError(f"{path}: too few samples")
    axis = metadata["trial"]["axis"]
    usable = [
        row
        for row in rows
        if row["experiment_state"] in {"excite", "settle_after"}
        and not row.get(f"{axis}.error")
        and row.get(f"{axis}.online") == "1"
    ]
    if len(usable) < 20:
        raise ValueError(f"{path}: too few valid excitation samples")
    return {"path": path, "metadata": metadata, "rows": rows, "usable": usable}


def row_float(row: dict, key: str, default: float = 0.0) -> float:
    value = row.get(key, "")
    return default if value in {"", None} else float(value)


def row_commands(row: dict, names) -> dict[str, MitCommand]:
    return {
        name: MitCommand(
            position=row_float(row, f"{name}.cmd_position"),
            velocity=row_float(row, f"{name}.cmd_velocity"),
            kp=row_float(row, f"{name}.cmd_kp"),
            kd=row_float(row, f"{name}.cmd_kd"),
            torque=row_float(row, f"{name}.cmd_torque"),
        )
        for name in names
    }


def configure_backend(
    cfg: dict, kind: str, params: np.ndarray, delay_steps: int
) -> MujocoBackend:
    backend = MujocoBackend(cfg)
    backend.start()
    values = {
        "groups": {
            "leg": {
                "damping": 0.01,
                "frictionloss": 0.1,
                "armature": 0.00016832,
                "torque_scale": 1.0,
            },
            "wheel": {
                "damping": 0.01,
                "frictionloss": 0.1,
                "armature": 0.00016832,
                "torque_scale": 1.0,
            },
        }
    }
    values["groups"][kind] = {
        "damping": float(params[0]),
        "frictionloss": float(params[1]),
        "armature": float(params[2]),
        "torque_scale": float(params[3]),
    }
    backend._apply_identified(values)
    backend._delay_steps = delay_steps
    backend._command_history = deque(maxlen=max(1, delay_steps + 1))
    return backend


def simulate_run(
    cfg: dict,
    run: dict,
    kind: str,
    params: np.ndarray,
    delay_steps: int,
) -> tuple[np.ndarray, np.ndarray]:
    backend = configure_backend(cfg, kind, params, delay_steps)
    axis = run["metadata"]["trial"]["axis"]
    names = tuple(cfg["robot"]["motors"])
    first = run["rows"][0]
    initial = RobotState(
        motors={
            name: MotorState(
                position=row_float(first, f"{name}.position"),
                velocity=row_float(first, f"{name}.velocity"),
                online=True,
                age_s=0.0,
            )
            for name in names
        },
        source="recording",
    )
    backend.mirror_state(initial)
    for name, motor in initial.motors.items():
        joint = backend._joint[name]
        dof = int(backend.model.jnt_dofadr[joint])
        backend.data.qvel[dof] = backend._sim_sign[name] * motor.velocity
    backend.enable()
    wanted = {id(row) for row in run["usable"]}
    positions, velocities = [], []
    try:
        for row in run["rows"]:
            backend.write(row_commands(row, names))
            if id(row) in wanted:
                motor = backend.read().motors[axis]
                positions.append(motor.position)
                velocities.append(motor.velocity)
    finally:
        backend.close()
    return np.asarray(positions), np.asarray(velocities)


def residuals(
    params: np.ndarray,
    cfg: dict,
    runs: list[dict],
    kind: str,
    delay_steps: int,
) -> np.ndarray:
    chunks = []
    for run in runs:
        axis = run["metadata"]["trial"]["axis"]
        simulated_q, simulated_dq = simulate_run(
            cfg, run, kind, params, delay_steps
        )
        measured_q = np.asarray(
            [row_float(row, f"{axis}.position") for row in run["usable"]]
        )
        measured_dq = np.asarray(
            [row_float(row, f"{axis}.velocity") for row in run["usable"]]
        )
        chunks.extend(((simulated_q - measured_q) / 0.05, (simulated_dq - measured_dq) / 1.0))
    return np.concatenate(chunks)


def rmse(residual: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(residual))))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--holdout", nargs="*", type=Path, default=[])
    parser.add_argument("--config", type=Path, default=HERE / "config.yaml")
    parser.add_argument(
        "--output", type=Path, default=HERE / "identified_params.yaml"
    )
    parser.add_argument("--max-delay-ms", type=float, default=30.0)
    parser.add_argument("--max-nfev", type=int, default=50)
    parser.add_argument(
        "--enable",
        action="store_true",
        help="enable the fitted override immediately (default writes it disabled)",
    )
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    runs = [load_run(path) for path in args.runs]
    holdouts = [load_run(path) for path in args.holdout]
    kinds = {
        cfg["robot"]["motors"][run["metadata"]["trial"]["axis"]]["kind"]
        for run in runs + holdouts
    }
    if len(kinds) != 1:
        raise ValueError("fit leg and wheel runs separately")
    kind = kinds.pop()

    probe = MujocoBackend(cfg)
    probe.start()
    axis = runs[0]["metadata"]["trial"]["axis"]
    dof = int(probe.model.jnt_dofadr[probe._joint[axis]])
    initial = np.asarray(
        [
            probe.model.dof_damping[dof],
            probe.model.dof_frictionloss[dof],
            probe.model.dof_armature[dof],
            1.0,
        ],
        dtype=float,
    )
    probe.close()
    lower = np.asarray([0.0, 0.0, 1e-7, 0.2])
    upper = np.asarray([2.0, 3.0, 0.1, 2.0])
    frequency = float(cfg["runtime"]["control_hz"])
    max_delay_steps = round(args.max_delay_ms * 1e-3 * frequency)

    baseline = rmse(residuals(initial, cfg, runs, kind, 0))
    best = None
    for delay_steps in range(max_delay_steps + 1):
        result = least_squares(
            residuals,
            initial,
            bounds=(lower, upper),
            args=(cfg, runs, kind, delay_steps),
            loss="soft_l1",
            max_nfev=args.max_nfev,
        )
        score = rmse(residuals(result.x, cfg, runs, kind, delay_steps))
        if best is None or score < best[0]:
            best = (score, delay_steps, result.x)
    assert best is not None
    score, delay_steps, fitted = best
    holdout_score = (
        rmse(residuals(fitted, cfg, holdouts, kind, delay_steps))
        if holdouts
        else None
    )

    existing = yaml.safe_load(args.output.read_text()) if args.output.exists() else {}
    output = copy.deepcopy(existing or {})
    output.update(
        enabled=bool(args.enable),
        model_sha256=model_sha256(cfg["mujoco"]["model"]),
        source_runs=[str(run["path"]) for run in runs],
        command_delay_s=delay_steps / frequency,
    )
    output.setdefault("groups", {})
    for group_name in ("leg", "wheel"):
        output["groups"].setdefault(
            group_name,
            {
                "damping": 0.01,
                "frictionloss": 0.1,
                "armature": 0.00016832,
                "torque_scale": 1.0,
            },
        )
    output["groups"][kind] = {
        "damping": float(fitted[0]),
        "frictionloss": float(fitted[1]),
        "armature": float(fitted[2]),
        "torque_scale": float(fitted[3]),
    }
    output["fit"] = {
        "kind": kind,
        "baseline_rmse": baseline,
        "fitted_rmse": score,
        "holdout_rmse": holdout_score,
        "samples": sum(len(run["usable"]) for run in runs),
    }
    args.output.write_text(yaml.safe_dump(output, sort_keys=False), encoding="utf-8")
    print(
        f"{kind}: baseline={baseline:.4f}, fitted={score:.4f}, "
        f"holdout={holdout_score}, delay={delay_steps / frequency:.4f}s"
    )
    print(f"wrote {args.output} (enabled={bool(args.enable)})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
