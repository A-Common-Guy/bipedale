#!/usr/bin/env python3
"""Closed-loop simulation of the whole-body controller without ROS.

Builds and runs closed_loop/closed_loop.cpp: MuJoCo on scene_ros2.xml with the
controller manager's 300 Hz wake-up rule, sandro_sim's latency emulation and
per-step MIT drive law, and sandro_controllers' WholeBodyCore, unchanged. A
23 s drive test takes under a second, so latency, gain and model sweeps that
take an afternoon through the ROS launch take a minute here. Recordings have
drive_test.py's layout, so compare_runs.py reads them next to hardware runs.

    # the hardware runs' sequence, controllers.yaml + the robot overrides
    python3 closed_loop.py run --scenario hardware \\
        --params sandro_bringup/config/sandro_real_overrides.yaml --output sim.csv
    python3 ../../sandro_bringup/tools/compare_runs.py sim.csv ../../../../recordings/real4_2026-09-20.csv

    # which latencies does the shipped controller survive?
    python3 closed_loop.py sweep --scenario hardware --grid imu_rate=30,100,200 \\
        --grid joint_delay=0.001,0.005

    # the gates a parameter or model change must pass before the robot
    # (controllers.yaml + sandro_real_overrides.yaml, like the robot)
    python3 closed_loop.py check

Needs the `mujoco` Python package (for its headers and library, or
--mujoco-dir), Eigen 3 headers (/usr/include/eigen3 or --eigen-dir), g++,
numpy and PyYAML.
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SRC = HERE.parents[1]  # ros2_ws/src
BINARY = HERE / "closed_loop" / "build" / "closed_loop"
MODEL = SRC / "sandro_description" / "mjcf" / "scene_ros2.xml"
CONTROLLERS = SRC / "sandro_bringup" / "config" / "controllers.yaml"
ROBOT_OVERRIDES = SRC / "sandro_bringup" / "config" / "sandro_real_overrides.yaml"
CONTROLLER = "whole_body_controller"
SOURCES = [
    HERE / "closed_loop" / "closed_loop.cpp",
    *(SRC / "sandro_controllers" / "src" / f"{name}.cpp"
      for name in ("kinematics", "lqr", "whole_body_core", "whole_body_model",
                   "whole_body_parameters")),
]
HEADERS = [*(SRC / "sandro_controllers" / "include").rglob("*.hpp"),
           *(SRC / "sandro_sim" / "include").rglob("*.hpp")]

sys.path.insert(0, str(SRC / "sandro_bringup" / "tools"))
import compare_runs  # noqa: E402

# Command phases, drive_test.py's "duration,forward,yaw[,height_rate]".
SCENARIOS = {
    # The sequence of every hardware recording so far (recordings/real*.csv).
    "hardware": ["5,0,0", "4,0.2,0", "4,0,0", "4,0.2,0.6", "3,0,0"],
    # drive_test.py's default sequence.
    "default": ["3,0,0", "4,0.3,0", "3,0,0", "4,0.3,1.0", "3,0,0", "3,0,0,1.2", "2,0,0",
                "3,0,0,-0.05", "2,0,0", "3,0,0,0.05", "3,0,0"],
    # Standing only, for noise floor and limit cycles.
    "stand": ["10,0,0"],
    # Standing with forward and backward shoves on the torso (pushes below).
    "push": ["10,0,0"],
}
SCENARIO_PUSHES = {"push": ["2,8,0.1", "6,-8,0.1"]}

# Simulation options of closed_loop.cpp and their sandro.launch.py names.
SIM_OPTIONS = {
    "imu_delay": ("--imu-delay", "sim_imu_delay_s"),
    "imu_rate": ("--imu-rate", "sim_imu_rate_hz"),
    "joint_delay": ("--joint-delay", "sim_joint_delay_s"),
    "joint_rate": ("--joint-rate", "sim_joint_rate_hz"),
    "joint_stagger": ("--joint-stagger", "sim_joint_stagger"),
    "command_delay": ("--command-delay", "sim_command_delay_s"),
    "drive_plugin": ("--drive-plugin", None),
    "imu_noise": ("--imu-noise", None),
    "timestep": ("--timestep", None),
    "update_rate": ("--update-rate", None),
    "settle": ("--settle", None),
    "seed": ("--seed", None),
}


def find_mujoco(explicit: str | None) -> Path:
    if explicit:
        return Path(explicit)
    try:
        import mujoco
    except ImportError:
        sys.exit("pip install mujoco, or pass --mujoco-dir with include/ and libmujoco.so*")
    return Path(mujoco.__file__).parent


def build(mujoco_dir: Path, eigen_dir: Path, force: bool = False) -> Path:
    newest = max(p.stat().st_mtime for p in [*SOURCES, *HEADERS])
    if not force and BINARY.exists() and BINARY.stat().st_mtime > newest:
        return BINARY
    libraries = sorted(mujoco_dir.glob("libmujoco.so*")) or sorted((mujoco_dir / "lib").glob("libmujoco.so*"))
    include = mujoco_dir / "include"
    if not libraries or not include.is_dir():
        sys.exit(f"no MuJoCo headers/library under {mujoco_dir}")
    BINARY.parent.mkdir(parents=True, exist_ok=True)
    command = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra",
               f"-I{SRC / 'sandro_controllers' / 'include'}", f"-I{SRC / 'sandro_sim' / 'include'}",
               "-isystem", str(eigen_dir), "-isystem", str(include),
               *map(str, SOURCES), str(libraries[0]), f"-Wl,-rpath,{libraries[0].parent}",
               "-o", str(BINARY)]
    print("building closed_loop ...", file=sys.stderr)
    subprocess.run(command, check=True)
    return BINARY


def flatten(prefix: str, value, out: dict) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            flatten(f"{prefix}.{key}" if prefix else str(key), item, out)
    else:
        out[prefix] = value


def controller_parameters(files: list[str], overrides: list[str]) -> dict[str, float]:
    """controllers.yaml, then each --params file, then --set, as ROS layers them."""
    values: dict = {}
    for path in [str(CONTROLLERS), *files]:
        document = yaml.safe_load(Path(path).read_text()) or {}
        section = (document.get(CONTROLLER) or {}).get("ros__parameters") or {}
        flatten("", section, values)
    for item in overrides:
        name, _, text = item.partition("=")
        values[name] = float(text)
    # Only the numeric ones reach the core; names, topics and the diagnostics
    # rate are ROS-side.
    skip = {"joints", "imu_name", "command_topic", "diagnostics.publish_rate_hz"}
    return {k: float(v) for k, v in values.items()
            if k not in skip and isinstance(v, (int, float)) and not isinstance(v, bool)}


def run_once(binary: Path, parameters: dict[str, float], scenario: str, sim: dict[str, str],
             output: Path, quiet: bool = False) -> tuple[int, str]:
    command = [str(binary), "--model", str(MODEL), "--output", str(output)]
    for name, value in parameters.items():
        command += ["--param", f"{name}={value!r}"]
    for phase in SCENARIOS[scenario]:
        command += ["--phase", phase]
    for push in SCENARIO_PUSHES.get(scenario, []):
        command += ["--push", push]
    for key, value in sim.items():
        command += [SIM_OPTIONS[key][0], str(value)]
    result = subprocess.run(command, capture_output=True, text=True)
    text = (result.stdout + result.stderr).strip()
    if result.returncode == 2:
        sys.exit(text)
    if not quiet:
        print(text)
    return result.returncode, text


def summary(path: Path, scenario: str) -> dict[int, dict[str, float]]:
    # The last control cycle can already count as the phase after the last.
    run = compare_runs.load(str(path))
    phases = sorted(int(p) for p in np.unique(run["diag"]["phase"])
                    if 0 <= p < len(SCENARIOS[scenario]))
    return {p: compare_runs.metrics(run, p) for p in phases}


def print_metrics(per_phase: dict[int, dict[str, float]]) -> None:
    keys = [k for k in compare_runs.ORDER if k != "duration_s"]
    print(f"{'phase':>5s} " + " ".join(f"{k:>15s}" for k in keys))
    for phase, values in per_phase.items():
        print(f"{phase:5d} " + " ".join(f"{values.get(k, math.nan):15.4f}" for k in keys))


def parse_sim(items: list[str]) -> dict[str, str]:
    sim = {}
    for item in items:
        key, _, value = item.partition("=")
        if key not in SIM_OPTIONS:
            sys.exit(f"unknown simulation option '{key}' (one of {', '.join(SIM_OPTIONS)})")
        sim[key] = value
    return sim


def command_run(args, binary: Path) -> int:
    parameters = controller_parameters(args.params, args.set)
    output = Path(args.output) if args.output else Path(tempfile.mkstemp(suffix=".csv")[1])
    code, _ = run_once(binary, parameters, args.scenario, parse_sim(args.sim), output)
    print(f"recording: {output}")
    print_metrics(summary(output, args.scenario))
    return code


def command_sweep(args, binary: Path) -> int:
    parameters = controller_parameters(args.params, args.set)
    base = parse_sim(args.sim)
    axes = []
    for item in args.grid:
        key, _, values = item.partition("=")
        axes.append((key, values.split(",")))
    header = [key for key, _ in axes]
    print(" ".join(f"{h:>14s}" for h in header)
          + f" {'result':>8s} {'stand_pitch_rms':>15s} {'stand_peak_hz':>13s} {'vel_err_rms':>11s}"
            f" {'wheel_flip':>10s} {'leg_eff_rms':>11s}")
    print("(stand_*: phase 0; the others: mean over all phases)")
    with tempfile.TemporaryDirectory() as tmp:
        for combination in itertools.product(*(values for _, values in axes)):
            sim = dict(base)
            run_parameters = dict(parameters)
            for (key, _), value in zip(axes, combination):
                if key in SIM_OPTIONS:
                    sim[key] = value
                else:
                    run_parameters[key] = float(value)
            output = Path(tmp) / "run.csv"
            code, _ = run_once(binary, run_parameters, args.scenario, sim, output, quiet=True)
            if code == 3:
                print(" ".join(f"{v:>14s}" for v in combination) + f" {'FELL':>8s}", flush=True)
                continue
            per_phase = summary(output, args.scenario)
            stand = per_phase.get(0, {})

            def mean(key: str) -> float:
                values = [m.get(key, math.nan) for m in per_phase.values()]
                values = [v for v in values if math.isfinite(v)]
                return float(np.mean(values)) if values else math.nan

            print(" ".join(f"{v:>14s}" for v in combination)
                  + f" {'ok':>8s}"
                  f" {stand.get('pitch_rms', math.nan):15.4f} {stand.get('pitch_peak_hz', math.nan):13.2f}"
                  f" {mean('vel_err_rms'):11.4f} {mean('wheel_flip_frac'):10.3f} {mean('leg_eff_rms'):11.3f}",
                  flush=True)
    return 0


# Gates: (description, scenario, simulation options, predicate on the
# per-phase metrics). Each must hold for a parameter or model change before
# it goes on the robot; none may fall. They encode what is known about the
# physical robot, not only the nominal simulation (SIM_TO_REAL.md,
# "Closed-loop simulation without ROS").
UART_IMU = {"imu_rate": "30"}  # what the BNO085 UART path delivered on every hardware run


def gates():
    return [
        ("hardware sequence, nominal latency: stands within 0.035 rad RMS, tracks 0.2 m/s",
         "hardware", {}, lambda m: m[0]["pitch_rms"] < 0.035 and m[1]["vel_err_rms"] < 0.12),
        ("hardware sequence, IMU at 30 Hz (UART)", "hardware", UART_IMU, lambda m: True),
        ("hardware sequence, drive law once per control cycle (no MitDrivePlugin)",
         "hardware", {"drive_plugin": "false"}, lambda m: True),
        ("8 N shoves for 0.1 s, forward and back", "push", {}, lambda m: True),
        ("drive_test default sequence, nominal latency", "default", {}, lambda m: True),
        ("drive_test default sequence, IMU at 30 Hz (UART)", "default", UART_IMU, lambda m: True),
    ]


def command_check(args, binary: Path) -> int:
    # The identified model carries the robot's payload, which only the robot
    # overrides tell the controller about.
    files = args.params or ([] if args.without_robot_overrides else [str(ROBOT_OVERRIDES)])
    parameters = controller_parameters(files, args.set)
    print(f"parameters: controllers.yaml{''.join(' + ' + Path(f).name for f in files)}")
    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        for description, scenario, sim, predicate in gates():
            output = Path(tmp) / "run.csv"
            code, _ = run_once(binary, parameters, scenario,
                               {**parse_sim(args.sim), **sim}, output, quiet=True)
            ok = code == 0
            detail = ""
            if ok:
                per_phase = summary(output, scenario)
                try:
                    ok = bool(predicate(per_phase))
                except (KeyError, IndexError):
                    ok = False
                stand = per_phase.get(0, {})
                detail = (f"stand pitch rms {stand.get('pitch_rms', math.nan):.3f} rad at "
                          f"{stand.get('pitch_peak_hz', math.nan):.2f} Hz")
            else:
                detail = "fell" if code == 3 else f"exit {code}"
            failures += 0 if ok else 1
            print(f"{'PASS' if ok else 'FAIL'}  {description}: {detail}", flush=True)
    print(f"{failures} of {len(gates())} checks failed" if failures else "all checks passed")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--mujoco-dir", help="directory with include/ and libmujoco.so* (default: the mujoco Python package)")
    parser.add_argument("--eigen-dir", default=os.environ.get("EIGEN_DIR", "/usr/include/eigen3"))
    parser.add_argument("--rebuild", action="store_true")
    sub = parser.add_subparsers(dest="command", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--params", action="append", default=[],
                        help="extra parameter YAML applied over controllers.yaml (like extra_params:=)")
    common.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                        help="one controller parameter, e.g. tasks.legs.pd_in_drive=0")
    common.add_argument("--sim", action="append", default=[], metavar="KEY=VALUE",
                        help=f"simulation option: {', '.join(SIM_OPTIONS)}")
    common.add_argument("--scenario", default="hardware", choices=sorted(SCENARIOS))
    run = sub.add_parser("run", parents=[common], help="one run, metrics per phase")
    run.add_argument("--output", help="recording CSV (default: a temporary file)")
    sweep = sub.add_parser("sweep", parents=[common], help="grid over simulation options and parameters")
    sweep.add_argument("--grid", action="append", required=True, metavar="KEY=V1,V2,...",
                       help="simulation option or controller parameter and its values")
    check = sub.add_parser("check", parents=[common],
                           help="the closed-loop gates (with sandro_real_overrides.yaml unless --params)")
    check.add_argument("--without-robot-overrides", action="store_true")
    args = parser.parse_args()
    binary = build(find_mujoco(args.mujoco_dir), Path(args.eigen_dir), args.rebuild)
    return {"run": command_run, "sweep": command_sweep, "check": command_check}[args.command](args, binary)


if __name__ == "__main__":
    raise SystemExit(main())
