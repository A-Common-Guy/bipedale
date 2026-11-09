"""Entry point: `ros2 run sandro_gui sandro_gui [--demo] [--hardware mujoco] [--port N]`."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
from pathlib import Path

import yaml

from .launcher import LaunchSettings
from .logbook import ERROR, Logbook, WARN

SOURCE = Path(__file__).resolve().parents[1]


def _share(package: str) -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package))
    except Exception:  # noqa: BLE001 - not installed, or no ROS at all
        return None


def resolve_paths(cfg: dict, cwd: Path) -> dict:
    gui = _share("sandro_gui") or SOURCE
    bringup = _share("sandro_bringup") or SOURCE.parent / "sandro_bringup"
    description = _share("sandro_description") or SOURCE.parent / "sandro_description"
    launch = cfg.get("launch", {})
    overrides = launch.get("overrides")
    return {
        "web": gui / "web",
        "teleop": bringup / "config" / "teleop.yaml",
        "controllers": bringup / "config" / "controllers.yaml",
        "overrides": str(bringup / "config" / overrides) if overrides else None,
        "mjcf": description / "mjcf" / cfg.get("model", {}).get("mjcf", "robot_full_floating.xml"),
        "ros2_control": description / "urdf" / "sandro.ros2_control.xacro",
        "cache": cwd / launch.get("runs_dir", "gui_runs") / "cache",
    }


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--ros-args" in argv:
        argv = argv[:argv.index("--ros-args")]
    parser = argparse.ArgumentParser(description="Sandro control panel")
    parser.add_argument("--config", help="gui.yaml (default: the installed one)")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--hardware", choices=("real", "mujoco"),
                        help="what Start launches (default: launch.hardware in gui.yaml)")
    parser.add_argument("--demo", action="store_true",
                        help="synthetic robot, no ROS or hardware needed")
    parser.add_argument("--print-guide", action="store_true",
                        help="print the gamepad table of docs/USER_GUIDE.md from teleop.yaml")
    args = parser.parse_args(argv)

    config = Path(args.config) if args.config else (_share("sandro_gui") or SOURCE) / "config" / "gui.yaml"
    cfg = yaml.safe_load(config.read_text()) or {}
    cwd = Path.cwd()
    paths = resolve_paths(cfg, cwd)
    if args.print_guide:
        from .gamepad import guide, guide_markdown
        print(guide_markdown(guide(yaml.safe_load(paths["teleop"].read_text()))), end="")
        return
    launch = cfg.get("launch", {})
    server_cfg = cfg.get("server", {})
    hardware = args.hardware or launch.get("hardware", "real")
    available = ("real", "mujoco") if args.demo or _share("mujoco_ros2_control") else ("real",)
    settings = LaunchSettings(
        hardware=hardware, args=list(launch.get("args", [])),
        simulation_args=list(launch.get("simulation_args", [])),
        overrides_path=paths["overrides"], stop_timeout_s=float(launch.get("stop_timeout_s", 20)),
        runs_dir=launch.get("runs_dir", "gui_runs"), keep_runs=int(launch.get("keep_runs", 30)),
        controllers=list(cfg.get("robot", {}).get("controllers", [])),
        available_hardware=available)
    teleop = yaml.safe_load(paths["teleop"].read_text())
    policies = teleop["joy_teleop"]["ros__parameters"].get("policies", [])
    log = Logbook(ignore=cfg.get("log", {}).get("ignore", []))

    if args.demo:
        from . import demo
        sources = demo.sources(log, settings, paths["controllers"], policies)
    else:
        from .bluetooth import Bluetooth
        from .can_monitor import CanMonitor, joint_mapping
        from .gamepad import JsReader
        from .launcher import Launcher
        from .system import SystemInfo
        launcher = Launcher(log, settings, str(cwd))
        can_cfg = cfg.get("can", {})
        interface = can_cfg.get("interface", "can0")
        spi = next((a.split(":=", 1)[1] for a in settings.args
                    if a.startswith("imu_spi_device:=")), "/dev/spidev6.0")
        sources = {"log": log, "launcher": launcher, "pad": JsReader(), "bt": Bluetooth(),
                   "system": SystemInfo(interface, [spi, "/dev/gpiochip0"])}
        if can_cfg.get("monitor", True) and hardware == "real":
            try:
                mapping = joint_mapping(paths["ros2_control"])
            except OSError as error:
                mapping = {}
                log.note(f"no joint mapping for the CAN monitor: {error}", WARN)
            sources["can"] = CanMonitor(interface, mapping)
        try:
            from .ros_bridge import RosBridge
            sources["ros"] = RosBridge(
                cfg, log, use_rosout=lambda: launcher.status()["state"] == "external",
                teleop_policies=policies)
        except ImportError as error:
            log.note(f"ROS 2 is not available ({error}); launch and status only", ERROR)

    from .panel import Panel
    from .server import serve
    panel = Panel(cfg, {k: str(v) if isinstance(v, Path) else v for k, v in paths.items()},
                  sources, simulation=hardware == "mujoco")
    host = args.host or server_cfg.get("host", "0.0.0.0")
    port = args.port or int(server_cfg.get("port", 8080))
    server = serve(panel, paths["web"], host, port, float(server_cfg.get("fast_rate_hz", 20)),
                   float(server_cfg.get("slow_rate_hz", 2)))
    print(f"sandro_gui: http://{host}:{port}  (hardware {hardware}"
          f"{', demo' if args.demo else ''}, pid {os.getpid()})", flush=True)

    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        server.server_close()
        ros = sources.get("ros")
        if ros is not None and hasattr(ros, "shutdown"):
            ros.shutdown()
        # The CAN, gamepad and Bluetooth threads sit in blocking system calls;
        # skip the interpreter teardown that would race them. A launch started
        # here runs in its own session and is not affected.
        sys.stdout.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
