#!/usr/bin/env python3
"""SandroControl: decoupled MIT controller, MuJoCo twin and web dashboard."""
from __future__ import annotations

import argparse
import asyncio
import importlib
import json
import logging
import os
import sys
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from config import load_config
from hardware_backend import HardwareBackend
from imu import Bno085UartShtp
from mujoco_backend import MujocoBackend
from runtime import ControlRuntime

HERE = Path(__file__).resolve().parent
log = logging.getLogger("sandro")

# EGL reaches the discrete GPU headlessly; GLFW needs an X display and usually
# lands on the integrated GPU. Offscreen frames are several times cheaper on
# the former, so prefer it and fall back only when its context cannot start.
GL_BACKENDS = ("egl", "glfw")

# aiohttp drains in-flight handlers before it runs on_cleanup, and its default
# allowance is 60 s. Websocket handlers never finish on their own, so an open
# dashboard used to hold the drives commanded for that whole window after
# Ctrl+C. Handlers are closed explicitly in on_shutdown now; this bound only
# covers a connection that still refuses to unwind.
SHUTDOWN_TIMEOUT_S = 3.0
# Last resort once the drives are already idle and the bus is closed: a wedged
# driver thread must never leave the operator reaching for kill -9.
EXIT_DEADLINE_S = 5.0


def select_gl_backend() -> str:
    """Bind the cheapest working offscreen GL context before MuJoCo loads.

    An explicit ``MUJOCO_GL`` is always honoured. Each candidate is proven by
    creating a real 1x1 context, because an unusable driver only fails when a
    context is constructed, not when its module is imported.
    """
    requested = os.environ.get("MUJOCO_GL", "").strip().lower()
    candidates = [requested] if requested else list(GL_BACKENDS)
    os.environ["MUJOCO_GL"] = candidates[0]
    from mujoco.rendering.classic import gl_context

    for name in candidates:
        try:
            module = importlib.import_module(f"mujoco.{name}")
            probe = module.GLContext(1, 1)
            probe.free()
        except Exception as exc:
            log.warning("GL backend %s unavailable: %s", name, exc)
            continue
        gl_context.GLContext = module.GLContext
        os.environ["MUJOCO_GL"] = name
        return name
    raise RuntimeError(f"no usable GL backend among {candidates}")


def load_controller(spec: str, cfg: dict):
    """Instantiate ``module:Class`` with the loaded configuration."""
    if str(HERE) not in sys.path:
        sys.path.insert(0, str(HERE))
    try:
        module_name, class_name = spec.rsplit(":", 1)
    except ValueError as exc:
        raise ValueError("--controller must use module:Class syntax") from exc
    if not module_name or not class_name:
        raise ValueError("--controller must use module:Class syntax")
    controller_type = getattr(importlib.import_module(module_name), class_name)
    controller = controller_type(cfg)
    if not callable(getattr(controller, "reset", None)) or not callable(
        getattr(controller, "update", None)
    ):
        raise TypeError(
            f"{spec} must provide reset(state) and update(state, target, dt)"
        )
    return controller


def build_system(cfg: dict, controller=None):
    mode = cfg["runtime"]["mode"]
    if mode == "both" and cfg["runtime"]["state_source"] != "physical":
        raise ValueError("both mode must use physical state_source for hardware safety")
    backends = []
    try:
        simulation = None
        if mode in {"simulation", "both"}:
            simulation = MujocoBackend(cfg)
            backends.append(simulation)
        if mode in {"physical", "both"}:
            backends.append(HardwareBackend(cfg))
        visualizer = simulation
        if visualizer is None:
            visualizer = MujocoBackend(cfg)
            visualizer.start()
        imu_cfg = cfg["imu"]
        imu = (
            Bno085UartShtp(
                imu_cfg["port"],
                int(imu_cfg["baudrate"]),
                float(imu_cfg["timeout_s"]),
                int(imu_cfg["report_interval_us"]),
            )
            if imu_cfg["enabled"]
            else None
        )
        return (
            ControlRuntime(cfg, backends, imu, controller=controller),
            visualizer,
            simulation is None,
        )
    except Exception:
        # A half-built system still owns the CAN socket and its RX thread, and
        # nothing downstream has been registered to release them.
        for backend in backends:
            try:
                backend.close()
            except Exception:
                log.exception("%s: close failed while unwinding startup", backend.name)
        raise


def make_web_app(runtime, visualizer, separate_visualizer, cfg):
    from aiohttp import WSCloseCode, WSMsgType, web

    app = web.Application()
    clients = set()
    render_executor = ThreadPoolExecutor(
        max_workers=1, thread_name_prefix="mujoco-render"
    )

    async def index(_request):
        return web.FileResponse(
            HERE / "dashboard.html", headers={"Cache-Control": "no-store"}
        )

    async def frame(_request):
        if separate_visualizer:
            visualizer.mirror_state(runtime.selected_state())
        loop = asyncio.get_running_loop()
        image = await loop.run_in_executor(
            render_executor, visualizer.render_jpeg
        )
        return web.Response(
            body=image,
            content_type="image/jpeg",
            headers={"Cache-Control": "no-store, max-age=0"},
        )

    def dispatch(message: dict) -> dict:
        kind = message.get("type")
        try:
            if kind == "enable":
                text = runtime.enable()
            elif kind == "disable":
                text = runtime.disable()
            elif kind == "estop":
                text = runtime.estop()
            elif kind == "joint_target":
                runtime.set_joint_targets(message["values"])
                text = "joint target accepted"
            elif kind == "cartesian_target":
                runtime.set_cartesian_target(
                    message["side"], message["x"], message["y"]
                )
                text = "Cartesian target accepted"
            elif kind == "wheel_torque":
                runtime.set_wheel_torques(message["values"])
                text = "wheel torque accepted"
            elif kind == "drive_target":
                runtime.set_drive_target(
                    message["forward_velocity_m_s"],
                    message["yaw_rate_rad_s"],
                )
                text = "drive target accepted"
            elif kind == "gains":
                runtime.set_gains(message["values"])
                text = "gains updated"
            elif kind == "balance":
                runtime.set_balance(message["values"])
                text = "balance parameters updated"
            elif kind == "drive":
                runtime.set_drive(message["values"])
                text = "drive parameters updated"
            elif kind == "sysid_arm":
                status = runtime.arm_sysid(message["values"])
                text = (
                    f"sys-ID armed: {status['preset']} on {status['axis']} "
                    f"at {status['amplitude']}"
                )
            elif kind == "sysid_start":
                text = f"sys-ID recording started: {runtime.start_sysid()}"
            elif kind == "sysid_abort":
                text = runtime.abort_sysid()
            elif kind == "hanged":
                visualizer.set_hanged(bool(message["enabled"]))
                text = "hanged mode enabled" if message["enabled"] else "free base enabled"
            elif kind == "camera":
                visualizer.move_camera(
                    str(message["action"]),
                    float(message.get("dx", 0.0)),
                    float(message.get("dy", 0.0)),
                )
                text = "camera moved"
            elif kind == "camera_reset":
                visualizer.reset_camera()
                text = "camera reset"
            else:
                raise ValueError(f"unknown command {kind!r}")
            return {
                "type": "ack",
                "message": text,
                "silent": kind in {
                    "joint_target",
                    "cartesian_target",
                    "wheel_torque",
                    "drive_target",
                    "camera",
                },
            }
        except Exception as exc:
            return {"type": "error", "message": str(exc)}

    async def websocket(request):
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        clients.add(ws)
        await ws.send_json(
            {
                "type": "config",
                "runtime_mode": cfg["runtime"]["mode"],
                "render_hz": cfg["runtime"]["render_hz"],
                "hanged": visualizer.hanged,
                "motors": cfg["robot"]["motors"],
                "legs": cfg["robot"]["legs"],
                "balance": cfg["balance"],
                "drive": cfg["drive"],
                "sysid_presets": runtime.sysid.presets(),
            }
        )
        loop = asyncio.get_running_loop()
        try:
            async for item in ws:
                if item.type != WSMsgType.TEXT:
                    continue
                try:
                    message = json.loads(item.data)
                except json.JSONDecodeError:
                    await ws.send_json({"type": "error", "message": "invalid JSON"})
                    continue
                reply = await loop.run_in_executor(None, dispatch, message)
                if not reply.get("silent") or reply["type"] == "error":
                    await ws.send_json(reply)
        finally:
            clients.discard(ws)
            if not clients and runtime.sysid.running:
                runtime.abort_sysid("dashboard disconnected")
            if not clients:
                stop_drive = getattr(runtime, "set_drive_target", None)
                if callable(stop_drive):
                    stop_drive(0.0, 0.0)
        return ws

    async def broadcast():
        period = 1.0 / float(cfg["runtime"]["telemetry_hz"])
        while True:
            await asyncio.sleep(period)
            if not clients:
                continue
            state = runtime.snapshot()
            for ws in list(clients):
                try:
                    await ws.send_json(state)
                except Exception:
                    clients.discard(ws)

    async def startup(_app):
        runtime.start()
        app["broadcast"] = asyncio.create_task(broadcast())

    async def stop_broadcast():
        task = app.get("broadcast")
        if task is None:
            return
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)

    async def shutdown(_app):
        """Reach the safe state before aiohttp drains open connections.

        on_shutdown runs ahead of that drain, on_cleanup after it. Safing the
        drives from on_cleanup therefore kept them commanded for as long as a
        dashboard websocket stayed open, which is the whole shutdown_timeout.
        """
        await stop_broadcast()
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, runtime.stop)
        await asyncio.gather(
            *(
                ws.close(
                    code=WSCloseCode.GOING_AWAY, message=b"controller shutting down"
                )
                for ws in list(clients)
            ),
            return_exceptions=True,
        )
        clients.clear()

    async def cleanup(_app):
        # Idempotent, and repeated on purpose: on_shutdown is skipped when a
        # second Ctrl+C interrupts the drain.
        await stop_broadcast()
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, runtime.stop)
        await loop.run_in_executor(render_executor, visualizer.close_renderer)
        render_executor.shutdown(wait=True)
        await loop.run_in_executor(None, runtime.close)
        if separate_visualizer:
            visualizer.close()

    app.router.add_get("/", index)
    app.router.add_get("/ws", websocket)
    app.router.add_get("/api/frame.jpg", frame)
    app.on_startup.append(startup)
    app.on_shutdown.append(shutdown)
    app.on_cleanup.append(cleanup)
    return app


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=HERE / "config.yaml", help="YAML configuration"
    )
    parser.add_argument(
        "--mode", choices=("simulation", "physical", "both"), help="override YAML mode"
    )
    parser.add_argument(
        "--controller",
        metavar="MODULE:CLASS",
        help="load a Python controller class instead of the manual controller",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    cfg = load_config(args.config)
    if args.mode:
        cfg["runtime"]["mode"] = args.mode
    gl_backend = select_gl_backend()
    controller = load_controller(args.controller, cfg) if args.controller else None
    runtime, visualizer, separate = build_system(cfg, controller=controller)
    from aiohttp import web

    server = cfg["server"]
    log.info(
        "mode=%s control=%g Hz controller=%s gl=%s dashboard=http://%s:%s",
        cfg["runtime"]["mode"],
        cfg["runtime"]["control_hz"],
        type(runtime.controller).__name__,
        gl_backend,
        server["host"],
        server["port"],
    )
    try:
        web.run_app(
            make_web_app(runtime, visualizer, separate, cfg),
            host=server["host"],
            port=int(server["port"]),
            print=None,
            access_log=None,
            shutdown_timeout=SHUTDOWN_TIMEOUT_S,
        )
    finally:
        # A second Ctrl+C raises straight through aiohttp's cleanup await, so
        # the handlers above may never have run. Both calls are idempotent.
        runtime.stop()
        _arm_exit_watchdog()
        runtime.close()
    return 0


def _arm_exit_watchdog() -> None:
    """Guarantee the process dies now that the drives are idle.

    MuJoCo's GL driver, pyserial and python-can all park threads that can
    outlive the last statement of main(). Force the exit rather than leave a
    half-dead controller holding the CAN socket.
    """

    def force_exit() -> None:
        log.error(
            "shutdown still incomplete %.1fs after safing the drives; forcing exit",
            EXIT_DEADLINE_S,
        )
        os._exit(0)

    watchdog = threading.Timer(EXIT_DEADLINE_S, force_exit)
    watchdog.daemon = True
    watchdog.start()


if __name__ == "__main__":
    raise SystemExit(main())
