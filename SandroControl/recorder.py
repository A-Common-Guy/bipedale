"""Non-blocking control-rate recorder for system-identification trials."""
from __future__ import annotations

import csv
import json
import queue
import threading
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from models import MitCommand, RobotState


class RunRecorder:
    """Move CSV and metadata I/O off the real-time control thread."""

    def __init__(self, output_dir: Path, motor_names, queue_size: int = 8192):
        self.output_dir = Path(output_dir)
        self.motor_names = tuple(motor_names)
        self._queue_size = int(queue_size)
        self._queue: queue.Queue = queue.Queue(maxsize=self._queue_size)
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._active = False
        self._dropped = 0
        self._rows = 0
        self._run_id = ""
        self._csv_path: Path | None = None
        self._metadata_path: Path | None = None
        self._metadata: dict = {}

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def start(self, metadata: dict) -> str:
        with self._lock:
            if self._active:
                raise RuntimeError("a recording is already active")
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
            self._run_id = f"sysid_{stamp}"
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self._csv_path = self.output_dir / f"{self._run_id}.csv"
            self._metadata_path = self.output_dir / f"{self._run_id}.json"
            self._metadata = dict(metadata)
            self._metadata.update(
                run_id=self._run_id,
                started_utc=datetime.now(timezone.utc).isoformat(),
                csv=str(self._csv_path),
                status="recording",
            )
            self._queue = queue.Queue(maxsize=self._queue_size)
            self._dropped = 0
            self._rows = 0
            self._active = True
            self._thread = threading.Thread(
                target=self._writer,
                daemon=True,
                name="sysid-recorder",
            )
            self._thread.start()
            return self._run_id

    def record(
        self,
        timestamp: float,
        state: RobotState,
        commands: dict[str, MitCommand],
        experiment: dict,
        metrics: dict,
    ) -> None:
        if not self.active:
            return
        row = {
            "time_monotonic_s": timestamp,
            "state_timestamp_s": state.timestamp,
            "experiment_state": experiment.get("state", ""),
            "experiment_segment": experiment.get("segment", ""),
            "cycle_ms": metrics.get("cycle_ms", 0.0),
            "overruns": metrics.get("overruns", 0),
            "imu_online": int(state.imu.online),
            "imu_age_s": state.imu.age_s,
        }
        for index, value in enumerate(state.imu.quaternion):
            row[f"imu_q{index}"] = value
        for index, value in enumerate(state.imu.gyro):
            row[f"imu_gyro_{index}"] = value
        for index, value in enumerate(state.imu.accel):
            row[f"imu_accel_{index}"] = value
        for name in self.motor_names:
            motor = state.motors.get(name)
            command = commands.get(name, MitCommand())
            prefix = f"{name}."
            for field, value in asdict(command).items():
                row[f"{prefix}cmd_{field}"] = value
            if motor is None:
                continue
            row.update(
                {
                    f"{prefix}position": motor.position,
                    f"{prefix}velocity": motor.velocity,
                    f"{prefix}torque": motor.torque,
                    f"{prefix}temperature": motor.temperature,
                    f"{prefix}online": int(motor.online),
                    f"{prefix}error": motor.error,
                    f"{prefix}age_s": motor.age_s,
                    f"{prefix}iq": motor.iq,
                    f"{prefix}mit_position": motor.mit_position,
                    f"{prefix}mit_velocity": motor.mit_velocity,
                    f"{prefix}mit_torque": motor.mit_torque,
                    f"{prefix}vbus": motor.vbus,
                }
            )
        try:
            self._queue.put_nowait(row)
        except queue.Full:
            with self._lock:
                self._dropped += 1

    def stop(self, status: str, reason: str = "") -> None:
        with self._lock:
            if not self._active:
                return
            self._active = False
            self._metadata["final_status"] = status
            self._metadata["reason"] = reason
        # A full queue must never block the control thread. The writer exits
        # after draining once it observes the stop event encoded as None.
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            threading.Thread(
                target=self._enqueue_stop,
                args=(status, reason),
                daemon=True,
            ).start()
            return

    def _enqueue_stop(self, status: str, reason: str) -> None:
        self._queue.put(None)

    def _writer(self) -> None:
        assert self._csv_path is not None
        first = self._queue.get()
        if first is None:
            self._finalize_metadata()
            return
        with self._csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(first))
            writer.writeheader()
            writer.writerow(first)
            self._rows = 1
            while True:
                row = self._queue.get()
                if row is None:
                    break
                writer.writerow(row)
                self._rows += 1
        self._finalize_metadata()

    def _finalize_metadata(self) -> None:
        self._metadata.update(
            status=self._metadata.get("final_status", "stopped"),
            reason=self._metadata.get("reason", ""),
            rows=self._rows,
            dropped_rows=self._dropped,
            stopped_utc=datetime.now(timezone.utc).isoformat(),
            duration_wall_s=(
                time.time()
                - datetime.fromisoformat(self._metadata["started_utc"]).timestamp()
            ),
        )
        assert self._metadata_path is not None
        self._metadata_path.write_text(
            json.dumps(self._metadata, indent=2, default=str),
            encoding="utf-8",
        )

    def status(self) -> dict:
        with self._lock:
            return {
                "active": self._active,
                "run_id": self._run_id,
                "path": str(self._csv_path) if self._csv_path else "",
                "rows": self._rows,
                "dropped_rows": self._dropped,
                "queue_depth": self._queue.qsize(),
            }

    def close(self) -> None:
        self.stop("shutdown", "runtime shutdown")
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout=2.0)
