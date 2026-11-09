"""HTTP side of the panel: static files, a JSON API and one Server-Sent Events stream.

The stream carries `fast` snapshots (joints, IMU, gamepad, rates) at
fast_rate_hz, `slow` ones (launch, controllers, CAN, system, checks) at
slow_rate_hz and new log lines. One ticker thread computes each snapshot
once, however many browsers are connected, and none while no browser is:
then it only drains the sampled topics and keeps the panel's session state
(panel.idle) current.
"""

from __future__ import annotations

import gzip
import json
import mimetypes
import threading
import time
import traceback
import zlib
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

from .logbook import ERROR

COMPRESSIBLE = (".html", ".js", ".css", ".json", ".svg", ".md", ".bin")


class Ticker:
    def __init__(self, panel, fast_hz: float, slow_hz: float) -> None:
        self.panel = panel
        self.fast_period, self.slow_period = 1.0 / fast_hz, 1.0 / slow_hz
        self.cond = threading.Condition()
        self.fast = self.slow = "{}"
        self.fast_seq = self.slow_seq = 0
        self.clients = 0
        self._demand = 0.0
        self._reported: dict[str, float] = {}
        threading.Thread(target=self._run, name="ticker", daemon=True).start()

    def attach(self) -> None:
        with self.cond:
            self.clients += 1

    def detach(self) -> None:
        with self.cond:
            self.clients -= 1

    def fresh(self, timeout: float = 1.0) -> tuple[str, str]:
        """The snapshots for /api/state, built now if no stream keeps them current."""
        with self.cond:
            if self.clients == 0:
                self._demand = time.monotonic() + 5.0
                seq = self.slow_seq
                self.cond.wait_for(lambda: self.slow_seq != seq, timeout=timeout)
            return self.fast, self.slow

    def _report(self, error: Exception) -> None:
        key = f"{type(error).__name__}: {error}"
        now = time.monotonic()
        if now - self._reported.get(key, -1e9) < 60.0:
            return
        if len(self._reported) > 50:
            self._reported.clear()
        self._reported[key] = now
        self.panel.log.note(f"snapshot failed: {error} (repeats are not logged for a minute)",
                            ERROR)
        traceback.print_exc()

    def _run(self) -> None:
        next_slow = 0.0
        while True:
            started = time.monotonic()
            try:
                self.panel.sample()
                if self.clients == 0 and started > self._demand:
                    if started >= next_slow:
                        next_slow = started + self.slow_period
                        self.panel.idle()
                    time.sleep(max(0.0, self.fast_period - (time.monotonic() - started)))
                    continue
                fast = self.panel.fast()
                fast_json = json.dumps(_finite(fast), default=_jsonable, separators=(",", ":"))
                slow_json = None
                if started >= next_slow:
                    next_slow = started + self.slow_period
                    slow = self.panel.slow()
                    slow["checks"] = self.panel.checks(slow, fast)
                    slow_json = json.dumps(_finite(slow), default=_jsonable,
                                           separators=(",", ":"))
            except Exception as error:  # noqa: BLE001 - keep ticking
                self._report(error)
                time.sleep(1.0)
                continue
            with self.cond:
                self.fast, self.fast_seq = fast_json, self.fast_seq + 1
                if slow_json is not None:
                    self.slow, self.slow_seq = slow_json, self.slow_seq + 1
                self.cond.notify_all()
            time.sleep(max(0.0, self.fast_period - (time.monotonic() - started)))


def _finite(value):
    """NaN and infinities become null: JSON has neither."""
    if isinstance(value, float):
        return value if value == value and abs(value) != float("inf") else None
    if isinstance(value, dict):
        return {k: _finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite(v) for v in value]
    return value


def _jsonable(value):
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "tolist"):
        return value.tolist()
    return str(value)


class Handler(BaseHTTPRequestHandler):
    server_version = "sandro_gui"
    panel = None
    ticker: Ticker = None
    web: Path = None
    _cache: dict = {}

    def log_message(self, fmt, *args):  # quiet: the panel has its own log
        pass

    # Helpers ------------------------------------------------------------

    def _send(self, status: int, body: bytes, kind: str, extra: dict | None = None) -> None:
        self.send_response(status)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, value, status: int = 200) -> None:
        body = json.dumps(_finite(value), default=_jsonable).encode()
        self._send(status, body, "application/json", {"Cache-Control": "no-store"})

    def _gzip_ok(self) -> bool:
        return "gzip" in self.headers.get("Accept-Encoding", "")

    def _blob(self, key: str, data, kind: str, etag: str) -> None:
        """`data` is the bytes, or a callable that reads them only when the ETag missed."""
        if self.headers.get("If-None-Match") == etag:
            self.send_response(HTTPStatus.NOT_MODIFIED)
            self.send_header("ETag", etag)
            self.end_headers()
            return
        headers = {"ETag": etag, "Cache-Control": "no-cache"}
        if self._gzip_ok() and key.endswith(COMPRESSIBLE):
            cached = self._cache.get(key)
            if cached is None or cached[0] != etag:
                cached = (etag, gzip.compress(data() if callable(data) else data, 6))
                self._cache[key] = cached
            data = cached[1]
            headers["Content-Encoding"] = "gzip"
        elif callable(data):
            data = data()
        self._send(200, data, kind, headers)

    def _static(self, path: str) -> None:
        relative = Path("index.html" if path in ("", "/") else unquote(path).lstrip("/"))
        # Checked on the request, not the resolved path: a --symlink-install
        # links every file back into the source tree.
        file = self.web / relative
        if relative.is_absolute() or ".." in relative.parts or not file.is_file():
            self._send(404, b"not found", "text/plain")
            return
        stat = file.stat()
        kind = mimetypes.guess_type(file.name)[0] or "application/octet-stream"
        if file.suffix == ".js":
            kind = "text/javascript"
        self._blob(str(file), file.read_bytes, kind + ("; charset=utf-8" if kind.startswith("text")
                                                       else ""),
                   f'"{stat.st_mtime_ns:x}-{stat.st_size:x}"')

    def _body(self) -> dict:
        length = int(self.headers.get("Content-Length") or 0)
        if not length:
            return {}
        return json.loads(self.rfile.read(length))

    # Routes -------------------------------------------------------------

    def do_HEAD(self):
        self.do_GET()

    def do_GET(self):
        url = urlparse(self.path)
        query = {k: v[-1] for k, v in parse_qs(url.query).items()}
        path = url.path
        try:
            if path == "/api/stream":
                return self._stream(int(query.get("after", self.panel.log.seq)))
            if path == "/api/info":
                return self._json(self.panel.info())
            if path == "/api/state":
                fast, slow = self.ticker.fresh()
                return self._json({"fast": json.loads(fast), "slow": json.loads(slow)})
            if path == "/api/logs":
                return self._json(self.panel.log.since(int(query.get("after", 0)),
                                                        int(query.get("limit", 2000))))
            if path == "/api/model":
                built, error = self.panel.model()
                if built is None:
                    return self._json({"ready": False, "error": error}, 503 if error else 202)
                return self._json(dict(built[0], ready=True))
            if path == "/api/model.bin":
                built, _ = self.panel.model()
                if built is None:
                    return self._send(503, b"", "application/octet-stream")
                return self._blob("model.bin", built[1], "application/octet-stream",
                                  f'"{len(built[1]):x}-{zlib.crc32(built[1]):x}"')
            if path == "/api/params":
                return self._json(self.panel._ros().parameters(query["node"]))
            if path == "/api/params/export":
                text = self.panel._ros().changed_yaml().encode()
                return self._send(200, text, "text/yaml; charset=utf-8",
                                  {"Content-Disposition": 'attachment; filename="tuned.yaml"'})
            if path == "/api/runs":
                return self._json(self.panel.launcher.runs_list())
            if path.startswith("/api/runs/"):
                file = self.panel.launcher.run_path(path.rsplit("/", 1)[-1])
                if file is None:
                    return self._send(404, b"no such run", "text/plain")
                return self._send(200, file.read_bytes(), "text/plain; charset=utf-8",
                                  {"Content-Disposition": f'attachment; filename="{file.name}"'})
            if path.startswith("/api/"):
                return self._json({"error": "unknown endpoint"}, 404)
            return self._static(path)
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as error:  # noqa: BLE001 - answer every request
            return self._json({"error": str(error)}, 500)

    def do_POST(self):
        path = urlparse(self.path).path
        if not path.startswith("/api/"):
            return self._json({"error": "unknown endpoint"}, 404)
        action = path[len("/api/"):]
        try:
            body = self._body()
            result = self.panel.act(action, body)
            if isinstance(result, dict) and result.get("ok") is False:
                return self._json({"error": result.get("message") or "failed"}, 400)
            return self._json(dict(result or {}, ok=True))
        except KeyError as error:
            return self._json({"error": f"unknown action {error}"}, 404)
        except (ValueError, TypeError) as error:
            return self._json({"error": str(error)}, 400)
        except Exception as error:  # noqa: BLE001
            payload = {"error": str(error)}
            if hasattr(error, "running"):
                payload["conflict"] = error.running
            return self._json(payload, getattr(error, "http_status", 500))

    def _stream(self, after: int) -> None:
        # One deflate stream per connection, flushed at every event: the
        # snapshots repeat their keys, so later events cost a fraction.
        packer = zlib.compressobj(5, zlib.DEFLATED, 31) if self._gzip_ok() else None
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Accel-Buffering", "no")
        if packer is not None:
            self.send_header("Content-Encoding", "gzip")
        self.end_headers()
        ticker, log = self.ticker, self.panel.log
        ticker.attach()
        # The held snapshots may be from before an idle spell: start with the next ones.
        fast_seq, slow_seq = ticker.fast_seq, ticker.slow_seq
        last_write = time.monotonic()
        try:
            while True:
                with ticker.cond:
                    ticker.cond.wait_for(lambda: ticker.fast_seq != fast_seq, timeout=2.0)
                    fast, fast_new = ticker.fast, ticker.fast_seq
                    slow, slow_new = ticker.slow, ticker.slow_seq
                chunks = []
                if fast_new != fast_seq:
                    fast_seq = fast_new
                    chunks.append(f"event: fast\ndata: {fast}\n\n")
                if slow_new != slow_seq:
                    slow_seq = slow_new
                    chunks.append(f"event: slow\ndata: {slow}\n\n")
                if log.seq > after:
                    entries = log.since(after, 500)
                    after = entries[-1]["seq"] if entries else log.seq
                    chunks.append(f"event: log\ndata: {json.dumps(entries)}\n\n")
                if not chunks and time.monotonic() - last_write > 10:
                    chunks.append(": keepalive\n\n")
                if chunks:
                    data = "".join(chunks).encode()
                    if packer is not None:
                        data = packer.compress(data) + packer.flush(zlib.Z_SYNC_FLUSH)
                    self.wfile.write(data)
                    self.wfile.flush()
                    last_write = time.monotonic()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            return
        finally:
            ticker.detach()


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def serve(panel, web: Path, host: str, port: int, fast_hz: float, slow_hz: float) -> Server:
    ticker = Ticker(panel, fast_hz, slow_hz)
    handler = type("PanelHandler", (Handler,), {"panel": panel, "ticker": ticker, "web": web,
                                                "_cache": {}})
    return Server((host, port), handler)
