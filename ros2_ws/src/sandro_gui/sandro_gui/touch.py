"""The Drive page's on-screen sticks as a gamepad: what a phone sends becomes /joy.

A browser posts its sticks and held buttons a dozen times a second
(/api/touch). While those posts are fresh this publishes them as the
sensor_msgs/Joy a DualSense would send (SDL's layout, teleop.yaml's indices),
20 times a second, so joy_teleop does with them exactly what it does with the
pad: the deadman, the stick limits of the running controller, the hop, the
policy switch. Nothing else in the stack knows a phone is driving.

What keeps it safe when the phone is not there any more:

- a post older than `timeout_s` (0.35 s: a dropped hotspot, a locked screen, a
  browser in the background) ends the driving. A second of released pads
  follows, so joy_teleop sees the deadman let go at once, then nothing: its
  own joy_timeout_s and the controllers' command timeout would catch it too;
- one browser drives at a time; another is refused until the first has let go;
- a connected gamepad has priority: with one delivering input the phone is
  refused, and two sources never alternate on /joy.

A routine (routines.py, config/routines.yaml) is the same thing with the
sticks moved by the panel: the page posts which routine its finger holds, and
the same rules stop it. It starts from zero on every new press.

The E-stop is not here: the panel's own button (CAN frame and service) is on
every page.
"""

from __future__ import annotations

import threading
import time

from .gamepad import SDL_AXES, SDL_BUTTONS

# What a phone may press: the hop, the policy switch, the height reset, the announce.
BUTTONS = ("cross", "triangle", "r3", "circle")
DEADMAN = "l1"


def _unit(value) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0
    return max(-1.0, min(1.0, value)) if value == value else 0.0


def to_joy(state: dict | None, speed: float = 1.0) -> tuple[list[float], list[int]]:
    """Axes and buttons of a sensor_msgs/Joy for a named state (gamepad.from_joy's inverse).

    state: lx, ly, rx, ry in -1..1 (right and up positive), `hold` (the deadman) and
    `buttons`, a list of names. `speed` scales the left stick (forward and turn) only.
    None is a pad with everything released.
    """
    axes = [0.0] * len(SDL_AXES)
    axes[4] = axes[5] = 1.0                       # triggers released: never the E-stop chord
    buttons = [0] * len(SDL_BUTTONS)
    if state is None or not state.get("hold"):
        return axes, buttons
    speed = max(0.0, min(1.0, float(speed)))
    axes[0] = -_unit(state.get("lx")) * speed     # SDL: left is +1
    axes[1] = _unit(state.get("ly")) * speed
    axes[2] = -_unit(state.get("rx"))
    axes[3] = _unit(state.get("ry"))
    buttons[SDL_BUTTONS.index(DEADMAN)] = 1
    for name in state.get("buttons") or ():
        if name in BUTTONS:
            buttons[SDL_BUTTONS.index(name)] = 1
    return axes, buttons


class TouchPad:
    def __init__(self, cfg: dict, publish, gamepad_live=lambda: False, clock=time.monotonic,
                 thread: bool = True, routines: dict | None = None) -> None:
        """publish(axes, buttons) sends one Joy; gamepad_live() is true while a real pad
        delivers input. `thread` False leaves the 20 Hz loop to the caller (tick()).
        `routines` is routines.load()'s result: what the page may ask to be played."""
        self.routines = routines or {}
        self.started = 0.0                        # when the routine being played began
        self.timeout_s = float(cfg.get("timeout_s", 0.35))
        self.release_s = float(cfg.get("release_s", 1.0))
        self.max_speed = max(0.0, min(1.0, float(cfg.get("max_speed_fraction", 1.0))))
        self.rate_hz = float(cfg.get("rate_hz", 20.0))
        self.publish, self.gamepad_live, self.clock = publish, gamepad_live, clock
        self.lock = threading.Lock()
        self.client: str | None = None
        self.state: dict | None = None
        self.stamp = -1e9
        self.released_at: float | None = None     # since when released pads are being sent
        self.error: str | None = None
        if thread:
            threading.Thread(target=self._run, name="touch", daemon=True).start()

    def update(self, body: dict) -> dict:
        """One post of the Drive page. Raises ValueError when this browser may not drive."""
        client = str(body.get("client") or "")
        if not client:
            raise ValueError("touch needs a client id")
        now = self.clock()
        state = {"lx": _unit(body.get("lx")), "ly": _unit(body.get("ly")),
                 "rx": _unit(body.get("rx")), "ry": _unit(body.get("ry")),
                 "hold": bool(body.get("hold")),
                 "buttons": [b for b in body.get("buttons") or [] if b in BUTTONS],
                 "speed": max(0.0, min(self.max_speed, float(body.get("speed", self.max_speed)))),
                 "routine": None, "play": None}
        if state["hold"] and body.get("routine"):
            if body["routine"] not in self.routines:
                raise ValueError(f"no routine {body['routine']!r}")
            state.update(routine=str(body["routine"]), play=body.get("play"))
        with self.lock:
            fresh = now - self.stamp < self.timeout_s
            if fresh and self.client != client and self.state and self.state["hold"]:
                raise ValueError("another browser is driving")
            if state["hold"] and self.gamepad_live():
                raise ValueError("a gamepad is connected and has priority: disconnect it to drive from here")
            before = self.state if fresh and self.state and self.state["hold"] else None
            if state["routine"] and (before is None or (before["routine"], before["play"])
                                     != (state["routine"], state["play"])):
                self.started = now                # a new press: from the beginning
            self.client, self.state, self.stamp = client, state, now
            if state["hold"]:
                self.released_at = None
        return {"driving": state["hold"]}

    def _playing(self, now: float) -> dict | None:
        """The held routine's progress (lock held); None when no routine is held."""
        state = self.state
        if not (state and state["hold"] and state["routine"] and now - self.stamp < self.timeout_s):
            return None
        duration = self.routines[state["routine"]].duration
        t = now - self.started
        return {"name": state["routine"], "t": min(t, duration), "duration_s": duration,
                "done": t >= duration}

    def status(self) -> dict:
        now = self.clock()
        with self.lock:
            fresh = now - self.stamp < self.timeout_s
            return {"driving": bool(fresh and self.state and self.state["hold"]),
                    "routine": self._playing(now),
                    "client": self.client if fresh else None,
                    "age_s": now - self.stamp if self.state else None,
                    "max_speed": self.max_speed, "error": self.error}

    def tick(self) -> None:
        """One 20 Hz step: the phone's pad while it holds, released pads for release_s after."""
        now = self.clock()
        with self.lock:
            holding = self.state is not None and self.state["hold"] and now - self.stamp < self.timeout_s
            if holding and self.state["routine"]:
                # The panel moves the sticks; once the routine is over, a released pad
                # for as long as the finger stays.
                sticks = self.routines[self.state["routine"]].sample(now - self.started)
                message = to_joy(sticks, self.state["speed"])
            elif holding:
                message = to_joy(self.state, self.state["speed"])
            elif self.state is not None:
                if self.released_at is None:
                    self.released_at = now
                if now - self.released_at > self.release_s:
                    self.state = None             # silent again: /joy is the gamepad's
                    return
                message = to_joy(None)
            else:
                return
        try:
            self.publish(*message)
            self.error = None
        except Exception as error:  # noqa: BLE001 - a publisher that is gone must not end the loop
            self.error = str(error)

    def _run(self) -> None:
        period = 1.0 / self.rate_hz
        while True:
            started = time.monotonic()
            self.tick()
            time.sleep(max(0.0, period - (time.monotonic() - started)))
