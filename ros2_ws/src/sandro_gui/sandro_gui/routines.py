"""Routines of the Drive page: a timed sequence of stick positions and buttons.

config/routines.yaml holds them, in the units the page posts (touch.py):

    routines:
      spin:
        label: Spin
        note: what it does and the room it needs
        keys:                       # the sticks, by time; a stick left out is 0
          - {t: 0.0}
          - {t: 0.5, lx: 1.0}       # linear from the key before
          - {t: 5.0, lx: 1.0}
          - {t: 5.5}
        buttons:                    # presses, each held for `for` seconds
          - {t: 1.0, for: 0.3, button: cross}

lx, ly, rx, ry are -1..1, right and up positive: ly forward, lx right turns
right, ry up stands taller (a height RATE in joy_teleop), rx right puts the
right side down. A key with `hold: true` keeps its values until the next key
instead of blending into it. The routine lasts until its last key, where the
sticks must be back at zero. A routine may press the hop (cross) and the
height reset (r3) and nothing else.

The panel plays a routine (touch.py), not the browser: the phone only says
which one it holds.
"""

from __future__ import annotations

from pathlib import Path

import yaml

STICKS = ("lx", "ly", "rx", "ry")
BUTTONS = ("cross", "r3")


class Routine:
    def __init__(self, name: str, spec: dict) -> None:
        self.name = name
        self.label = str(spec.get("label") or name)
        self.note = str(spec.get("note") or "")
        self.keys = []
        for key in spec.get("keys") or []:
            unknown = set(key) - {"t", "hold", *STICKS}
            if unknown:
                raise ValueError(f"routine {name}: unknown key field {sorted(unknown)}")
            values = tuple(max(-1.0, min(1.0, float(key.get(s, 0.0)))) for s in STICKS)
            self.keys.append((float(key["t"]), values, bool(key.get("hold"))))
        if len(self.keys) < 2 or self.keys[0][0] != 0.0:
            raise ValueError(f"routine {name}: needs keys from t 0")
        if any(b[0] <= a[0] for a, b in zip(self.keys, self.keys[1:])):
            raise ValueError(f"routine {name}: key times must increase")
        if any(self.keys[-1][1]):
            raise ValueError(f"routine {name}: the sticks must end at zero")
        self.duration = self.keys[-1][0]
        self.presses = []
        for press in spec.get("buttons") or []:
            if press.get("button") not in BUTTONS:
                raise ValueError(f"routine {name}: may press {BUTTONS}, not {press.get('button')!r}")
            start = float(press["t"])
            self.presses.append((start, start + float(press.get("for", 0.3)), press["button"]))

    def sample(self, t: float) -> dict | None:
        """The page's state at t seconds into the routine; None once it is over."""
        if t < 0.0 or t >= self.duration:
            return None
        for (t0, v0, hold), (t1, v1, _) in zip(self.keys, self.keys[1:]):
            if t < t1:
                f = 0.0 if hold else (t - t0) / (t1 - t0)
                values = [a + (b - a) * f for a, b in zip(v0, v1)]
                break
        state = dict(zip(STICKS, values))
        state["hold"] = True
        state["buttons"] = [button for start, end, button in self.presses if start <= t < end]
        return state

    def describe(self) -> dict:
        return {"name": self.name, "label": self.label, "note": self.note, "duration_s": self.duration}


def load(path: str | Path | None) -> dict[str, Routine]:
    """The routines of a file, in its order; none when there is no file."""
    if not path or not Path(path).is_file():
        return {}
    data = yaml.safe_load(Path(path).read_text()) or {}
    return {name: Routine(name, spec) for name, spec in (data.get("routines") or {}).items()}
