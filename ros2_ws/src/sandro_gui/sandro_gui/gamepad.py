"""The PS5 pad as the panel sees it: live input, battery, and the button guide.

Two sources give the same named state. /joy is what the robot actually uses
(SDL's gamepad layout, the indices in sandro_bringup/config/teleop.yaml);
the kernel joystick node (/dev/input/jsN, joydev order) works before any ROS
node runs, so a freshly paired pad can be tested on its own.

The guide is generated from teleop.yaml, so it always shows the buttons the
running teleop nodes are configured with.
"""

from __future__ import annotations

import glob
import os
import select
import struct
import threading
import time

# SDL GameController order, which joy_node reports for the DualSense.
SDL_BUTTONS = ["cross", "circle", "square", "triangle", "create", "ps", "options", "l3", "r3",
               "l1", "r1", "up", "down", "left", "right", "mute"]
SDL_AXES = ["lx", "ly", "rx", "ry", "l2", "r2"]
# Linux joydev order for hid-playstation (BTN_SOUTH, EAST, NORTH, WEST, TL, ...).
JS_BUTTONS = ["cross", "circle", "triangle", "square", "l1", "r1", "l2", "r2", "create",
              "options", "ps", "l3", "r3"]

LABELS = {
    "cross": "Cross", "circle": "Circle", "square": "Square", "triangle": "Triangle",
    "create": "Create", "options": "Options", "ps": "PS", "mute": "Mute",
    "l1": "L1", "r1": "R1", "l2": "L2", "r2": "R2", "l3": "L3", "r3": "R3",
    "up": "D-pad up", "down": "D-pad down", "left": "D-pad left", "right": "D-pad right",
    "lx": "left stick left/right", "ly": "left stick up/down",
    "rx": "right stick left/right", "ry": "right stick up/down",
}
_EVENT = struct.Struct("<IhBB")


def _pick(values, index, default=0.0):
    try:
        return float(values[index])
    except (IndexError, TypeError):
        return default


def from_joy(axes, buttons) -> dict:
    """Named state from a sensor_msgs/Joy (SDL: left and up +1, triggers +1 released)."""
    state = {name: bool(_pick(buttons, i)) for i, name in enumerate(SDL_BUTTONS)}
    l2 = (1.0 - _pick(axes, 4, 1.0)) / 2.0 if len(axes) > 4 else 0.0
    r2 = (1.0 - _pick(axes, 5, 1.0)) / 2.0 if len(axes) > 5 else 0.0
    state.update(l2=l2 > 0.5, r2=r2 > 0.5)
    return {"buttons": state, "lx": -_pick(axes, 0), "ly": _pick(axes, 1),
            "rx": -_pick(axes, 2), "ry": _pick(axes, 3), "l2": l2, "r2": r2}


def from_js(axes, buttons) -> dict:
    """Named state from joydev values (-32767..32767, y down, triggers -1 released)."""
    a = [v / 32767.0 for v in axes]
    state = {name: bool(_pick(buttons, i)) for i, name in enumerate(JS_BUTTONS)}
    hat_x, hat_y = _pick(a, 6), _pick(a, 7)
    state.update(left=hat_x < -0.5, right=hat_x > 0.5, up=hat_y < -0.5, down=hat_y > 0.5)
    return {"buttons": state, "lx": _pick(a, 0), "ly": -_pick(a, 1), "rx": _pick(a, 3),
            "ry": -_pick(a, 4), "l2": (_pick(a, 2, -1.0) + 1.0) / 2.0,
            "r2": (_pick(a, 5, -1.0) + 1.0) / 2.0}


def battery() -> dict | None:
    for path in sorted(glob.glob("/sys/class/power_supply/ps-controller-battery-*")):
        try:
            with open(os.path.join(path, "capacity")) as stream:
                capacity = int(stream.read().strip())
            with open(os.path.join(path, "status")) as stream:
                status = stream.read().strip()
        except (OSError, ValueError):
            continue
        return {"percent": capacity, "status": status,
                "mac": os.path.basename(path).rsplit("-", 1)[-1]}
    return None


def find_pad() -> tuple[str, str] | None:
    """(device, name) of the gamepad's joystick node, skipping motion sensors and touchpad."""
    best = None
    for device in sorted(glob.glob("/dev/input/js*")):
        try:
            with open(f"/sys/class/input/{os.path.basename(device)}/device/name") as stream:
                name = stream.read().strip()
        except OSError:
            name = ""
        if "Motion" in name or "Touchpad" in name:
            continue
        if "Wireless Controller" in name or "DualSense" in name:
            return device, name
        best = best or (device, name)
    return best


class JsReader:
    """Reads the pad's joystick node in a thread; reconnects when it reappears."""

    def __init__(self) -> None:
        self._state: dict | None = None
        self._device: tuple[str, str] | None = None
        self._time = 0.0
        self._error: str | None = None
        threading.Thread(target=self._run, name="gamepad", daemon=True).start()

    def _run(self) -> None:
        while True:
            found = find_pad()
            if found is None:
                self._device, self._state = None, None
                time.sleep(1.0)
                continue
            try:
                fd = os.open(found[0], os.O_RDONLY | os.O_NONBLOCK)
            except OSError as error:
                self._error = f"{found[0]}: {error.strerror}"
                time.sleep(2.0)
                continue
            self._device, self._error = found, None
            axes, buttons = [0] * 8, [0] * 13
            try:
                while True:
                    ready, _, _ = select.select([fd], [], [], 1.0)
                    if not ready:
                        continue
                    data = os.read(fd, _EVENT.size * 64)
                    if not data:
                        break
                    for offset in range(0, len(data) - _EVENT.size + 1, _EVENT.size):
                        _, value, kind, number = _EVENT.unpack_from(data, offset)
                        kind &= 0x7F
                        if kind == 1 and number < 32:
                            buttons.extend([0] * (number + 1 - len(buttons)))
                            buttons[number] = value
                        elif kind == 2 and number < 32:
                            axes.extend([0] * (number + 1 - len(axes)))
                            axes[number] = value
                    self._state, self._time = from_js(axes, buttons), time.monotonic()
            except OSError:
                pass
            finally:
                os.close(fd)
            self._device, self._state = None, None

    def snapshot(self) -> dict:
        return {"device": self._device[0] if self._device else None,
                "name": self._device[1] if self._device else None,
                "error": self._error, "state": self._state,
                "age_s": time.monotonic() - self._time if self._state else None}


def _button(index: int) -> str:
    return SDL_BUTTONS[index] if 0 <= index < len(SDL_BUTTONS) else f"button{index}"


def _axis(index: int) -> str:
    return SDL_AXES[index] if 0 <= index < len(SDL_AXES) else f"axis{index}"


def guide(teleop: dict) -> dict:
    """Every gamepad combination the teleop nodes react to, from teleop.yaml."""
    t = teleop["joy_teleop"]["ros__parameters"]
    e = teleop["joy_estop"]["ros__parameters"]
    dead = _button(t["deadman_button"])
    policies = list(t.get("policies", []))
    colors = list(t.get("policy_colors", []))
    hold = float(e.get("rearm_hold_s", 2.0))
    trig = [_axis(i) for i in e["axes"]]
    rearm = [_button(i) for i in e["rearm_buttons"]]
    pressed = (1.0 - float(e["axis_pressed_below"])) / 2.0
    entries = [
        {"group": "Drive", "keys": [dead], "hold": True,
         "text": "Deadman. Driving commands are sent only while it is held; releasing it "
                 "sends zero speed once and the controller holds position."},
        {"group": "Drive", "keys": [dead, _axis(t["forward_axis"])],
         "text": "Forward and backward speed. Full deflection is the running controller's "
                 f"own speed limit times speed_fraction ({t.get('speed_fraction', 1.0):g})."},
        {"group": "Drive", "keys": [dead, _axis(t["yaw_axis"])],
         "text": "Turn (yaw rate); stick left turns left."},
        {"group": "Drive", "keys": [dead, _axis(t["height_axis"])],
         "text": "Body height up and down (a height rate: the body stays where you leave it)."},
        {"group": "Drive", "keys": [dead, _axis(t["roll_axis"])],
         "text": "Body roll, stick right puts the right side down. Only policies with a roll "
                 "command (rl_agile_terrain) use it."},
        {"group": "Drive", "keys": [dead, _button(t["recentre_button"])],
         "text": "Height back to the running policy's initial height."},
        {"group": "Policies", "keys": [dead, _button(t["switch_button"])],
         "text": "Switch to the next policy (" + " → ".join(policies) + "). The robot first "
                 f"settles at {t.get('handover_height_m', 0.17):g} m with zero speed and roll, "
                 "then the incoming policy takes over in one control cycle. Releasing "
                 f"{LABELS[dead]} or the E-stop aborts the switch "
                 f"(gives up after {t.get('handover_timeout_s', 8.0):g} s)."},
        {"group": "Policies", "keys": [_button(t["announce_button"])],
         "text": "Rumble the running policy's number: one pulse for the first, two for the "
                 "second. One long rumble means a switch was refused or aborted."},
        {"group": "Safety", "keys": trig, "hold": False,
         "text": f"Software E-stop: squeeze both triggers past {pressed:.0%}. Deactivates "
                 "every controller (all torques to zero) and sends the CAN E-stop to the drives. "
                 "It is not a substitute for the power cut-off."},
        {"group": "Safety", "keys": rearm, "hold": True,
         "text": f"Re-arm after an E-stop, in two holds of {hold:g} s each; only once the "
                 "drives have been armed in this launch (the first arming is the operator's, "
                 "on the panel). First hold: the drives are armed at zero torque with no "
                 "controller. Hold the robot upright, then hold again: the controller that ran "
                 f"last starts again (or {e.get('rearm_controller', 'whole_body_controller')}). "
                 "A fallen controller restarts with a single hold."},
    ]
    lightbar = [{"policy": name, "rgb": colors[3 * i:3 * i + 3]}
                for i, name in enumerate(policies) if len(colors) >= 3 * i + 3]
    lightbar += [{"policy": "no controller running", "rgb": [255, 0, 0]},
                 {"policy": "a controller outside the switch list", "rgb": [255, 255, 255]}]
    return {"entries": entries, "lightbar": lightbar, "policies": policies,
            "labels": LABELS, "deadman": dead, "estop": trig, "rearm": rearm,
            "estop_fraction": pressed}


def guide_markdown(data: dict) -> str:
    rows = ["| Buttons | What it does |", "|---|---|"]
    for entry in data["entries"]:
        keys = " + ".join(data["labels"].get(k, k) for k in entry["keys"])
        if entry.get("hold"):
            keys = "hold " + keys
        rows.append(f"| {keys} | {entry['text']} |")
    rows += ["", "Light bar colour:", ""]
    for item in data["lightbar"]:
        rows.append(f"- rgb({', '.join(str(c) for c in item['rgb'])}): {item['policy']}")
    return "\n".join(rows) + "\n"
