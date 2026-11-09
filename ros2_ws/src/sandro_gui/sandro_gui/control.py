"""The controller-manager side of the panel's actions, as sequences of plain calls.

RosBridge makes the calls over the controller manager's services, the demo
on its fake robot, the tests on recorders. A manager provides:

    controllers(timeout) -> {name: {"name", "type", "state", "claimed"}}
    hardware_states(timeout) -> {component: state label}
    load(name), configure(name)
    switch(activate, deactivate, strict, timeout)   raises when refused
    set_hardware(name, active) -> state label       raises when refused
    trigger(service, timeout, wait) -> (success, message), raises when unavailable
    estop_service, clear_service                    the Trigger services' names
"""

from __future__ import annotations

import threading

from .errors import Conflict
from .logbook import WARN

RL_POLICY = "RlPolicyController"
BALANCERS = ("WholeBodyController", "LqrBalancerController", "DrivingBalancerController",
             RL_POLICY)
# Status bits the controllers publish in ~/diagnostics (sandro_controllers):
# rl_policy_core.hpp RlStatus, whole_body_core.hpp WholeBodyStatus, control_core.hpp.
RL_FALLEN, RL_OSCILLATION, RL_PHASE_FALLEN = 1 << 1, 1 << 6, 2
WBC_FALL_LATCH = 1 << 2
SAT_FALL_LATCH = 1 << 9
DIAGNOSTIC_ERROR = 2


class ArmFailed(RuntimeError):
    """The hardware component did not become active."""


def short_type(kind: str | None) -> str:
    return (kind or "").split("/")[-1]


def condition(kind: str | None, diagnostics: dict | None) -> dict | None:
    """{"state": "fallen" | "error", "text"} when a controller's diagnostics level is ERROR."""
    if not diagnostics or diagnostics.get("level") != DIAGNOSTIC_ERROR:
        return None
    values = diagnostics.get("values") or {}

    def bits(key: str) -> int:
        try:
            return int(float(values.get(key, 0)))
        except (TypeError, ValueError):
            return 0

    kind = short_type(kind)
    if kind == RL_POLICY:
        if bits("status_mask") & RL_OSCILLATION:
            return {"state": "fallen", "text": "cut the drives on an oscillation (RL_OSCILLATION)"}
        if bits("phase") == RL_PHASE_FALLEN or bits("status_mask") & RL_FALLEN:
            return {"state": "fallen", "text": "fallen: the policy cut the drives"}
    elif kind == "WholeBodyController" and bits("status_mask") & WBC_FALL_LATCH:
        return {"state": "fallen", "text": "fall latch: the wheels hold zero torque"}
    elif kind in ("LqrBalancerController", "DrivingBalancerController") \
            and bits("saturation_mask") & SAT_FALL_LATCH:
        return {"state": "fallen", "text": "fall latch: the wheels hold zero torque"}
    return {"state": "error", "text": diagnostics.get("message") or "diagnostics level ERROR"}


def needs_handover(target_type: str | None, running_types: list[str | None]) -> bool:
    """An RL policy taking over from a balancer: ~/arm_handover skips its stand phase."""
    return short_type(target_type) == RL_POLICY and any(
        short_type(kind) in BALANCERS for kind in running_types)


def owning(listed: dict, besides: str | None = None) -> list[str]:
    return [c["name"] for c in listed.values()
            if c["state"] == "active" and c["claimed"] and c["name"] != besides]


class Control:
    def __init__(self, manager, log, component: str = "SandroSystem",
                 broadcasters: list[str] | None = None,
                 teleop_policies: list[str] | None = None) -> None:
        self.m = manager
        self.log = log
        self.component = component
        self.broadcasters = list(broadcasters or [])
        self.teleop_policies = list(teleop_policies or [])
        self.estops = 0
        self.last_controller: str | None = None

    def _unless_stopped(self, estops: int, what: str) -> None:
        if self.estops != estops:
            raise RuntimeError(f"E-STOP during the request: {what}")

    def _ready(self, name: str, listed: dict) -> None:
        if name not in listed:
            self.m.load(name)
            self.m.configure(name)
        elif listed[name]["state"] == "unconfigured":
            self.m.configure(name)

    # Controllers --------------------------------------------------------

    def activate(self, name: str, swap: bool = False) -> str:
        estops = self.estops
        listed = self.m.controllers()
        running = owning(listed, name)
        if running and not swap:
            raise Conflict(running)
        # Jazzy activates a controller on inactive hardware, and it acts the
        # moment the drives arm.
        drives = self.m.hardware_states().get(self.component)
        if name not in self.broadcasters and drives is not None and drives != "active":
            raise RuntimeError(f"{self.component} is {drives}: arm the drives first "
                               f"(Arm, or Arm and start)")
        self._ready(name, listed)
        if name in self.teleop_policies:
            for other in self.teleop_policies:
                if other != name and other not in listed:
                    try:
                        self.m.load(other)
                        self.m.configure(other)
                    except Exception as error:  # noqa: BLE001 - the switch is optional
                        self.log.note(f"could not preload {other} for the gamepad switch: {error}",
                                      WARN)
        if name not in listed:
            listed = self.m.controllers()
        target = listed.get(name, {}).get("type")
        handover = needs_handover(target, [listed[r]["type"] for r in running if r in listed])
        self._unless_stopped(estops, f"{name} was not started")
        if handover:
            try:
                accepted, answer = self.m.trigger(f"/{name}/arm_handover", 2.0, 1.0)
            except Exception as error:  # noqa: BLE001 - reported as a refusal
                accepted, answer = False, str(error)
            if not accepted:
                raise RuntimeError(f"{name} did not take the handover ({answer}); nothing was "
                                   f"switched, {', '.join(running)} keeps the robot")
        self.m.switch([name], running)
        self.last_controller = name
        message = f"activated {name}" + (f" (deactivated {', '.join(running)}"
                                         f"{', handover armed' if handover else ''})"
                                         if running else "")
        self.log.note(message, WARN)
        return message

    def deactivate(self, name: str) -> str:
        self.m.switch([], [name])
        self.log.note(f"deactivated {name}", WARN)
        return f"deactivated {name}"

    def restart(self, name: str) -> str:
        """Deactivate and activate again: how a fallen or latched controller starts over."""
        estops = self.estops
        if self.m.controllers().get(name, {}).get("state") == "active":
            self.m.switch([], [name])
        self._unless_stopped(estops, f"{name} was stopped, not restarted")
        self.activate(name)
        self.log.note(f"restarted {name}", WARN)
        return f"restarted {name}"

    def configure(self, name: str) -> str:
        if name not in self.m.controllers():
            self.m.load(name)
        self.m.configure(name)
        self.log.note(f"configured {name}")
        return f"configured {name}"

    # Hardware -----------------------------------------------------------

    def arm(self, component: str | None = None) -> str:
        component = component or self.component
        try:
            label = self.m.set_hardware(component, True)
        except Exception as error:  # noqa: BLE001 - the caller names the drive fault
            raise ArmFailed(f"{component} did not arm ({error})") from error
        self.log.note(f"{component} is {label}", WARN)
        # The drives' read error that follows an E-stop deactivates the broadcasters.
        listed = self.m.controllers()
        stopped = [b for b in self.broadcasters if b in listed and listed[b]["state"] == "inactive"]
        if stopped:
            try:
                self.m.switch(stopped, [], strict=False)
            except Exception as error:  # noqa: BLE001 - armed all the same
                self.log.note(f"could not restart {', '.join(stopped)}: {error}", WARN)
        return f"{component} {label}"

    def disarm(self, component: str | None = None) -> str:
        component = component or self.component
        running = owning(self.m.controllers())
        if running:
            self.m.switch([], running, strict=False)
            self.last_controller = running[0]
        label = self.m.set_hardware(component, False)
        message = (f"deactivated {', '.join(running)}; " if running else "") + f"{component} {label}"
        self.log.note(message, WARN)
        return message

    def arm_and_start(self, controller: str, component: str | None = None) -> str:
        component = component or self.component
        estops = self.estops
        notes = []
        if self.m.hardware_states().get(component) != "active":
            notes.append(self.arm(component))
        self._unless_stopped(estops, f"{controller} was not started")
        notes.append(self.activate(controller))
        return "; ".join(notes)

    # Stops --------------------------------------------------------------

    def emergency_stop(self, direct=None) -> str:
        """`direct` sends the CAN E-stop on the panel's own socket: first, before any DDS call.

        The E-stop service and the controller deactivation then run in parallel,
        so a slow controller manager never delays the drives' stop.
        """
        self.estops += 1
        notes = [direct()] if direct is not None else []
        service: dict = {}

        def fire() -> None:
            try:
                _, answer = self.m.trigger(self.m.estop_service, 3.0, 0.0)
                service["note"] = f"CAN E-stop service: {answer}"
            except Exception as error:  # noqa: BLE001
                service["note"] = f"no CAN E-stop service ({error})"

        thread = threading.Thread(target=fire, name="estop", daemon=True)
        thread.start()
        try:
            running = owning(self.m.controllers(2.0))
            if running:
                self.m.switch([], running, strict=False, timeout=2.0)
                self.last_controller = running[0]
                deactivated = f"deactivated {', '.join(running)}"
            else:
                deactivated = "no controller was running"
        except Exception as error:  # noqa: BLE001 - the CAN E-stop went out regardless
            deactivated = f"controllers: {error}"
        thread.join(4.0)
        notes += [service.get("note", "CAN E-stop service: no answer within 4 s"), deactivated]
        return "; ".join(notes)

    def clear_errors(self) -> str:
        _, answer = self.m.trigger(self.m.clear_service, 3.0, 1.0)
        self.log.note(answer, WARN)
        return answer
