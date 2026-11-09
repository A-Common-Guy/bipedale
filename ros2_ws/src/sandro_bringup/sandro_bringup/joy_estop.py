"""Gamepad emergency stop and two-step re-arm for Sandro.

Squeezing the configured trigger chord (PS5 L2 + R2 by default) deactivates
every active ros2_control controller that owns command interfaces, which zeros
all six efforts in simulation and on hardware, and additionally triggers the
CAN E-stop service when it exists (physical robot only). The chord is
edge-triggered and latched until released.

Re-arming is deliberate and takes two separate holds of the re-arm buttons
(PS5 Create + Options for 2 s by default), so the motors never jump straight
from a stop into a balancer:

1. With the drives disarmed (after the CAN E-stop the drives latch
   ESTOP_REQUESTED and the hardware component leaves `active`), the first hold
   activates the hardware component. Activation clears that bit and the CAN
   watchdog code only, any other fault still refuses, and the drives hold zero
   torque with no controller. The gamepad does this only once the drives have
   been active in this launch: a real-hardware launch starts them inactive,
   and the first arming of a session is the operator's (control panel,
   `ros2 control` or arm_hardware:=true), after the pre-arm checks.
2. With the drives armed and no balancer running, the next hold activates the
   controller that ran last (`rearm_controller` until one has run), loading
   and configuring it first if needed, and then loads the other policies of
   the gamepad switch inactive. Hold the robot upright: the legs stand at once
   and the wheels engage when it is upright and still.

A controller that still runs but reports ERROR on its ~/diagnostics (fallen,
latched limp) restarts with a single hold, deactivated and activated in one
switch, without the E-stop.

The same can be done by hand:

    ros2 control set_hardware_component_state SandroSystem active
    ros2 control switch_controllers --activate whole_body_controller

This is software over DDS and CAN. It is not a substitute for the physical
power cut-off.
"""

from __future__ import annotations

import time

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController, ListControllers, ListHardwareComponents, LoadController,
    SetHardwareComponentState, SwitchController)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from lifecycle_msgs.msg import State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

# An ERROR older than this (several diagnostics periods) no longer counts.
ERROR_FRESH_S = 1.0


def chord_pressed(axes, chord, pressed_below) -> bool:
    """True when every axis index in `chord` is present and at or below `pressed_below`.

    SDL reports the PS5 triggers as axes, +1 released and -1 fully pressed; a
    joystick that has not sent an event yet reads 0, which never triggers.
    """
    if not chord:
        return False
    try:
        return all(axes[index] <= pressed_below for index in chord)
    except IndexError:
        return False


def buttons_held(buttons, chord) -> bool:
    """True when every button index in `chord` is present and pressed."""
    if not chord:
        return False
    try:
        return all(buttons[index] > 0 for index in chord)
    except IndexError:
        return False


def rearm_step(hardware_active, running, target_state, failed=(), armed_once=True) -> str:
    """What one re-arm hold does.

    One of arm_hardware, not_armed_yet (the gamepad never arms first),
    blocked (a controller is active on disarmed drives and would act the
    moment they arm), restart (every running controller reports ERROR),
    nothing, activate, configure_and_activate or load_and_activate.

    `hardware_active` is None when the hardware component does not exist (it
    then cannot be the reason for refusing), `running` lists the active
    command-owning controllers, `target_state` is the re-arm controller's
    state (None when not loaded) and `failed` the running controllers whose
    diagnostics report ERROR.
    """
    if hardware_active is False:
        if running:
            return "blocked"
        return "arm_hardware" if armed_once else "not_armed_yet"
    if running:
        return "restart" if set(running) <= set(failed) else "nothing"
    if target_state is None:
        return "load_and_activate"
    return "configure_and_activate" if target_state == "unconfigured" else "activate"


def switch_partners(policies, controller) -> list[str]:
    """The gamepad switch's other policies when `controller` is one of them."""
    return [p for p in policies if p != controller] if controller in policies else []


def controllers_to_stop(states) -> list[str]:
    """Names of active controllers that own command interfaces.

    Broadcasters claim no command interface and keep publishing state, which
    is exactly what you want to keep after a stop.
    """
    return [
        state.name
        for state in states
        if state.state == "active" and len(state.claimed_interfaces) > 0
    ]


def reports_error(statuses, error=None) -> bool:
    error = DiagnosticStatus.ERROR if error is None else error
    return any(status.level == error for status in statuses)


def succeeded(future) -> bool:
    return not future.exception() and future.result().ok


class JoyEstop(Node):
    def __init__(self) -> None:
        super().__init__("joy_estop")
        self.declare_parameter("axes", [4, 5])
        self.declare_parameter("axis_pressed_below", -0.5)
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("hardware_estop_service", "/gim6010_estop/trigger")
        self.declare_parameter("service_timeout_s", 2.0)
        self.declare_parameter("rearm_buttons", [4, 6])
        self.declare_parameter("rearm_hold_s", 2.0)
        self.declare_parameter("rearm_controller", "whole_body_controller")
        self.declare_parameter("switch_policies", ["rl_flat_level", "rl_agile_terrain"])
        self.declare_parameter("hardware_component", "SandroSystem")
        self.declare_parameter("rearm_broadcasters", ["joint_state_broadcaster"])
        self.declare_parameter("rearm_timeout_s", 10.0)
        self.chord = [int(v) for v in self.get_parameter("axes").value]
        self.pressed_below = float(self.get_parameter("axis_pressed_below").value)
        self.rearm_chord = [int(v) for v in self.get_parameter("rearm_buttons").value]
        self.rearm_hold_s = float(self.get_parameter("rearm_hold_s").value)
        self.rearm_controller = self.get_parameter("rearm_controller").value
        self.switch_policies = list(self.get_parameter("switch_policies").value)
        self.hardware_component = self.get_parameter("hardware_component").value
        self.rearm_broadcasters = list(self.get_parameter("rearm_broadcasters").value)
        self.rearm_timeout_s = float(self.get_parameter("rearm_timeout_s").value)
        manager = self.get_parameter("controller_manager").value
        self.timeout = float(self.get_parameter("service_timeout_s").value)
        group = ReentrantCallbackGroup()
        self.group = group
        self.list_client = self.create_client(
            ListControllers, f"{manager}/list_controllers", callback_group=group)
        self.switch_client = self.create_client(
            SwitchController, f"{manager}/switch_controller", callback_group=group)
        self.list_hardware_client = self.create_client(
            ListHardwareComponents, f"{manager}/list_hardware_components",
            callback_group=group)
        self.set_hardware_client = self.create_client(
            SetHardwareComponentState, f"{manager}/set_hardware_component_state",
            callback_group=group)
        self.load_client = self.create_client(
            LoadController, f"{manager}/load_controller", callback_group=group)
        self.configure_client = self.create_client(
            ConfigureController, f"{manager}/configure_controller", callback_group=group)
        self.hardware_client = self.create_client(
            Trigger, self.get_parameter("hardware_estop_service").value,
            callback_group=group)
        self.subscription = self.create_subscription(
            Joy, self.get_parameter("joy_topic").value, self._on_joy, 10,
            callback_group=group)
        # joy_teleop publishes the running command-owning controller; re-arm
        # restarts the last one rather than the launch's.
        self.create_subscription(
            String, "/teleop/active_controller", self._on_active,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group)
        self.latched = False
        self.rearm_since = None
        self.rearm_fired = False
        self.rearm_busy = False
        self.rearm_started = 0.0
        self.rearm_generation = 0
        self.hardware_active = None
        self.armed_once = False
        self.polling_hardware = False
        self.last_active: str | None = None
        self.listed: dict[str, str] = {}
        self.errors: dict[str, float] = {}
        self.diagnostics: dict[str, object] = {}
        self._watch(self.rearm_controller)
        self.hardware_timer = self.create_timer(1.0, self._poll_hardware, callback_group=group)
        self.get_logger().info(
            f"gamepad E-stop armed on axes {self.chord} (press together); re-arm by "
            f"holding buttons {self.rearm_chord} for {self.rearm_hold_s:.1f} s, twice")

    @property
    def target(self) -> str:
        return self.last_active or self.rearm_controller

    def _on_active(self, msg: String) -> None:
        if msg.data and msg.data != self.last_active:
            self.last_active = msg.data
            self._watch(msg.data)
            self.get_logger().info(f"re-arm now starts {self.target}")

    def _watch(self, name: str) -> None:
        if name and name not in self.diagnostics:
            self.diagnostics[name] = self.create_subscription(
                DiagnosticArray, f"/{name}/diagnostics",
                lambda msg, name=name: self._on_diagnostics(name, msg), 10,
                callback_group=self.group)

    def _on_diagnostics(self, name: str, msg: DiagnosticArray) -> None:
        if reports_error(msg.status):
            self.errors[name] = time.monotonic()
        else:
            self.errors.pop(name, None)

    def _poll_hardware(self) -> None:
        """Until the drives have been active once in this launch (armed_once)."""
        if self.armed_once:
            self.hardware_timer.cancel()
            return
        if self.polling_hardware or not self.list_hardware_client.service_is_ready():
            return
        self.polling_hardware = True
        self.list_hardware_client.call_async(
            ListHardwareComponents.Request()).add_done_callback(self._on_hardware_polled)

    def _on_hardware_polled(self, future) -> None:
        self.polling_hardware = False
        try:
            self._note_hardware(future.result().component)
        except Exception:  # noqa: BLE001 - the next poll retries
            pass

    def _note_hardware(self, components) -> None:
        states = [c.state.id for c in components if c.name == self.hardware_component]
        self.hardware_active = states[0] == State.PRIMARY_STATE_ACTIVE if states else None
        if self.hardware_active and not self.armed_once:
            self.armed_once = True
            self.get_logger().info(
                f"{self.hardware_component} armed in this session; the gamepad re-arm may "
                "re-activate it after a stop")

    def _on_joy(self, msg: Joy) -> None:
        pressed = chord_pressed(msg.axes, self.chord, self.pressed_below)
        if pressed and not self.latched:
            self.latched = True
            self.trigger()
        elif not pressed and self.latched:
            self.latched = False
            self.get_logger().warn(
                f"E-stop chord released. To re-arm, hold buttons {self.rearm_chord} for "
                f"{self.rearm_hold_s:.1f} s (drives at zero torque), then again "
                f"({self.target}).")
        if pressed or not buttons_held(msg.buttons, self.rearm_chord):
            self.rearm_since = None
            self.rearm_fired = False
            return
        now = time.monotonic()
        if self.rearm_since is None:
            self.rearm_since = now
        elif not self.rearm_fired and now - self.rearm_since >= self.rearm_hold_s:
            self.rearm_fired = True
            self.rearm()

    def rearm(self) -> None:
        now = time.monotonic()
        if self.rearm_busy and now - self.rearm_started < self.rearm_timeout_s:
            self.get_logger().warn("re-arm already in progress")
            return
        if self.rearm_busy:
            self.get_logger().error(
                f"re-arm: the previous attempt got no answer in {self.rearm_timeout_s:.0f} s; "
                "starting over (check the controller manager log)")
        if not (self.list_client.service_is_ready() and
                self.list_hardware_client.service_is_ready()):
            self.rearm_busy = False
            self.get_logger().error("controller manager not reachable; cannot re-arm")
            return
        self.rearm_busy = True
        self.rearm_started = now
        self.rearm_generation += 1
        self._call(self.list_hardware_client, ListHardwareComponents.Request(),
                   self._on_hardware_listed)

    def _call(self, client, request, handler) -> None:
        """call_async whose reply is dropped once a newer re-arm has started."""
        generation = self.rearm_generation

        def done(future):
            if generation != self.rearm_generation:
                self.get_logger().warn("re-arm: ignoring a late reply to an abandoned attempt")
                return
            handler(future)
        client.call_async(request).add_done_callback(done)

    def _rearm_failed(self, what) -> None:
        self.rearm_busy = False
        self.get_logger().error(f"re-arm: {what}")

    def _on_hardware_listed(self, future) -> None:
        try:
            components = future.result().component
        except Exception as error:  # noqa: BLE001
            self._rearm_failed(f"list_hardware_components failed: {error}")
            return
        self._note_hardware(components)
        self._call(self.list_client, ListControllers.Request(), self._on_rearm_listed)

    def _on_rearm_listed(self, future) -> None:
        try:
            controllers = future.result().controller
        except Exception as error:  # noqa: BLE001
            self._rearm_failed(f"list_controllers failed: {error}")
            return
        running = controllers_to_stop(controllers)
        self.listed = {c.name: c.state for c in controllers}
        now = time.monotonic()
        failed = [n for n in running if now - self.errors.get(n, -1e9) < ERROR_FRESH_S]
        step = rearm_step(self.hardware_active, running, self.listed.get(self.target),
                          failed, self.armed_once)
        if step == "arm_hardware":
            self.get_logger().warn(
                f"RE-ARM 1/2: activating {self.hardware_component} (drives at zero torque)")
            request = SetHardwareComponentState.Request()
            request.name = self.hardware_component
            request.target_state = State(id=State.PRIMARY_STATE_ACTIVE, label="active")
            self._call(self.set_hardware_client, request, self._on_hardware_armed)
        elif step == "not_armed_yet":
            self._rearm_failed(
                f"{self.hardware_component} has not been armed in this session; arm it "
                "from the control panel (or ros2 control) after the pre-arm checks")
        elif step == "blocked":
            self._rearm_failed(
                f"{running} active while {self.hardware_component} is disarmed; it would "
                "act the moment the drives arm. Squeeze the E-stop chord first")
        elif step == "restart":
            self.get_logger().warn(f"RE-ARM: restarting {running} (reported ERROR)")
            for name in running:
                self.errors.pop(name, None)
            request = SwitchController.Request()
            request.deactivate_controllers = running
            request.activate_controllers = running
            request.strictness = SwitchController.Request.STRICT
            request.timeout = rclpy.duration.Duration(seconds=self.timeout).to_msg()
            self._call(self.switch_client, request,
                       lambda done, names=running: self._on_activated(done, names))
        elif step == "nothing":
            self.rearm_busy = False
            self.get_logger().warn(f"re-arm: {running} already running; nothing to do")
        elif step == "activate":
            self._activate()
        elif step == "configure_and_activate":
            self._configure()
        else:
            self.get_logger().info(f"re-arm: loading {self.target}")
            request = LoadController.Request()
            request.name = self.target
            self._call(self.load_client, request, self._on_loaded)

    def _on_hardware_armed(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self._rearm_failed(f"set_hardware_component_state failed: {error}")
            return
        self.rearm_busy = False
        if response.ok:
            self.get_logger().warn(
                f"RE-ARM 1/2 done: drives armed at zero torque. Hold the robot upright and "
                f"hold the re-arm buttons again to start {self.target}.")
            self.list_client.call_async(
                ListControllers.Request()).add_done_callback(self._restart_broadcasters)
        else:
            self.get_logger().error(
                f"RE-ARM refused: {self.hardware_component} is '{response.state.label}'; "
                "the controller manager log names the drive fault")

    def _restart_broadcasters(self, future) -> None:
        """The hardware read error also deactivated the broadcasters reading it."""
        try:
            controllers = future.result().controller
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"list_controllers failed: {error}")
            return
        names = [c.name for c in controllers
                 if c.name in self.rearm_broadcasters and c.state == "inactive"]
        if not names:
            return
        request = SwitchController.Request()
        request.activate_controllers = names
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.timeout = rclpy.duration.Duration(seconds=self.timeout).to_msg()
        self.switch_client.call_async(request).add_done_callback(
            lambda done, names=names: self.get_logger().info(
                f"re-arm: restarted {names}: {'ok' if succeeded(done) else 'FAILED'}"))

    def _on_loaded(self, future) -> None:
        try:
            ok = future.result().ok
        except Exception as error:  # noqa: BLE001
            ok = False
            self.get_logger().error(f"load_controller failed: {error}")
        if not ok:
            self._rearm_failed(f"could not load {self.target}")
            return
        self._configure()

    def _configure(self) -> None:
        request = ConfigureController.Request()
        request.name = self.target
        self._call(self.configure_client, request, self._on_configured)

    def _on_configured(self, future) -> None:
        try:
            ok = future.result().ok
        except Exception as error:  # noqa: BLE001
            ok = False
            self.get_logger().error(f"configure_controller failed: {error}")
        if not ok:
            self._rearm_failed(f"could not configure {self.target}")
            return
        self._activate()

    def _activate(self) -> None:
        target = self.target
        self.get_logger().warn(f"RE-ARM 2/2: activating {target}")
        request = SwitchController.Request()
        request.activate_controllers = [target]
        request.strictness = SwitchController.Request.STRICT
        request.timeout = rclpy.duration.Duration(seconds=self.timeout).to_msg()
        self._call(self.switch_client, request,
                   lambda done: self._on_activated(done, [target]))

    def _on_activated(self, future, names) -> None:
        try:
            ok = future.result().ok
        except Exception as error:  # noqa: BLE001
            ok = False
            self.get_logger().error(f"switch_controller failed: {error}")
        self.rearm_busy = False
        if ok:
            self.get_logger().warn(f"RE-ARM done: {names} active")
            for name in names:
                self._preload_partners(name)
        else:
            self.get_logger().error(f"RE-ARM: {names} did not activate")

    def _preload_partners(self, controller: str) -> None:
        """Loads and configures the gamepad switch's other policies, as the launch does."""
        for partner in switch_partners(self.switch_policies, controller):
            state = self.listed.get(partner)
            if state is None:
                request = LoadController.Request()
                request.name = partner
                self.load_client.call_async(request).add_done_callback(
                    lambda done, partner=partner: self._configure_partner(partner)
                    if succeeded(done) else self.get_logger().error(
                        f"re-arm: could not load {partner} for the policy switch"))
            elif state == "unconfigured":
                self._configure_partner(partner)

    def _configure_partner(self, partner: str) -> None:
        request = ConfigureController.Request()
        request.name = partner
        self.configure_client.call_async(request).add_done_callback(
            lambda done: self.get_logger().info(
                f"re-arm: {partner} inactive for the policy switch: "
                f"{'ok' if succeeded(done) else 'FAILED'}"))

    def trigger(self) -> None:
        self.get_logger().error("GAMEPAD E-STOP: deactivating controllers")
        if self.list_client.service_is_ready():
            future = self.list_client.call_async(ListControllers.Request())
            future.add_done_callback(self._on_listed)
        else:
            self.get_logger().error("controller manager not reachable; nothing to deactivate")
        self._trigger_hardware()

    def _on_listed(self, future) -> None:
        try:
            names = controllers_to_stop(future.result().controller)
        except Exception as error:  # noqa: BLE001 - report any failure of the stop path
            self.get_logger().error(f"list_controllers failed: {error}")
            return
        if not names:
            self.get_logger().warn("no active command-owning controller to deactivate")
            return
        self.last_active = names[0]
        self._watch(names[0])
        request = SwitchController.Request()
        request.deactivate_controllers = names
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.timeout = rclpy.duration.Duration(seconds=self.timeout).to_msg()
        switch = self.switch_client.call_async(request)
        switch.add_done_callback(
            lambda done, names=names: self._on_switched(done, names))

    def _on_switched(self, future, names) -> None:
        try:
            ok = future.result().ok
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"switch_controller failed: {error}")
            return
        level = self.get_logger().error if ok else self.get_logger().fatal
        level(f"deactivated {names}: {'ok' if ok else 'FAILED'}")

    def _trigger_hardware(self) -> None:
        if not self.hardware_client.service_is_ready():
            self.get_logger().info("no CAN E-stop service (simulation); controllers only")
            return
        future = self.hardware_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_hardware)

    def _on_hardware(self, future) -> None:
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"CAN E-stop call failed: {error}")
            return
        (self.get_logger().error if response.success else self.get_logger().fatal)(
            f"CAN E-stop: {response.message}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyEstop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
