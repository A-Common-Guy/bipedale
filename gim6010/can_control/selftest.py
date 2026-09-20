#!/usr/bin/env python3
"""End-to-end test of gim6010_can against the software motor emulator.

Runs entirely in-process on python-can's 'virtual' interface — no hardware,
no vcan kernel module. Exercises: heartbeat decode, scan, torque mode enable
+ streaming + synchronous RTR feedback, all getters, detailed error decode,
estop/clear, SDO read/write (endpoints_v3.json), deadman, MIT echo, and
output-frame unit conversion.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import can

from gim6010_can import GIM6010Bus, decode_flags, AXIS_ERRORS
from sim_motor import SimMotor

ENDPOINTS = (Path(__file__).resolve().parent.parent
             / "manual" / "assets_protocol_manual" / "endpoints_v3.json")

PASS = 0


def check(name: str, cond: bool, detail: str = ""):
    global PASS
    status = "ok " if cond else "FAIL"
    print(f"  [{status}] {name}{(' — ' + detail) if detail else ''}")
    if cond:
        PASS += 1
    else:
        raise AssertionError(name)


def main() -> int:
    sim_bus = can.Bus(interface="virtual", channel="simnet",
                      receive_own_messages=False)
    sim = SimMotor(sim_bus, node_id=7)
    host_bus = can.Bus(interface="virtual", channel="simnet",
                       receive_own_messages=False)

    with GIM6010Bus(bus=host_bus, endpoints=ENDPOINTS) as bus:
        m = bus.motor(7)

        print("discovery / heartbeat")
        nodes = bus.scan(0.4)
        check("scan finds node 7", nodes == [7], f"nodes={nodes}")
        check("heartbeat state = IDLE", m.state.axis_state_name() == "IDLE")
        life0 = m.state.life
        time.sleep(0.35)
        check("life counter advances", m.state.life != life0)
        check("no lost heartbeats", m.state.lost_heartbeats == 0)

        print("torque mode")
        m.enable_torque_mode()
        time.sleep(0.15)
        check("closed loop entered", sim.axis_state == 8,
              f"sim state={sim.axis_state}")
        check("mode = torque/passthrough",
              (sim.control_mode, sim.input_mode) == (1, 1))
        m.set_input_torque(0.05)
        time.sleep(0.25)
        pos, vel = m.get_encoder_estimates()
        check("RTR pos/vel reply, motor spins under torque", vel > 0.1,
              f"pos={pos:.3f} vel={vel:.3f}")
        tsp, tq = m.get_torques()
        check("torque setpoint echoed", abs(tsp - 0.05) < 1e-6,
              f"sp={tsp:.4f} act={tq:.4f}")
        check("periodic 0x09 also decoded", m.state.t_encoder > 0)

        print("feedback rates over CAN (non-blocking loop support)")
        counter = {"n": 0}
        m.on_encoder = lambda s: counter.update(n=counter["n"] + 1)
        time.sleep(0.5)
        slow = counter["n"]
        m.set_feedback_rates(encoder=2)
        check("rate readback", m.get_feedback_rates()["encoder"] == 2)
        counter["n"] = 0
        time.sleep(0.5)
        fast = counter["n"]
        check("2 ms stream is faster than 10 ms default", fast > 2 * slow,
              f"{slow} -> {fast} frames per 0.5s")
        check("stream >= ~250 Hz", fast >= 125, f"{fast * 2} Hz")
        m.encoder_event.clear()
        check("encoder_event paces the loop", m.encoder_event.wait(0.05))
        age = m.state.encoder_age()
        check("encoder_age fresh", age < 0.05, f"age={age * 1e3:.1f} ms")
        check("unknown rate key rejected", _raises(
            lambda: m.set_feedback_rates(bogus=1), KeyError))
        m.on_encoder = None

        print("getters")
        iq_sp, iq_meas = m.get_iq()
        check("get_iq", abs(iq_sp - 0.05 / 0.07) < 1e-3)
        mt, ft = m.get_temperature()
        check("get_temperature", (mt, ft) == (32.5, 30.0))
        vb, ib = m.get_bus_voltage_current()
        check("get_bus_voltage_current", vb == 24.0, f"vbus={vb}")
        ep, mp = m.get_powers()
        check("get_powers", abs(ep - 24.0 * 0.1) < 1e-4)
        sc, cpr = m.get_encoder_count()
        check("get_encoder_count", isinstance(sc, int))

        print("errors / estop")
        sim.errors["encoder"] = 0x2  # CPR_POLEPAIRS_MISMATCH
        raw, names = m.get_error("encoder")
        check("detailed encoder error decode",
              raw == 2 and names == ["CPR_POLEPAIRS_MISMATCH"], str(names))
        allerr = m.get_all_errors()
        check("get_all_errors covers 4 subsystems", set(allerr) ==
              {"motor", "encoder", "controller", "system"})
        m.estop()
        time.sleep(0.15)
        check("estop -> ESTOP_REQUESTED in heartbeat",
              "ESTOP_REQUESTED" in decode_flags(AXIS_ERRORS,
                                                m.state.axis_error or 0))
        check("estop -> axis idles", sim.axis_state == 1)
        m.set_axis_state(8)
        time.sleep(0.1)
        check("closed loop refused while faulted", sim.axis_state == 1)
        m.clear_errors()
        time.sleep(0.15)
        check("clear_errors clears", not m.state.any_error)

        print("SDO")
        check("sdo read vbus_voltage", m.sdo_read("vbus_voltage") == 24.0)
        back = m.sdo_write("axis0.motor.config.gear_ratio", 9.25)
        check("sdo write + verify gear_ratio", back == 9.25)
        m.set_deadman(2000)
        check("deadman via break_timeout", sim.params[73][1] == 2000)
        check("read-only endpoint rejected", _raises(
            lambda: m.sdo_write("vbus_voltage", 1.0), PermissionError))
        check("suffix path lookup", m.sdo_read("gear_ratio") == 9.25)

        print("MIT mode")
        m.set_controller_mode(3, 9)
        m.set_axis_state(8)
        time.sleep(0.1)
        fb = m.mit_control(torque=1.0)
        check("MIT echo received", fb is not None)
        check("MIT torque roundtrip (quantized)",
              abs(fb[2] - 1.0) < 2 * 100 / 4095, f"t={fb[2]:.3f}")
        m.idle()

    # full CLI commission flow against the sim (own bus on the same channel)
    print("commission (CLI end-to-end)")
    import gim_cli
    rc = gim_cli.main(["--interface", "virtual", "-c", "simnet", "commission",
                       "7", "--baud", "1000000", "--r120", "off",
                       "--deadman", "1000"])
    check("commission exits 0", rc == 0)
    check("baud 1M written", sim.params[69][1] == 1000000)
    check("r120 disabled", sim.params[72][1] == 0)
    check("deadman set", sim.params[73][1] == 1000)
    check("configuration saved", sim.saved)
    sim.saved = False
    time.sleep(0.8)  # let the sim finish its simulated reboot
    rc = gim_cli.main(["--interface", "virtual", "-c", "simnet", "commission",
                       "7", "--baud", "1000000", "--r120", "off"])
    check("idempotent: nothing to change, no save", rc == 0 and not sim.saved)

    # output-frame conversion on a fresh bus (ratio 8)
    print("output frame")
    host2 = can.Bus(interface="virtual", channel="simnet",
                    receive_own_messages=False)
    with GIM6010Bus(bus=host2, frame="output", ratio=8.0) as bus2:
        mo = bus2.motor(7)
        sim.axis_state = 8
        sim.control_mode = 1
        sim.input_mode = 1
        mo.set_input_torque(0.4)  # 0.4 Nm at output -> 0.05 Nm rotor on wire
        time.sleep(0.1)
        check("output->rotor torque /8", abs(sim.last_raw_torque - 0.05) < 1e-6,
              f"wire={sim.last_raw_torque}")
        sim.vel = 8.0  # rotor rev/s
        sim.input_torque = 0.0
        sim.control_mode = 3  # freeze the model so vel stays exact
        time.sleep(0.05)
        _, vel = mo.get_encoder_estimates()
        check("rotor->output vel /8", abs(vel - 1.0) < 1e-6, f"vel={vel}")

    sim.stop()
    sim_bus.shutdown()
    print(f"\nall {PASS} checks passed")
    return 0


def _raises(fn, exc) -> bool:
    try:
        fn()
        return False
    except exc:
        return True


if __name__ == "__main__":
    sys.exit(main())
