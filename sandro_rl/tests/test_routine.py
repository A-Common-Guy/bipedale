"""The Drive page's routines replayed in the evaluator: `--scenario routine:NAME[:SPEED]`."""
from types import SimpleNamespace

import pytest

from sandro_rl import evaluate

IFACE = SimpleNamespace(v_max=0.5, yaw_max=1.5, h_rate=0.05, roll_max=0.3)


def state(**values):
    return {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "hold": True, "buttons": [], **values}


def test_sticks_become_the_commands_joy_teleop_publishes():
    command, _ = evaluate.routine_command(state(lx=1.0, ly=0.8, rx=0.5, ry=-1.0), 0.5, IFACE, 0.15, 0.15, False)
    # The slider scales the left stick only; stick right turns right (negative yaw) and
    # puts the right side down (positive roll); stick down lowers the body.
    assert command == pytest.approx([0.8 * 0.5 * 0.5, -1.0 * 0.5 * 1.5, -0.05, 0.15, 0.0])
    assert evaluate.routine_command(None, 1.0, IFACE, 0.15, 0.15, True) == ([0.0] * 5, False)


def test_cross_is_teleops_hop_and_r3_drives_the_height_home():
    command, _ = evaluate.routine_command(state(buttons=["cross"]), 1.0, IFACE, 0.15, 0.15, False)
    assert command[4] == evaluate.TELEOP_JUMP_HEIGHT_M
    command, running = evaluate.routine_command(state(), 1.0, IFACE, 0.12, 0.17, True)
    assert running and command[2] == pytest.approx(0.05)          # the rate limit, upwards
    command, running = evaluate.routine_command(state(), 1.0, IFACE, 0.169, 0.17, True)
    assert not running and command[2] == 0.0                      # within 2 mm: done
    command, running = evaluate.routine_command(state(ry=0.5), 1.0, IFACE, 0.12, 0.17, True)
    assert not running and command[2] == pytest.approx(0.025)     # the stick ends it


def test_every_routine_of_the_panel_loads_as_a_scenario():
    import yaml
    names = list(yaml.safe_load(evaluate.ROUTINES_FILE.read_text())["routines"])
    assert names
    for name in names:
        routine, speed = evaluate.load_routine(f"routine:{name}:0.7")
        assert speed == 0.7
        phases = evaluate.SCENARIOS[f"routine:{name}:0.7"]
        assert float(phases[1].split(",")[0]) == routine.duration
        assert routine.sample(routine.duration) is None
    assert evaluate.load_routine(f"routine:{names[0]}")[1] == 0.5
    with pytest.raises(ValueError):
        evaluate.load_routine("routine:no_such_routine")
