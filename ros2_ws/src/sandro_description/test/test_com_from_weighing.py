import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import com_from_weighing as tool  # noqa: E402


def test_two_tilts_recover_height_and_offset():
    mass, height, offset = 4.9, 0.118, -0.006
    tilts = []
    for pitch_deg, x in ((25.0, 0.30), (-20.0, -0.28)):
        pitch = math.radians(pitch_deg)
        tilts.append((pitch, mass * (height * math.sin(pitch) + offset * math.cos(pitch)) / x, x))
    got_height, got_offset, sd_height, sd_offset, _, condition = tool.solve(
        mass, tilts, 0.5, 0.002, 0.002)
    assert got_height == pytest.approx(height, abs=1e-9)
    assert got_offset == pytest.approx(offset, abs=1e-9)
    assert sd_height < 0.004 and sd_offset < 0.002
    assert condition < 5.0


def test_payload_places_the_measured_com():
    sim = {"axle": np.array([0.01, 0.0, -0.20]), "base_mass": 3.4,
           "base_com": np.array([0.02, -0.01, -0.08])}
    mass, height, offset = 4.9, 0.11, -0.008
    extra, position = tool.payload_for(sim, mass, height, offset, keep_y=0.03)
    assert extra == pytest.approx(1.5)
    com = (sim["base_mass"] * sim["base_com"] + extra * position) / mass
    assert com[0] - sim["axle"][0] == pytest.approx(offset)
    assert com[2] - sim["axle"][2] == pytest.approx(height)
    assert position[1] == 0.03
    with pytest.raises(ValueError):
        tool.payload_for(sim, 3.0, height, offset, keep_y=0.0)


def test_write_payload_keeps_the_rest_of_the_file(tmp_path, monkeypatch):
    original = (Path(tool.IDENTIFIED)).read_text(encoding="utf-8")
    copy = tmp_path / "sim_identified.yaml"
    copy.write_text(original, encoding="utf-8")
    monkeypatch.setattr(tool, "IDENTIFIED", copy)
    tool.write_payload(1.523, [-0.012, 0.035, -0.141])
    before, after = yaml.safe_load(original), yaml.safe_load(copy.read_text(encoding="utf-8"))
    assert after["payload"]["mass_kg"] == 1.523
    assert after["payload"]["position_m"] == [-0.012, 0.035, -0.141]
    assert after["payload"]["half_size_m"] == before["payload"]["half_size_m"]
    after.pop("payload")
    before.pop("payload")
    assert after == before
