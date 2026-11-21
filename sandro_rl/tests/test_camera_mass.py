"""evaluate.py's --sim camera_mass: the D435 and its bracket taken off the model, which is
the robot the policies of before 2026-11-18 were trained and measured on (DESIGN.md 27)."""

import mujoco
import numpy as np
import pytest

from sandro_rl import evaluate, sources


def test_the_camera_comes_off_the_model_and_nothing_else_changes():
    with_camera = mujoco.MjModel.from_xml_path(str(sources.SCENE))
    without = mujoco.MjModel.from_xml_path(str(sources.SCENE))
    evaluate.scale_camera_mass(without, 0.0)
    torso = with_camera.body("torso").id
    camera_kg = sources.identified()["masses"]["camera_kg"]
    assert with_camera.body_subtreemass[torso] - without.body_subtreemass[torso] == pytest.approx(camera_kg, abs=1e-6)
    others = [b for b in range(with_camera.nbody) if b != with_camera.body("camera").id]
    assert np.array_equal(with_camera.body_mass[others], without.body_mass[others])
    assert np.array_equal(with_camera.body_inertia[others], without.body_inertia[others])


def test_the_option_is_read_and_is_off_by_default():
    latency = sources.Latency.from_launch("encoder")
    assert evaluate.SimOptions.from_closed_loop({}, latency).camera_mass_scale == 1.0
    assert evaluate.SimOptions.from_closed_loop({"camera_mass": "0"}, latency).camera_mass_scale == 0.0
