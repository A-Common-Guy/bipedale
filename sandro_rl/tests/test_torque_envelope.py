"""The drives' thermal envelope: the same law as sandro_hardware/torque_envelope.hpp
(test_torque_envelope.cpp has the same numbers), read from the description."""

import math

import numpy as np
import pytest

from sandro_rl import config as config_mod, sources, torque_envelope
from sandro_rl.interface import PolicyInterface
from sandro_rl.robot import RobotSpec


@pytest.fixture(scope="module")
def envelope():
    return sources.DriveEnvelope.from_description()


def test_description_sets_the_envelope(envelope):
    assert (envelope.peak_nm, envelope.rated_nm, envelope.time_constant_s, envelope.knee) == (9.0, 5.0, 10.0, 0.7)


def test_peak_lasts_until_the_knee_then_folds_back_to_rated(envelope):
    heat, t, dt = 0.0, 0.0, 1 / 300
    while torque_envelope.limit(heat, envelope, np) >= 9.0 - 1e-9:
        heat = torque_envelope.account(heat, torque_envelope.limit(heat, envelope, np) ** 2, dt, envelope)
        t += dt
    assert t == pytest.approx(-10.0 * math.log(1 - 17.5 / 81), abs=0.02)
    for _ in range(300 * 120):
        heat = torque_envelope.account(heat, torque_envelope.limit(heat, envelope, np) ** 2, dt, envelope)
    assert torque_envelope.limit(heat, envelope, np) == pytest.approx(5.0, abs=1e-3)


def test_ros_torque_limits_are_the_envelope_peak():
    spec = RobotSpec.load()
    iface = PolicyInterface(config_mod.load("flat_level.yaml"), spec)
    assert iface.thermal
    np.testing.assert_array_equal(iface.torque_limit, np.full(6, spec.envelope.peak_nm))
