import csv
import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

import imu_delay  # noqa: E402
from compare_runs import load  # noqa: E402


def _rocking_recording(path, imu_delay_s, encoder_delay_s, imu_hz=200.0, seconds=30.0, seed=0):
    """Hand rocking with blocked wheels, sampled the way the controller receives it:
    each sensor on its own clock, delivered after its delay, read at 300 Hz.

    Returns the true lag: the mean age of a new IMU sample when the controller
    first reads it, minus the same for the encoder. With fixed clock phases the
    wait for the next 300 Hz read differs between the streams, so this is not
    exactly the difference of the transport delays."""
    rng = np.random.default_rng(seed)
    freqs = rng.uniform(0.4, 4.0, 12)
    amps = 0.25 / freqs / np.sqrt(freqs.size)
    phases = rng.uniform(0.0, 2.0 * np.pi, freqs.size)

    def pitch(t):
        return np.sum(amps * np.sin(2.0 * np.pi * freqs * np.asarray(t)[..., None] + phases), axis=-1)

    def pitch_rate(t):
        return np.sum(2.0 * np.pi * freqs * amps * np.cos(
            2.0 * np.pi * freqs * np.asarray(t)[..., None] + phases), axis=-1)

    def sampled(cycle, rate_hz, phase, delay):
        return phase + np.floor((cycle - delay - phase) * rate_hz) / rate_hz

    cycles = np.arange(0.5, seconds, 1.0 / 300.0)
    t_imu = sampled(cycles, imu_hz, 0.0013, imu_delay_s)
    t_enc = sampled(cycles, 200.0, 0.0031, encoder_delay_s)
    gyro_noise = {}
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t_s", "stream", "phase", "pos_right_wheel", "pos_left_wheel",
                         "imu_qx", "imu_qy", "imu_qz", "imu_qw", "imu_gx", "imu_gy", "imu_gz"])
        for cycle, ti, te in zip(cycles, t_imu, t_enc):
            received = cycle + rng.uniform(0.0, 0.0005)
            wheel = -pitch(te)
            writer.writerow([f"{received:.5f}", "joint", 0, f"{wheel:.6f}", f"{wheel:.6f}",
                             "", "", "", "", "", "", ""])
            theta = pitch(ti)
            noise = gyro_noise.setdefault(round(ti, 6), rng.normal(0.0, 0.01))
            writer.writerow([f"{received:.5f}", "imu", 0, "", "",
                             0.0, np.sin(theta / 2.0), 0.0, np.cos(theta / 2.0),
                             0.0, pitch_rate(ti) + noise, 0.0])

    def first_read_age(sample_times):
        new = np.concatenate([[True], np.diff(sample_times) > 0])
        return float(np.mean(cycles[new] - sample_times[new]))

    return first_read_age(t_imu) - first_read_age(t_enc)


@pytest.mark.parametrize("imu_hz,imu_delay_s", [(200.0, 0.012), (200.0, 0.004), (30.0, 0.025)])
def test_recovers_imu_delay_relative_to_encoders(tmp_path, imu_hz, imu_delay_s):
    path = tmp_path / "rock.csv"
    expected = _rocking_recording(path, imu_delay_s, 0.002, imu_hz=imu_hz)
    assert abs(expected - (imu_delay_s - 0.002)) < 0.0034
    result = imu_delay.estimate(load(str(path)))
    assert abs(result["imu_hz"] - imu_hz) < 0.1 * imu_hz
    for channel in ("orientation", "gyro"):
        assert result[channel]["lag_s"] == pytest.approx(expected, abs=0.0015), channel
        assert result[channel]["corr"] < -0.95, channel


def test_opposite_wheel_signs_do_not_cancel(tmp_path):
    path = tmp_path / "rock.csv"
    expected = _rocking_recording(path, 0.010, 0.002)
    run = load(str(path))
    run["joint"]["pos_left_wheel"] = -run["joint"]["pos_left_wheel"]
    result = imu_delay.estimate(run)
    assert result["orientation"]["lag_s"] == pytest.approx(expected, abs=0.0015)
