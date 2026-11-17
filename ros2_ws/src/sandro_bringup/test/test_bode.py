"""bode.py on simulated closed loops: a body behind an 8 Hz compliance, held by a
rate feedback that also fights a disturbance, and swept by the WBC's chirp."""

import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
spec = importlib.util.spec_from_file_location("bode", TOOLS / "bode.py")
bode = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bode)

FN, ZETA, GAIN = 8.0, 0.3, 10.0      # the plant: GAIN / s * wn^2 / (s^2 + 2 zeta wn s + wn^2)
FEEDBACK = 1.5                       # u = r - FEEDBACK * y: loop gain 0.74 at 4 Hz, 0.5 at the mode
F0, F1, DURATION, AMPLITUDE = 2.0, 25.0, 20.0, 0.3


def plant(f):
    return GAIN * bode.mode_shape(f, -1, FN, ZETA, 0.0)


def closed_loop(disturbance=0.02, cutoff=5.0, seed=0, lead_s=1.0, sub=5):
    """(t, r, u, y) at 300 Hz, the chirp starting lead_s after t = 0, with a body
    sway (white noise low-passed at cutoff) the controller fights; the gyro's
    coherence with the chirp comes out about 0.7, as on the robot. RK4."""
    rng = np.random.default_rng(seed)
    dt = 1 / (sub * bode.FS)
    n = int((DURATION + 2 * lead_s) * bode.FS) * sub
    t = np.arange(n) * dt
    r = bode.chirp(t - lead_s, F0, F1, DURATION, AMPLITUDE)
    white = rng.normal(size=n // sub + 1).repeat(sub)[:n]
    d = np.zeros(n)
    a = 1 - math.exp(-2 * math.pi * cutoff * dt)
    for k in range(1, n):
        d[k] = d[k - 1] + a * (white[k] - d[k - 1])
    d *= disturbance / d.std()
    wn = 2 * math.pi * FN

    def deriv(x, u):                     # x: v (behind the compliance), v', y (the rate)
        return np.array([x[1], wn ** 2 * (u - x[0]) - 2 * ZETA * wn * x[1], GAIN * x[0]])

    x, u, y = np.zeros(3), np.zeros(n), np.zeros(n)
    for k in range(n):
        y[k] = x[2] + d[k]
        u[k] = r[k] - FEEDBACK * y[k]
        k1 = deriv(x, u[k])
        k2 = deriv(x + 0.5 * dt * k1, u[k])
        k3 = deriv(x + 0.5 * dt * k2, u[k])
        k4 = deriv(x + dt * k3, u[k])
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return t[::sub], r[::sub], u[::sub], y[::sub]


@pytest.fixture(scope="module")
def loop():
    return closed_loop()


def test_the_chirp_is_the_wbc_formula():
    # whole_body_core.cpp sysid_excitation_nm, mode 2, by hand.
    rate = (F1 - F0) / DURATION
    sweep = lambda t: AMPLITUDE * math.sin(2 * math.pi * (F0 * t + 0.5 * rate * t * t))
    assert bode.chirp(1.234, F0, F1, DURATION, AMPLITUDE) == pytest.approx(sweep(1.234))
    taper = 0.5 * (1 - math.cos(math.pi * 0.25 / bode.TAPER_S))
    assert bode.chirp(0.25, F0, F1, DURATION, AMPLITUDE) == pytest.approx(taper * sweep(0.25))
    assert bode.chirp(-0.1, F0, F1, DURATION) == 0.0 and bode.chirp(DURATION + 0.1, F0, F1, DURATION) == 0.0


def test_the_chirp_start_is_found_in_the_measured_torque(loop):
    t, _, u, _ = loop
    # plan.json logged the trigger 137 ms before the controller started the sweep.
    event = {"t": 1.0 - 0.137, "f0_hz": F0, "f1_hz": F1, "duration_s": DURATION}
    offset, match = bode.align(t, u, event)
    assert offset == pytest.approx(0.137, abs=1.5 / bode.FS)
    assert match > 0.5


def test_through_the_chirp_the_bode_plot_and_the_mode_are_the_plant(loop):
    t, r, u, y = loop
    f, h, _, coherence = bode.frequency_response(bode.spectra(r, u, y))
    band = (f >= 4.0) & (f <= 20.0)
    assert np.median(coherence[band]) == pytest.approx(0.72, abs=0.12)
    error = abs(h[band] - plant(f[band])) / abs(plant(f[band]))
    assert np.median(error) < 0.25            # 0.11-0.21 over seeds 1-8
    mode = bode.fit_mode(f, h, coherence, (3.0, 22.0), slope=-1)
    assert mode["fn_hz"] == pytest.approx(FN, abs=0.4)
    assert mode["zeta"] == pytest.approx(ZETA, abs=0.06)
    assert mode["k"] == pytest.approx(GAIN, rel=0.15)


def test_dividing_by_the_measured_torque_is_biased_where_the_loop_is_strong():
    """The same loop built exactly in the frequency domain, 437 s long so the
    averaging leaves only the bias: a disturbance as large as the chirp's answer
    (coherence 0.5), which the controller's reaction carries into u."""
    rng = np.random.default_rng(1)
    n = 2 ** 17
    f = np.fft.rfftfreq(n, 1 / bode.FS)
    p = np.zeros(len(f), complex)
    p[1:] = plant(f[1:])
    s = 1 / (1 + FEEDBACK * p)
    r_f = np.where((f >= 2) & (f <= 25), np.exp(2j * np.pi * rng.random(len(f))), 0)
    d_f = abs(p) * (rng.normal(size=len(f)) + 1j * rng.normal(size=len(f))) / math.sqrt(2) * ((f > 0.5) & (f < 30))
    r = np.fft.irfft(r_f, n)
    u = np.fft.irfft(s * r_f - FEEDBACK * s * d_f, n)
    y = np.fft.irfft(p * s * r_f + s * d_f, n)
    fe, h, h_direct, _ = bode.frequency_response(bode.spectra(r, u, y))

    def errors(lo, hi):
        b = (fe >= lo) & (fe <= hi)
        true = plant(fe[b])
        return np.median(abs(h[b] - true) / abs(true)), np.median(abs(h_direct[b] - true) / abs(true))

    ref, direct = errors(3, 6)           # loop gain about 1
    assert ref < 0.1 and direct > 0.3
    ref, direct = errors(12, 20)         # loop gain under 0.3: both hold
    assert ref < 0.1 and direct < 0.15


def test_a_lightly_damped_mode_is_a_peak_and_a_well_damped_one_only_a_fit():
    f = np.arange(0.25, 150, 0.25)
    coherence = np.ones_like(f)
    peaks = bode.resonances(f, 3.0 * bode.mode_shape(f, -1, 8.0, 0.1, 0.0), coherence, (3.0, 22.0), slope=-1)
    assert len(peaks) == 1 and peaks[0]["f_hz"] == pytest.approx(8.0, abs=0.25)
    assert peaks[0]["zeta"] == pytest.approx(0.1, abs=0.02)
    h = 3.0 * bode.mode_shape(f, -1, 12.5, 0.5, 0.004)
    assert bode.resonances(f, h, coherence, (3.0, 22.0), slope=-1) == []
    mode = bode.fit_mode(f, h, coherence, (3.0, 22.0), slope=-1)
    assert mode["fn_hz"] == pytest.approx(12.5, abs=0.1)
    assert mode["zeta"] == pytest.approx(0.5, abs=0.01)
    assert mode["delay_ms"] == pytest.approx(4.0, abs=0.5)


def test_analyse_a_recording(tmp_path, loop, monkeypatch):
    """The whole tool on the simulated loop as a yaw recording: the right wheel
    takes +u, the left -u, the gyro's z is y."""
    t, _, u, y = loop
    t0 = 1794525338.0                                    # a robot clock
    efforts = np.zeros((len(t), 6))
    efforts[:, bode.JOINTS.index("right_wheel")] = u
    efforts[:, bode.JOINTS.index("left_wheel")] = -u
    joints = np.column_stack([t + t0, efforts])
    gyro = np.column_stack([t + t0, np.zeros_like(y), np.zeros_like(y), y])
    monkeypatch.setattr(bode, "read_bag", lambda path: (joints, gyro))
    (tmp_path / "plan.json").write_text(json.dumps({"events": [
        {"kind": "start", "t": t0},
        {"kind": "chirp", "kp": 12.0, "amplitude_nm": AMPLITUDE, "f0_hz": F0, "f1_hz": F1,
         "duration_s": DURATION, "t": t0 + 0.95}]}))
    result = bode.analyse(tmp_path)
    assert result["axis"] == "yaw" and result["band_hz"] == [3.0, 22.0] and result["slope"] == -1
    assert result["per_chirp"][0]["start_offset_s"] == pytest.approx(0.05, abs=1.5 / bode.FS)
    assert result["mode"]["fn_hz"] == pytest.approx(FN, abs=0.4)
    saved = json.loads((tmp_path / "bode_yaw.json").read_text())
    assert saved["mode"] == result["mode"] and len(saved["f_hz"]) == len(saved["h"]) == len(saved["coherence"])
    with pytest.raises(ValueError, match="no roll chirp"):
        bode.analyse(tmp_path, axis="roll")
