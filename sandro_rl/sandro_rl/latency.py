"""Sensor latency on the GPU: the sample-and-hold delay line of sandro_sim/DelayLine.

Every physics step writes the true (noisy) sensor values into a ring buffer.
A reader at time t sees, per channel, the sample taken at the newest instant
of that channel's clock (phase + k / rate) whose transport delay has elapsed
by t, i.e. sampled at  phase + floor((t - delay - phase) * rate) / rate,
reconstructed by linear interpolation between the two physics steps around
it. With rate 0 the channel is sampled continuously and only delayed. This
is the timing model of DelayLine with the history at physics resolution
instead of at the 300 Hz consumer's updates.
"""

from __future__ import annotations

import jax.numpy as jp


def write(history: jp.ndarray, step: jp.ndarray, values: jp.ndarray) -> jp.ndarray:
    """history [L, C]; step: index of the physics step `values` belong to."""
    length = history.shape[0]
    return history.at[step % length].set(values)


def sample_time(now_s, delay_s, period_s, phase_s):
    """When the sample a reader sees at now_s was taken (vectorised over channels)."""
    continuous = now_s - delay_s
    period = jp.maximum(period_s, 1e-9)
    gridded = phase_s + jp.floor((now_s - delay_s - phase_s) / period + 1e-6) * period
    return jp.where(period_s > 0.0, gridded, continuous)


def read(history: jp.ndarray, step: jp.ndarray, dt: float, sample_s: jp.ndarray) -> jp.ndarray:
    """Per-channel value at sample_s [C] from history [L, C] holding steps up to `step`.

    Times before the episode start hold the first sample, as DelayLine holds
    the first value rather than emit zeros.
    """
    length = history.shape[0]
    position = jp.clip(sample_s / dt, 0.0, step.astype(jp.float32))
    position = jp.maximum(position, step.astype(jp.float32) - (length - 2))
    i0 = jp.floor(position).astype(jp.int32)
    weight = position - i0
    i1 = jp.minimum(i0 + 1, step)
    channels = jp.arange(history.shape[1])
    v0 = history[i0 % length, channels]
    v1 = history[i1 % length, channels]
    return v0 + weight * (v1 - v0)
