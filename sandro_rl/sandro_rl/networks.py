"""Actor and critic MLPs and the running observation normaliser."""

from __future__ import annotations

from typing import Sequence

import flax.linen as nn
import jax
import jax.numpy as jp
from flax import struct

ACTIVATIONS = {"elu": nn.elu, "tanh": nn.tanh, "relu": nn.relu, "swish": nn.swish}


class MLP(nn.Module):
    hidden: Sequence[int]
    out: int
    activation: str = "elu"
    out_scale: float = 1.0

    @nn.compact
    def __call__(self, x):
        act = ACTIVATIONS[self.activation]
        for width in self.hidden:
            x = act(nn.Dense(width, kernel_init=nn.initializers.orthogonal(jp.sqrt(2.0)))(x))
        return nn.Dense(self.out, kernel_init=nn.initializers.orthogonal(self.out_scale))(x)


@struct.dataclass
class Normalizer:
    mean: jax.Array
    var: jax.Array
    count: jax.Array

    @classmethod
    def create(cls, size: int) -> "Normalizer":
        return cls(jp.zeros(size), jp.ones(size), jp.float32(1e-4))

    def update(self, batch: jax.Array) -> "Normalizer":
        batch = batch.reshape(-1, batch.shape[-1])
        n = batch.shape[0]
        mean, var = jp.mean(batch, 0), jp.var(batch, 0)
        total = self.count + n
        delta = mean - self.mean
        new_mean = self.mean + delta * n / total
        m2 = self.var * self.count + var * n + delta ** 2 * self.count * n / total
        return Normalizer(new_mean, m2 / total, total)

    def __call__(self, x: jax.Array, clip: float = 10.0) -> jax.Array:
        return jp.clip((x - self.mean) / jp.sqrt(self.var + 1e-8), -clip, clip)
