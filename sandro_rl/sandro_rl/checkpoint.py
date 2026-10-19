"""Checkpoints: network parameters, normalisers, config and the model fingerprint."""

from __future__ import annotations

import json
import platform
import warnings
from pathlib import Path

import jax
import numpy as np
from flax import serialization

from . import sources


def save(directory: Path, ts, config: dict, contract: dict, extra: dict | None = None,
         name: str = "checkpoint") -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    payload = {
        "params": jax.device_get(ts.params),
        "actor_norm": {"mean": np.asarray(ts.actor_norm.mean), "var": np.asarray(ts.actor_norm.var),
                       "count": np.asarray(ts.actor_norm.count)},
        "critic_norm": {"mean": np.asarray(ts.critic_norm.mean), "var": np.asarray(ts.critic_norm.var),
                        "count": np.asarray(ts.critic_norm.count)},
    }
    (directory / f"{name}.msgpack").write_bytes(serialization.msgpack_serialize(
        jax.tree.map(np.asarray, payload)))
    import importlib.metadata as md
    meta = {
        "config": config,
        "fingerprint": sources.fingerprint(),
        "contract": contract,
        "versions": {p: md.version(p) for p in ("mujoco", "mujoco-mjx", "mujoco-warp", "jax", "flax")},
        "python": platform.python_version(),
        **(extra or {}),
    }
    (directory / "meta.json").write_text(json.dumps(meta, indent=1, default=float))
    return directory / f"{name}.msgpack"


def load(path: Path) -> tuple[dict, dict]:
    path = Path(path)
    if path.is_dir():
        path = path / "checkpoint.msgpack"
    payload = serialization.msgpack_restore(path.read_bytes())
    meta = json.loads((path.parent / "meta.json").read_text())
    return payload, meta


def check_model(meta: dict, strict: bool = False) -> list[str]:
    """Warn (or raise) when the robot model changed since the checkpoint was trained."""
    changed = sources.compare_fingerprints(meta.get("fingerprint", {}))
    if changed:
        message = ("the robot model or its parameters changed since this policy was trained:\n  "
                   + "\n  ".join(changed)
                   + "\nRe-run the consistency check and see docs/DESIGN.md (refresh procedure).")
        if strict:
            raise RuntimeError(message)
        warnings.warn(message, stacklevel=2)
    return changed
