import os
import shutil
from pathlib import Path

import pytest

# Tests run next to a training job: take GPU memory on demand only.
os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")

import jax  # noqa: E402

# Tests compare against float64 references (mj_ray, numpy); on GPUs with
# TF32 the default float32 matmul keeps about three digits.
jax.config.update("jax_default_matmul_precision", "highest")

from sandro_rl import sources  # noqa: E402


@pytest.fixture
def model_copy(tmp_path: Path):
    """A copy of the generated MJCF folder; returns (scene path, robot path)."""
    target = tmp_path / "mjcf"
    shutil.copytree(sources.SCENE.parent, target)
    return target / sources.SCENE.name, target / "robot_ros2.xml"
