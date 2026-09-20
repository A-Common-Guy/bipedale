"""Loading, validation, and hashing for identified MuJoCo parameters."""
from __future__ import annotations

import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


def model_sha256(model_path: str | Path) -> str:
    """Hash an MJCF file and its recursively included XML files."""
    root_path = Path(model_path).resolve()
    digest = hashlib.sha256()
    visited: set[Path] = set()

    def visit(path: Path) -> None:
        path = path.resolve()
        if path in visited:
            return
        visited.add(path)
        content = path.read_bytes()
        try:
            identity = path.relative_to(root_path.parent)
        except ValueError:
            identity = Path(path.name)
        digest.update(str(identity).encode())
        digest.update(content)
        try:
            xml = ET.fromstring(content)
        except ET.ParseError:
            return
        for include in xml.findall(".//include"):
            filename = include.get("file")
            if filename:
                visit(path.parent / filename)

    visit(root_path)
    return digest.hexdigest()


def load_identified(path: str | Path, model_path: str | Path) -> dict | None:
    path = Path(path)
    if not path.exists():
        return None
    with path.open(encoding="utf-8") as stream:
        values = yaml.safe_load(stream) or {}
    if not values.get("enabled", False):
        return None
    expected = str(values.get("model_sha256", ""))
    actual = model_sha256(model_path)
    if not expected or expected != actual:
        raise ValueError(
            "identified parameter model hash does not match the configured MJCF"
        )
    groups = values.get("groups", {})
    for name in ("leg", "wheel"):
        group = groups.get(name)
        if not isinstance(group, dict):
            raise ValueError(f"identified parameters missing {name} group")
        for key in ("damping", "frictionloss", "armature", "torque_scale"):
            value = float(group[key])
            if value < 0:
                raise ValueError(f"identified {name}.{key} must be non-negative")
    delay = float(values.get("command_delay_s", 0.0))
    if not 0.0 <= delay <= 0.1:
        raise ValueError("identified command_delay_s must be between 0 and 0.1")
    return values
