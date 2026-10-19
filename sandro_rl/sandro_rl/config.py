"""YAML configuration with inheritance and dotted command-line overrides.

A task file names its parent with `inherit:` (relative to its own folder)
and only states what differs. `--set a.b.c=value` overrides any key; the
value is parsed as YAML, so numbers, booleans and lists work.
"""

from __future__ import annotations

import copy
from pathlib import Path

import yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / "configs"


def _merge(base: dict, extra: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict) and not value.get("_replace", False):
            out[key] = _merge(out[key], value)
        else:
            if isinstance(value, dict):
                value = {k: v for k, v in value.items() if k != "_replace"}
            out[key] = copy.deepcopy(value)
    return out


def _load(path: Path) -> dict:
    document = yaml.safe_load(path.read_text()) or {}
    parent = document.pop("inherit", None)
    config = _load(path.parent / parent) if parent else {}
    return _merge(config, document)


def load(path: str | Path, overrides: list[str] | None = None) -> dict:
    path = Path(path)
    if not path.is_file() and (CONFIG_DIR / path).is_file():
        path = CONFIG_DIR / path
    if not path.is_file() and (CONFIG_DIR / f"{path}.yaml").is_file():
        path = CONFIG_DIR / f"{path}.yaml"
    config = _load(path)
    for item in overrides or []:
        key, _, text = item.partition("=")
        set_path(config, key, yaml.safe_load(text))
    config.setdefault("name", path.stem)
    return config


def set_path(config: dict, dotted: str, value) -> None:
    node = config
    parts = dotted.split(".")
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value


def get_path(config: dict, dotted: str, default=None):
    node = config
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node
