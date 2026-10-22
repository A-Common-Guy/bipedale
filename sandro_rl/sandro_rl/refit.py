"""The curriculum of a fine-tune onto the wheel drive-train lag.

A fine-tune starts where the task's curriculum ended (every stage's settings
from the first iteration) and ramps the wheel lag in: up to 30 ms first, then
base.yaml's 10-80 ms from iteration 300. Prints the --set argument for train:

    uv run python -m sandro_rl.train configs/agile_terrain.yaml --resume policies/agile_terrain \\
        --set "$(uv run python -m sandro_rl.refit configs/agile_terrain.yaml)" ...

scripts/refit_lag.sh runs it for every policy.
"""

from __future__ import annotations

import argparse
import json

from . import config as config_mod
from .train import curriculum_stages

LAG_KEY = "env.randomization.drives.wheel_lag_s"


def lag_stages(config: dict, ramp=(0.0, 0.03), ramp_iterations: int = 300) -> list[dict]:
    final = {}
    for stage in curriculum_stages(config):
        final.update(stage["set"])
    full = config_mod.get_path(config, LAG_KEY)
    return [{"iteration": 0, "set": {**final, LAG_KEY: list(ramp)}},
            {"iteration": ramp_iterations, "set": {LAG_KEY: full}}]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("config")
    args = parser.parse_args()
    print("curriculum.stages=" + json.dumps(lag_stages(config_mod.load(args.config))))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
