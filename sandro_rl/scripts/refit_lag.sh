#!/usr/bin/env bash
# Fine-tune policies onto the wheel drive-train lag (configs/base.yaml
# wheel_lag_s), one after the other: runs/<policy>_lag. See sandro_rl/refit.py.
#
#     scripts/refit_lag.sh flat_tracking agile_terrain jump step_climb stand_up
set -euo pipefail
cd "$(dirname "$0")/.."
PY=${PY:-.venv/bin/python}
for p in "$@"; do
  stages=$($PY -m sandro_rl.refit "configs/$p.yaml")
  echo "=== $p $(date -Is)"
  $PY -m sandro_rl.train "configs/$p.yaml" --out "runs/${p}_lag" --resume "policies/$p" \
      --set "$stages" --set train.iterations=1000 --set train.learning_rate=3.0e-4 \
      --set train.resume_log_std=-1.2 > "runs/${p}_lag.log" 2>&1
done
echo "refit done"
