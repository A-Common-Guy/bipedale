#!/usr/bin/env bash
# Replace policies/<policy> by its wheel-lag refit runs/<policy>_lag (see
# refit_lag.sh): checkpoint, ONNX, and the refit's curves as *_lag; the
# lag sweep before the refit is kept as eval/lag_sweep_before.md. Then
# re-evaluates: --lag-sweep and --compare at the nominal 40 ms lag.
set -euo pipefail
cd "$(dirname "$0")/.."
PY=${PY:-.venv/bin/python}
for p in "$@"; do
  src=runs/${p}_lag dst=policies/$p
  [ -f "$src/policy.onnx" ] || JAX_PLATFORMS=cpu $PY -m sandro_rl.export "$src"
  [ -f "$dst/eval/lag_sweep_before.md" ] || mv "$dst/eval/lag_sweep.md" "$dst/eval/lag_sweep_before.md"
  cp "$src"/{checkpoint.msgpack,meta.json,policy.onnx,policy.json} "$dst/"
  cp "$src/train_log.csv" "$dst/train_log_lag.csv"
  cp "$src/train_summary.json" "$dst/train_summary_lag.json"
  cp "$src/curves.png" "$dst/curves_lag.png"
  JAX_PLATFORMS=cpu $PY -m sandro_rl.evaluate "$dst" --lag-sweep > /dev/null
  JAX_PLATFORMS=cpu $PY -m sandro_rl.evaluate "$dst" --compare --sim wheel_lag=0.04 > /dev/null
  echo "packaged $p"
done
