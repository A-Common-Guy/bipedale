"""Spectrum of the RL policy's actions from its 50 Hz diagnostics.

    python3 buzz_actions.py BAG
"""
import sys
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]  # buzz_spectrum.py, here or in recordings/
from buzz_spectrum import welch

reader = SequentialReader()
reader.open(StorageOptions(uri=sys.argv[1], storage_id="mcap"), ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
rows = []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic.endswith("/diagnostics") and "rl_" in topic:
        msg = deserialize_message(data, get_message(types[topic]))
        values = {}
        for s in msg.status:
            for v in s.values:
                try:
                    values[v.key] = float(v.value)
                except ValueError:
                    pass
        rows.append((stamp * 1e-9, values))
print(f"{len(rows)} diagnostics, {len(rows) / (rows[-1][0] - rows[0][0]):.0f} Hz; keys:",
      ", ".join(k for k in rows[0][1] if not k.startswith("obs"))[:900])
t = [r[0] for r in rows]
for key in [f"action_{i}" for i in range(6)] + [k for k in rows[0][1] if "oscillation" in k]:
    x = np.array([r[1].get(key, np.nan) for r in rows])
    if np.isnan(x).all():
        continue
    f, p, fs = welch(t, x, segment=128)
    df = f[1] - f[0]
    band = (f >= 5) & (f <= 15)
    low = (f > 0) & (f < 5)
    print(f"{key:28s} mean {x.mean():+.3f}  std {x.std():.3f}  RMS 5-15 Hz {np.sqrt(p[band].sum() * df):.3f} "
          f"(peak {f[band][np.argmax(p[band])]:.1f} Hz)  RMS <5 Hz {np.sqrt(p[low].sum() * df):.3f}")
