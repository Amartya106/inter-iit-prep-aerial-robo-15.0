#!/usr/bin/env python3
# Edit LOG below, then: python3 plot.py

import glob
import pandas as pd
import matplotlib.pyplot as plt

LOG = "sweep_final_recorded"      # <-- change per log
CSV = "csv_final"                                # output dir from ulog2csv

motors = pd.read_csv(glob.glob(f"{CSV}/{LOG}_actuator_motors_0.csv")[0])
pos    = pd.read_csv(glob.glob(f"{CSV}/{LOG}_vehicle_local_position_0.csv")[0])

t0 = min(motors.timestamp.iloc[0], pos.timestamp.iloc[0])
tm = (motors.timestamp - t0) / 1e6
tp = (pos.timestamp - t0) / 1e6

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

ax1.plot(tp, -pos.z, color="tab:blue", lw=1.2)
ax1.axhline(20, color="gray", ls="--", lw=0.8)
ax1.set_ylabel("altitude (m)")
ax1.grid(alpha=0.3)
ax1.set_title(LOG)
ax1.set_xlim(40, 95)

for i in range(4):
    ax2.plot(tm, motors[f"control[{i}]"], lw=0.9, label=f"motor {i}")
ax2.set_ylabel("normalised command")
ax2.set_xlabel("time (s)")
ax2.legend(ncol=4, fontsize=8)
ax2.grid(alpha=0.3)
ax2.set_ylim(0.5, 0.85) # to make it more readable

# effectiveness transition times, in seconds — read them off ulog_messages
# and paste here, e.g. TRANSITIONS = [(45.2, 100), (53.2, 75), ...]
TRANSITIONS = [(55.38, 100), (63.38, 75), (71.38, 50), (79.38, 25), (87.38, 0)]

for x, pct in TRANSITIONS:
    ax1.axvline(x, color="k", ls=":", lw=0.9)
    ax2.axvline(x, color="k", ls=":", lw=0.9)
    ax1.text(x, ax1.get_ylim()[1], f" {pct}%", va="top", fontsize=8, rotation=90)

plt.tight_layout()
plt.savefig(f"{LOG}.png", dpi=150, bbox_inches="tight")
plt.show()
