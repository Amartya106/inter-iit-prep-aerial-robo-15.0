#!/usr/bin/env python3
"""
Print effectiveness transition times for a sweep log, on the same time axis
used by plot.py (origin = first actuator_motors sample).

Usage:  python3 times.py            (edit LOG below)
"""

from pyulog import ULog

LOG = "../logs/sweep_final_recorded.ulg"      # <-- change per log

u = ULog(LOG)

t0 = [d for d in u.data_list if d.name == "actuator_motors"][0].data["timestamp"][0]

print(f"--- {LOG} ---")

print("\nconsole messages:")
for m in u.logged_messages:
    if "effectiveness" in m.message or "Sweep armed" in m.message:
        print(f"  {(m.timestamp - t0)/1e6:7.2f}s  {m.message}")

eff = [d for d in u.data_list if d.name == "rotor_effectiveness"]

if eff:
    d = eff[0].data
    print("\nrotor_effectiveness topic:")
    for ts, sc in zip(d["timestamp"], d["scale"]):
        print(f"  {(ts - t0)/1e6:7.2f}s  scale={sc:.2f}")

    print("\npaste into plot.py:")
    print("TRANSITIONS = [")
    for ts, sc in zip(d["timestamp"], d["scale"]):
        print(f"    ({(ts - t0)/1e6:.2f}, {int(round(sc*100))}),")
    print("]")
else:
    print("\n(no rotor_effectiveness topic in this log)")
