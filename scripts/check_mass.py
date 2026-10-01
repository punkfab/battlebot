"""check_mass.py — the weight-class gate.

Printed mass = CAD volume x density x fill (fill is a CALIBRATION parameter: weigh the first
prints and write the ratio back into params.fill). Bought parts are catalog-nominal until weighed.
"""
import sys

from _common import gate

import bot
from params import DEFAULT


def main(p=DEFAULT):
    its = bot.items(p)
    tot, table = bot.mass_table(its)
    print("mass gate")
    print(table)
    groups = {}
    for i in its:
        groups[i.group] = groups.get(i.group, 0) + i.mass_g
    print("  by sim body:", ", ".join(f"{k} {v:.1f} g" for k, v in groups.items()))
    unverified = sum(i.mass_g for i in its if i.kind != "printed")
    ok = gate(tot <= p.mass_target_g, f"total {tot:.1f} g <= target {p.mass_target_g:.0f} g",
              f"headroom {p.mass_limit_g - tot:.0f} g to the {p.mass_limit_g:.0f} g class limit; "
              f"{unverified:.0f} g is catalog-nominal (unweighed)")
    print("MASS", "GREEN" if ok else "RED")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
