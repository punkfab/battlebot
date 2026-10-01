"""check_parts.py — every printed part is a valid print + sim body; the design invariants hold.

  * params validate (arm clears the nose, arm misses the wheels, bot is invertible)
  * each printed part: ONE solid, watertight tessellation (MuJoCo inertia + slicer both need it)
  * invertible: wheels stand proud of the lid, so the bot still drives upside down
"""
import sys

from _common import to_mesh, gate

import bot
import parts
from params import DEFAULT


def main(p=DEFAULT):
    ok = True
    print("part gates")
    errs = p.validate()
    ok &= gate(not errs, "params validate", "; ".join(errs))
    printed = [i for i in bot.items(p) if i.kind == "printed"]
    extra = [("tyre_print", parts.tyre(p, as_printed=True))]
    for name, solid in [(i.name, i.solid) for i in printed] + extra:
        n = len(solid.solids())
        wt = to_mesh(solid).is_watertight
        ok &= gate(n == 1 and wt, f"{name:10s} single body + watertight", f"solids={n} watertight={wt}")
    # ground: at rest only the tyres and the wedge may reach the floor (the wedge must scrape it)
    its = bot.items(p)
    low = {i.name: i.solid.bounding_box().min.Z for i in its if i.solid is not None}
    intruders = {n: round(z, 2) for n, z in low.items() if z < 0.2 and not n.startswith("tyre") and n != "arm"}
    ok &= gate(not intruders, "nothing but tyres + wedge near the floor at rest", str(intruders) if intruders else "")
    ok &= gate(0.0 <= low["arm"] <= 1.0, "wedge scrapes the floor at rest (0..1 mm)", f"arm min z {low['arm']:.2f} mm")
    proud = p.wheel_d - (p.top_z + p.ground_clear)
    ok &= gate(proud >= 1.0, "invertible (wheels proud of lid)", f"{proud:.1f} mm")
    print("PARTS", "GREEN" if ok else "RED")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
