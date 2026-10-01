"""check_ir.py — the ASSEMBLY IR is the top-level artifact; everything derived from it must agree.

  * IR well-formed: mate tree + closures valid; mobility = 3 (two wheels + lifter)
  * MJCF (featuretree mjcf_emit) carries exactly the CAD mass roll-up, on the expected rigid bodies
  * 3MF (featuretree threemf_emit) round-trips: every occurrence placed where the CAD puts it
"""
import os
import sys

os.environ.setdefault("MUJOCO_GL", "osmesa")
from _common import gate, np, ROOT

import assembly
import bot
import spokes
from params import DEFAULT


def main(p=DEFAULT):
    A = spokes.load("featuretree", "assembly_ir")
    MJ = spokes.load("featuretree", "mjcf_emit")
    E = spokes.load("featuretree", "threemf_emit")
    import mujoco
    ok = True
    print("assembly-IR gates")
    asm, sim = assembly.build(p)
    probs = assembly.problems(asm)
    ok &= gate(not probs, "IR well-formed (mate tree + closures)", "; ".join(probs))
    mob = A.mobility(asm)["mobility"]
    ok &= gate(mob == 3, "mobility = 3 (wheel_L, wheel_R, lifter)", f"{mob}")

    m = mujoco.MjModel.from_xml_string(MJ.emit(asm, assembly.BUILD, dict(sim, free_root=True)))
    bodies = {m.body(i).name: m.body_mass[i] * 1e3 for i in range(1, m.nbody)}
    roll = sum(i.mass_g for i in bot.items(p))
    ok &= gate(set(bodies) == {"chassis", "hub_L", "hub_R", "arm"}, "MJCF rigid bodies = fastened groups", str(sorted(bodies)))
    ok &= gate(abs(sum(bodies.values()) - roll) < 0.05, "MJCF mass = CAD roll-up",
               f"{sum(bodies.values()):.2f} g vs {roll:.2f} g")

    f = os.path.join(assembly.BUILD, "check_ir.3mf")
    E.emit_assembly(asm, f, meshes=sim["meshes"])
    back = dict(E.read_assembly(f))
    worst = 0.0
    for i in bot.items(p):
        if i.solid is None:
            continue
        bb = i.solid.bounding_box()
        v = back[i.name]
        worst = max(worst, float(np.abs(v.min(0) - [bb.min.X, bb.min.Y, bb.min.Z]).max()),
                    float(np.abs(v.max(0) - [bb.max.X, bb.max.Y, bb.max.Z]).max()))
    ok &= gate(len(back) == len(asm["occurrences"]) and worst < 0.05, "3MF round-trip places every part",
               f"{len(back)} parts, worst bbox error {worst:.3f} mm")
    ok &= gate(E.read_embedded_ir(f)["name"] == "battlebot", "IR embedded in the 3MF package")
    print("IR", "GREEN" if ok else "RED")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
