"""assembly.py — the battlebot as an ASSEMBLY IR: the top-level design artifact.

bot.items() (parts placed by mates) -> featuretree's assembly IR: one occurrence per part (its own
part-local geometry + its placement), `fastened` mates inside each rigid group, `revolute` mates for
the wheels and the lifter. Everything downstream DERIVES from this: the MJCF (featuretree
mjcf_emit), later FreeCAD/Onshape assemblies and 3MF. Nothing downstream re-describes geometry.

Printed parts are not in featuretree PART IR yet, so each occurrence's geometry rides as a
part-local STL (the `sim["meshes"]` bridge); when a part moves to the part IR its occurrence gets
`part=` and the STL goes away.

    python cad/assembly.py        -> build/battlebot.asm.json (+ meshes)
"""
from __future__ import annotations

import dataclasses
import hashlib
import json
import os

import bot
import parts
import spokes
from params import P, DEFAULT

A = spokes.load("featuretree", "assembly_ir")
BUILD = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "build"))
GROUP_ROOT = {"chassis": "chassis", "arm": "arm", "wheel_L": "hub_L", "wheel_R": "hub_R"}


def _t12(loc):
    t = loc.wrapped.Transformation()
    return [round(t.Value(r, c), 6) for r in (1, 2, 3) for c in (1, 2, 3, 4)]


def _rgb(hexc):
    return [int(hexc[k:k + 2], 16) / 255 for k in (1, 3, 5)] if hexc else None


def _export(solid, path):
    from build123d import export_stl
    if not os.path.exists(path):
        tmp = f"{path}.{os.getpid()}.tmp"                    # atomic: parallel sweeps share build/
        export_stl(solid, tmp)
        os.replace(tmp, path)
    return path


def variant_key(p: P, lift_deg=None):
    return hashlib.sha1(repr((p, lift_deg)).encode()).hexdigest()[:10]


def build(p: P = DEFAULT, lift_deg: float | None = None, mu_tyre=0.9, mu_print=0.35):
    """-> (asm, sim): the assembly IR (design facts) + the sim sidecar (meshes, masses, colliders)."""
    its = bot.items(p, lift_deg)
    gc = p.ground_clear
    mdir = os.path.join(BUILD, f"asm_{variant_key(p, lift_deg)}")
    os.makedirs(mdir, exist_ok=True)

    occs, meshes, mass = [], {}, {}
    geo = [i for i in its if i.local is not None]
    for k, i in enumerate(geo, 1):
        occs.append(A.occurrence(f"o{k}", i.name, None, _t12(i.loc), color=_rgb(i.color), component=i.group))
        meshes[i.name] = _export(i.local, os.path.join(mdir, f"{i.name}.stl"))
        mass[i.name] = i.mass_g * 1e-3
    for i in its:                                           # lumped mass rides on its group's root
        if i.local is None:
            mass[GROUP_ROOT[i.group]] += i.mass_g * 1e-3

    mates = [A.mate(f"fix_{i.name}", "fastened", GROUP_ROOT[i.group], i.name)
             for i in geo if i.name != GROUP_ROOT[i.group]]
    lift0 = p.arm_rest_deg if lift_deg is None else lift_deg
    for side, sgn in (("L", 1), ("R", -1)):
        mates.append(A.mate(f"wheel_{side}", "revolute", "chassis", f"hub_{side}",
                            axis={"origin": [p.wheel_x, sgn * p.wheel_y, p.axle_z + gc], "dir": [0, 1, 0]}))
    mates.append(A.mate("lifter", "revolute", "chassis", "arm",
                        axis={"origin": [p.pivot_x, 0, p.pivot_z + gc], "dir": [0, -1, 0]},   # + = tip up
                        limits={"value": [p.arm_rest_deg - lift0 - 3.0, p.arm_max_deg - lift0]}))

    asm = A.assembly("battlebot", occs, mates=mates,
                     provenance={"generator": "battlebot/cad/assembly.py", "params": dataclasses.asdict(p),
                                 "lift_deg": lift_deg, "units": "mm, BOT frame (origin on the ground)"})

    proxies = []
    for k, solid in enumerate(parts.collision_pieces(p)["arm"]):
        proxies.append(_export(solid, os.path.join(mdir, f"arm_proxy{k}.stl")))
    outer = {"chassis": "hull", "lid": "hull", "tyre_L": "hull", "tyre_R": "hull", "arm": proxies}
    sim = {"meshes": meshes, "mass_kg": mass,
           "collision": {i.name: outer.get(i.name, "none") for i in geo},
           "friction": {i.name: (mu_tyre if i.name.startswith("tyre") else mu_print) for i in geo},
           "rgba": {i.name: " ".join(f"{c:.3f}" for c in _rgb(i.color)) + " 1" for i in geo}}
    return asm, sim


def problems(asm):
    return A.mate_tree_problems(asm) + A.closure_problems(asm)


if __name__ == "__main__":
    asm, sim = build()
    assert not problems(asm), problems(asm)
    path = os.path.join(BUILD, "battlebot.asm.json")
    with open(path, "w") as f:
        json.dump(asm, f, indent=1)
    mob = A.mobility(asm)
    print(f"{path}: {len(asm['occurrences'])} occurrences, {len(asm['mates'])} mates, "
          f"mobility {mob['mobility']} (2 wheels + lifter)")
