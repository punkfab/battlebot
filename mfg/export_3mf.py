"""export_3mf.py — the battlebot delivered as 3MF (3D Manufacturing Format).

  build/battlebot.3mf        the ASSEMBLY: every part placed (components object), materials,
                             the assembly IR embedded -> opens in any 3MF viewer/slicer
  build/battlebot_lift.3mf   the same assembly POSED (lifter up) by featuretree's kinematic solver
  build/plate_<MAT>.3mf      PRINT PLATES: printed parts in print orientation (cad/parts.PRINT),
                             shelf-packed on the bed, one plate per material — what the print cell eats

    python mfg/export_3mf.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "cad"))

import numpy as np            # noqa: E402

import assembly              # noqa: E402
import bot                   # noqa: E402
import parts                 # noqa: E402
import spokes                # noqa: E402
from params import DEFAULT   # noqa: E402

BED_MM = (256.0, 256.0)      # Bambu X1/P1-class bed; the print cell's real bed goes here
GAP_MM = 6.0


def _loc_matrix(loc):
    t = loc.wrapped.Transformation()
    M = np.eye(4)
    for r in range(3):
        for c in range(4):
            M[r, c] = t.Value(r + 1, c + 1)
    return M


def plates(p=DEFAULT):
    """-> {material: [object dict]} with bed transforms; raises if a part can't fit the bed."""
    from build123d import export_stl
    its = [i for i in bot.items(p) if i.kind == "printed"]
    by_mat = {}
    for i in its:
        local = parts.tyre(p, as_printed=True) if i.name.startswith("tyre") else i.local
        rot, note = parts.PRINT[parts.print_key(i.name)]
        oriented = rot * local
        bb = oriented.bounding_box()
        by_mat.setdefault(i.material, []).append((i, local, rot, bb, note))
    out = {}
    for mat, rows in by_mat.items():
        rows.sort(key=lambda r: -r[3].size.Y)                 # shelf pack, tallest-in-Y first
        x = y = shelf = 0.0
        objs = []
        for i, local, rot, bb, note in rows:
            w, h = bb.size.X, bb.size.Y
            if w > BED_MM[0] or h > BED_MM[1]:
                raise ValueError(f"{i.name} ({w:.0f}x{h:.0f} mm) does not fit the {BED_MM} bed")
            if x + w > BED_MM[0]:
                x, y, shelf = 0.0, y + shelf + GAP_MM, 0.0
            if y + h > BED_MM[1]:
                raise ValueError(f"plate {mat} overflows the bed at {i.name} — needs a second plate")
            R = _loc_matrix(rot)
            T = np.eye(4)
            T[:3, 3] = [x - bb.min.X, y - bb.min.Y, -bb.min.Z]   # min corner on the bed origin, z=0
            path = os.path.join(assembly.BUILD, f"print_{i.name}.stl")
            export_stl(local, path)
            objs.append({"name": i.name, "stl": path, "transform": T @ R, "material": mat,
                         "color": [int(i.color[k:k + 2], 16) / 255 for k in (1, 3, 5)], "note": note})
            x += w + GAP_MM
            shelf = max(shelf, h)
        out[mat] = objs
    return out


def main():
    E = spokes.load("featuretree", "threemf_emit")
    asm, sim = assembly.build(DEFAULT)
    mats = {i.name: (i.material or "bought") for i in bot.items(DEFAULT) if i.local is not None}
    b = assembly.BUILD
    r = E.emit_assembly(asm, os.path.join(b, "battlebot.3mf"), meshes=sim["meshes"], materials=mats)
    print(f"build/battlebot.3mf       assembly: {r['objects']} meshes, {r['components']} placed, IR embedded")
    lift = 60.0 - DEFAULT.arm_rest_deg
    r = E.emit_assembly(asm, os.path.join(b, "battlebot_lift.3mf"), meshes=sim["meshes"], materials=mats,
                        pose={"lifter": lift})
    print(f"build/battlebot_lift.3mf  posed by the solver: lifter +{lift:.0f}° (arm at 60°)")
    for mat, objs in plates().items():
        path = os.path.join(b, f"plate_{mat}.3mf")
        E.emit_objects(objs, path, metadata={"Title": f"battlebot {mat} plate",
                                             "Description": "; ".join(f"{o['name']}: {o['note']}" for o in objs)})
        print(f"build/plate_{mat}.3mf    {len(objs)} parts: {', '.join(o['name'] for o in objs)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
