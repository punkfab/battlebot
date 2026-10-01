"""bot.py — the whole antweight, placed BY MATES in the BOT frame (origin on the ground).

`items(p, lift_deg)` is the one assembly definition everything else consumes: the mass gate,
the assembly-sequence gate, the sim (masses + CoM) and the render. Each item carries its solid,
where its mass comes from, and its render colour.

    python bot.py            -> build/bot_*.stl + build/bot_iso.png + manifest + mass table
"""
from __future__ import annotations

import os
import subprocess
from dataclasses import dataclass

from build123d import Location, Pos, export_stl

import bom
import chassis
import parts
import layout as L
from params import P, DEFAULT, place

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "..", "build")


@dataclass
class Item:
    name: str
    local: object          # build123d solid in the PART's own frame (mm) — None for lumped mass
    loc: object            # build123d Location: part frame -> BOT frame (the occurrence transform)
    mass_g: float
    kind: str              # "printed" | "bought" | "lumped"
    group: str             # rigid group (fastened together): "chassis" | "arm" | "wheel_L" | "wheel_R"
    color: str
    source: str = ""
    material: str = ""     # print material for printed parts ("PETG" | "TPU")

    @property
    def solid(self):       # placed in the BOT frame
        return None if self.local is None else self.loc * self.local


def items(p: P = DEFAULT, lift_deg: float | None = None):
    errs = p.validate()
    if errs:
        raise ValueError("invalid params: " + "; ".join(errs))
    base = Pos(0, 0, p.ground_clear)                        # CHASSIS frame -> BOT frame
    cm = chassis.mates(p)
    out = []

    def printed(name, local, loc, rho, group, color):
        mat = "TPU" if rho == p.rho_tpu else "PETG"
        out.append(Item(name, local, loc, local.volume * rho * p.fill, "printed", group, color,
                        "CAD volume x rho x fill", mat))

    def bought(name, local, loc, mass, color, source, group="chassis"):
        out.append(Item(name, local, loc, mass, "bought", group, color, source))

    def onto(frm, target):                                  # the placement place() would apply
        return target * frm.inverse()

    printed("chassis", chassis.part(p), base, p.rho_petg, "chassis", "#3b3f46")
    printed("lid", parts.lid(p), onto(parts.lid_mates(p)["seat"], base * cm["lid"]), p.rho_petg, "chassis", "#ff5a1f")
    m = bom.CATALOG[p.motor]
    for key in ("motor_L", "motor_R"):
        bought(key, bom.motor_envelope(m), base * cm[key], m.mass_g, "#c0c4cc", m.source)
    s = bom.CATALOG[p.servo]
    bought("servo", bom.servo_envelope(s), base * cm["servo"], s.mass_g, "#2a6fdb", s.source)
    b = bom.CATALOG[p.battery]
    bought("battery", bom.battery_envelope(b), base * cm["battery"], b.mass_g, "#e8c547", b.source)
    bought("board", bom.board_envelope(p), base * cm["board"], bom.board_mass_g(p), "#1f8a4c",
           "custom carrier (FR4 est. + parts est.)")

    for key in ("wheel_L", "wheel_R"):
        w = onto(parts.wheel_mates(p)["centre"], base * cm[key])
        printed(f"hub_{key[-1]}", parts.hub(p), w, p.rho_petg, key, "#ff5a1f")
        printed(f"tyre_{key[-1]}", parts.tyre(p), w, p.rho_tpu, key, "#18181b")

    piv = onto(parts.arm_mates(p)["pivot"], base * L.pivot_loc(p, lift_deg))
    printed("arm", parts.arm(p), piv, p.rho_petg, "arm", "#ff5a1f")
    printed("hubkey_L", parts.hub_key(p, True), piv, p.rho_petg, "arm", "#8a8f98")
    printed("hubkey_R", parts.hub_key(p, False), piv, p.rho_petg, "arm", "#8a8f98")

    # lumped: one fastener family. lid 4 + pivot 1 + horn 1 screws; 4 corner + 1 pivot inserts
    n_screw, n_insert = 6, 5
    out.append(Item("fasteners", None, None, n_screw * bom.M2_SCREW_G + n_insert * bom.M2_INSERT_G, "lumped",
                    "chassis", "", f"{n_screw}x M2 screw + {n_insert}x M2 heat-set insert"))
    out.append(Item("wiring", None, None, bom.WIRING_G, "lumped", "chassis", "", "leads estimate"))
    return out


def mass_table(its):
    tot = sum(i.mass_g for i in its)
    rows = [f"  {i.name:10s} {i.mass_g:7.1f} g  {i.kind:8s} {i.source}" for i in its]
    return tot, "\n".join(rows)


def com_mm(its, group=None):
    """Mass-weighted centre (mm, BOT frame). Lumped items sit at the chassis CoM."""
    import numpy as np
    num, den = np.zeros(3), 0.0
    for i in its:
        if i.solid is None or (group and i.group != group):
            continue
        c = i.solid.center()
        num += i.mass_g * np.array([c.X, c.Y, c.Z])
        den += i.mass_g
    return num / den


def export(its, prefix="bot"):
    os.makedirs(BUILD, exist_ok=True)
    paths = {}
    for i in its:
        if i.solid is None:
            continue
        path = os.path.join(BUILD, f"{prefix}_{i.name}.stl")
        tmp = f"{path}.{os.getpid()}.tmp"                   # atomic: parallel sweeps share build/
        export_stl(i.solid, tmp)
        os.replace(tmp, path)
        paths[i.name] = path
    return paths


def render(its, paths, out_png, camera="140,-150,110,0,0,15", size="1200,900"):
    """Headless OpenSCAD preview: one coloured import() per part."""
    scad = os.path.join(BUILD, os.path.basename(out_png).replace(".png", ".scad"))
    with open(scad, "w") as f:
        for i in its:
            if i.name in paths:
                f.write(f'color("{i.color}") import("{os.path.abspath(paths[i.name])}");\n')
    subprocess.run(["openscad", "-o", out_png, f"--imgsize={size}", f"--camera={camera}",
                    "--viewall", "--autocenter", "--colorscheme=Tomorrow", scad],
                   check=True, capture_output=True)
    return out_png


if __name__ == "__main__":
    p = DEFAULT
    its = items(p)
    tot, table = mass_table(its)
    print("manifest / mass:")
    print(table)
    print(f"  {'TOTAL':10s} {tot:7.1f} g   (target {p.mass_target_g:.0f}, class limit {p.mass_limit_g:.0f})")
    print("CoM (mm, bot frame):", [round(v, 1) for v in com_mm(its)])
    paths = export(its)
    print("render:", render(its, paths, os.path.join(BUILD, "bot_iso.png")))
    up = items(p, lift_deg=60)
    paths_up = export(up, prefix="bot_up")
    print("render:", render(up, paths_up, os.path.join(BUILD, "bot_lift60.png"), camera="0,-250,40,0,0,20"))
