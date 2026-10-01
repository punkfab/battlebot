"""chassis.py — the printed tub. Every locating feature is grown FROM layout.py datums.

Design-for-assembly intent: every internal component drops in TOP-DOWN into a feature that
locates it (motor cradles, servo ear slots, board posts with pins, battery fence); the lid then
clamps the stack. The only fasteners are M2 screws into M2 heat-set inserts.
Local frame = CHASSIS frame (origin at the floor underside centre).
"""
from __future__ import annotations

from build123d import Align, Box, Cylinder, Pos, Rot, export_stl

from bom import CATALOG
import layout as L
from params import P, DEFAULT

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
EPS = 0.5  # boolean overshoot


def _ycyl(r, y0, y1, x, z):
    """Cylinder along y spanning [y0, y1] at (x, z)."""
    lo, hi = min(y0, y1), max(y0, y1)
    return Pos(x, (lo + hi) / 2, z) * (Rot(90, 0, 0) * Cylinder(r, hi - lo))


def _rim_slot(p, r, up):
    """Keyhole slot through a side wall, open to the rim. Local frame: axis point on the INNER
    wall face, +y outward; the slot rises `up` mm (to the rim) above the axis."""
    t = p.wall + 2 * EPS
    y = p.wall / 2
    return (Pos(0, y, 0) * (Rot(90, 0, 0) * Cylinder(r, t))) + Pos(0, y, (up + EPS) / 2) * Box(2 * r, t, up + EPS)


def part(p: P = DEFAULT):
    m, s, b = CATALOG[p.motor], CATALOG[p.servo], CATALOG[p.battery]

    adds = []   # locating features, unioned onto the hollowed shell

    # --- corner posts (lid screws), full height
    for x, y in L.corner_post_xy(p):
        adds.append(Pos(x, y, 0) * Cylinder(p.post_d / 2, p.ch_h, align=MIN))

    # --- motor cradles: two short U-saddles per motor (at the gearbox and at the can end)
    for loc in L.motor_locs(p).values():
        for dy in (-(4.0), -(m.body_len - 4.0)):           # local -y = into the chassis
            saddle = Box(m.w + 4, 6.0, p.axle_z - p.floor, align=MIN)
            saddle = loc * Pos(0, dy, -(p.axle_z - p.floor)) * saddle
            adds.append(saddle)

    # --- servo ear slots: a block beyond each case end, slotted to capture the ear in y
    sl = L.servo_loc(p)
    x_far = s.spline_off - s.body_l                         # case far end, servo-local x
    ear_y = -(s.body_h - s.ear_at)                          # ear plane, servo-local y
    ear_ext = (s.ear_span - s.body_l) / 2
    for xc in (x_far - ear_ext / 2 - 1.0, s.spline_off + ear_ext / 2 + 1.0):
        blk = Pos(xc, ear_y, 0) * Box(ear_ext + 2.0, s.ear_t + 6.0, s.body_w)
        adds.append(sl * blk)

    # --- -y passive pivot boss (holds an M2 insert coaxial with the arm hub)
    adds.append(_ycyl(p.hub_r + 2.5, -p.inner_y - EPS, -p.inner_y + 7.0, p.pivot_x, p.pivot_z))  # wider than the slot: bonds to the wall around it

    # --- board posts
    for x, y in L.board_post_xy(p):
        adds.append(Pos(x, y, 0) * Cylinder(3.0, p.board_z, align=MIN))
        adds.append(Pos(x, y, p.board_z - EPS) * Cylinder(p.board_pin_d / 2 - 0.1, p.board_t + EPS, align=MIN))   # flush with board top

    # --- battery fence: low corner tabs around the pack footprint
    bl = L.battery_loc(p)
    bx, by, _ = b.size
    for sx in (1, -1):
        for sy in (1, -1):
            tab = Pos(sx * (bx / 2 + 0.5 + 0.75), sy * (by / 2 - 4), 0) * Box(1.5, 8, 5, align=MIN)
            adds.append(bl * tab)

    # --- shell: hollow a plain box FIRST, then union the features (cutting the cavity after
    # would eat them)
    shell = Box(p.ch_len, p.ch_w, p.ch_h, align=MIN) - (
        Pos(0, 0, p.floor) * Box(2 * p.inner_x, 2 * p.inner_y, p.ch_h, align=MIN))
    body = shell
    for a in adds:
        body = body + a

    cuts = []
    # motor envelopes (so saddles become U-channels) + axle holes through the walls
    for loc in L.motor_locs(p).values():
        cuts.append(loc * Pos(0, -m.body_len / 2, 0) * Box(m.w + 0.4, m.body_len + 0.4, m.h + 0.4))
        cuts.append(loc * _rim_slot(p, 2.25, p.ch_h - p.axle_z))   # rim-open: motor drops in top-down
    # servo: case + ear slot clearance
    cuts.append(sl * Pos(x_far + s.body_l / 2, -s.body_h / 2, 0) * Box(s.body_l + 0.4, s.body_h, s.body_w + 2 * EPS))
    cuts.append(sl * Pos(x_far + s.body_l / 2, ear_y, 0) * Box(s.ear_span + 0.6, s.ear_t + 0.4, s.body_w + 2 * EPS))
    # arm hub bearings: rim-open U-slots through both side walls (servo spline + hubs drop in)
    for sgn in (1, -1):
        loc = Pos(p.pivot_x, sgn * p.inner_y, p.pivot_z) * (Rot(0, 0, 0) if sgn > 0 else Rot(0, 0, 180))
        cuts.append(loc * _rim_slot(p, p.hub_r + 0.2, p.ch_h - p.pivot_z))
    # M2 insert pilot in the -y pivot boss
    cuts.append(_ycyl(p.insert_hole_d / 2, -p.inner_y - EPS, -p.inner_y + p.insert_depth + 1, p.pivot_x, p.pivot_z))
    # heat-set insert pilots in the corner posts (from the rim down)
    for x, y in L.corner_post_xy(p):
        cuts.append(Pos(x, y, p.ch_h - p.insert_depth - 1) * Cylinder(p.insert_hole_d / 2, p.insert_depth + 1 + EPS, align=MIN))

    for c in cuts:
        body = body - c
    return body


# Mates, chassis frame. +Z of a mate points along the joining direction.
def mates(p: P = DEFAULT):
    return {
        "ground": Pos(0, 0, -p.ground_clear),
        "lid": L.lid_loc(p),
        **L.motor_locs(p),
        "servo": L.servo_loc(p),
        "battery": L.battery_loc(p),
        "board": L.board_loc(p),
        **L.wheel_locs(p),
        "pivot": L.pivot_loc(p),
    }


if __name__ == "__main__":
    import os
    os.makedirs("build", exist_ok=True)
    c = part()
    export_stl(c, "build/chassis.stl")
    print(f"chassis: vol {c.volume:.0f} mm^3, solids {len(c.solids())}, bbox {c.bounding_box().size}")
