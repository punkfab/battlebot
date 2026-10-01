"""parts.py — the other printed parts: lid, wheel hub, tyre, lifter arm.

Each part: `<name>(p)` -> build123d solid in its LOCAL frame, plus a MATES dict from `<name>_mates(p)`.
The chassis lives in chassis.py (it is the datum part).
"""
from __future__ import annotations

import math

from build123d import (Align, Box, Cylinder, Plane, Polygon, Pos, Rot, extrude)

from bom import CATALOG
import layout as L
from params import P, DEFAULT

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
EPS = 0.5


def _ycyl(r, y0, y1, x=0.0, z=0.0):
    lo, hi = min(y0, y1), max(y0, y1)
    return Pos(x, (lo + hi) / 2, z) * (Rot(90, 0, 0) * Cylinder(r, hi - lo))


# ----------------------------------------------------------------------------------- lid
def lid(p: P = DEFAULT):
    """Local frame: centre of the lid UNDERSIDE (seats on the rim). Clamps board + servo."""
    s = CATALOG[p.servo]
    body = Box(p.ch_len, p.ch_w, p.lid_t, align=MIN)
    down = p.ch_h                                            # chassis z of the underside
    # board clamp bosses: press the board onto its posts
    for x, y in L.board_post_xy(p):
        h = down - (p.board_z + p.board_t + 0.1)
        body = body + Pos(x, y, -h) * Cylinder(3.0, h + EPS, align=MIN)
    # servo clamp rib: presses the case onto the floor
    sl = L.servo_loc(p)
    cx = sl.position.X + s.spline_off - s.body_l / 2
    cy = sl.position.Y - s.body_h / 2
    h = down - (p.floor + s.body_w + 0.2)
    body = body + Pos(cx, cy, -h) * Box(s.body_l - 4, 8.0, h + EPS, align=MIN)
    # screw clearance at the corner posts
    for x, y in L.corner_post_xy(p):
        body = body - Pos(x, y, -EPS) * Cylinder(p.screw_clear_d / 2, p.lid_t + 2 * EPS, align=MIN)
    return body


def lid_mates(p: P = DEFAULT):
    return {"seat": Pos(0, 0, 0)}


# --------------------------------------------------------------------------------- wheel
def hub(p: P = DEFAULT):
    """Local frame: wheel centre, axle along +y (+y = inboard face, where the shaft enters)."""
    m = CATALOG[p.motor]
    h = _ycyl(p.hub_d / 2, -p.wheel_w / 2, p.wheel_w / 2)
    # lightening pockets (through), 6 around
    for k in range(6):
        a = 2 * math.pi * k / 6
        r = (p.hub_d / 2 + 5.0) / 2
        h = h - _ycyl(3.0, -p.wheel_w, p.wheel_w, r * math.cos(a), r * math.sin(a))
    # D-bore from the inboard face, depth = shaft engagement
    engage = inboard_engagement(p)
    bore = _ycyl(m.shaft_d / 2 + 0.05, p.wheel_w / 2 - engage, p.wheel_w / 2 + EPS)
    flat = Pos(0, p.wheel_w / 2 - engage / 2 + EPS / 2, m.shaft_d / 2 - 0.25 + 2) * Box(4, engage + EPS, 4)
    h = h - (bore - flat)
    return h


def inboard_engagement(p: P):
    m = CATALOG[p.motor]
    shaft_tip = p.inner_y + m.shaft_len                      # |y| of the shaft tip
    return shaft_tip - (p.wheel_y - p.wheel_w / 2)


def tyre(p: P = DEFAULT, as_printed: bool = False):
    """TPU tyre, stretched over the hub. Local frame = wheel centre, axle along y.
    The assembly uses the INSTALLED shape (bore = hub OD); the print file is `as_printed`,
    bore undersized by tyre_stretch — the interference IS the retention."""
    bore = p.hub_d / 2 - (p.tyre_stretch if as_printed else 0.0)
    return _ycyl(p.wheel_d / 2, -p.wheel_w / 2, p.wheel_w / 2) - _ycyl(bore, -p.wheel_w, p.wheel_w)


def wheel_mates(p: P = DEFAULT):
    return {"centre": Pos(0, 0, 0)}


# ----------------------------------------------------------------------------------- arm
def arm_pieces(p: P = DEFAULT):
    """The fork's CONVEX pieces (arm-local frame): {name: solid}. arm() is their union minus the
    notches, and the sim collides these same solids — one construction, no re-described geometry.
    (A U-fork's convex hull would fill the gap between the plates; its pieces don't.)"""
    beta = math.radians(-p.arm_rest_deg)                     # rest: tip-down angle
    Lr, wl, t = p.arm_len, p.wedge_len, p.arm_plate_t
    y_in, y_out = p.arm_y, p.arm_y + t
    ph = p.arm_plate_h
    plen, z0 = p.plate_span
    out = {}
    for side, sgn in (("L", 1), ("R", -1)):
        out[f"plate_{side}"] = Pos(plen / 2, sgn * (y_in + t / 2), z0 + ph / 2) * Box(plen, t, ph)
        out[f"disc_{side}"] = _ycyl(ph / 2 + 1.0, sgn * y_in, sgn * y_out)
    # wedge crossbar: profile in the arm's xz-plane, extruded across the full fork width
    pts = [(Lr, 0.0),
           (Lr - wl * math.cos(beta), -wl * math.sin(beta)),
           (Lr - wl, p.wedge_h),
           (Lr - 0.8, 0.9)]
    out["wedge"] = extrude(Plane.XZ * Polygon(*pts, align=None), amount=y_out, both=True)
    return out


def arm(p: P = DEFAULT):
    """Lifter U-fork. Local frame: origin ON the pivot axis at the bot's centre plane (y=0),
    axis along y, arm pointing +x at 0 deg (lift = Rot(0, -deg, 0)). Wedge ground-face is cut so
    it lies flat 0.5 mm above the ground at the rest angle."""
    t, ph, y_in, y_out = p.arm_plate_t, p.arm_plate_h, p.arm_y, p.arm_y + p.arm_plate_t
    pieces = list(arm_pieces(p).values())
    body = pieces[0]
    for q in pieces[1:]:
        body = body + q
    # downward-open key notches: the fork drops (along its own -z) onto the two keyed hubs
    for sgn in (1, -1):
        yc = sgn * (y_in + t / 2)
        notch = Pos(0, yc, -(ph / 2 + 2) / 2) * Box(p.key_w + 0.2, t + 2 * EPS, ph / 2 + 2)
        body = body - notch - _ycyl(p.hub_r + 0.1, sgn * (y_in - EPS), sgn * (y_out + EPS))
    return body


# Print orientation per part (part-local -> bed), a DFM decision that lives with the part.
# (rotation, note). The slicer file is generated from these; supports flagged honestly.
PRINT = {
    "chassis": (Rot(0, 0, 0), "open top up; floor on the bed"),
    "lid":     (Rot(180, 0, 0), "top face on the bed; bosses/rib point up"),
    "hub":     (Rot(90, 0, 0), "axle vertical; D-bore prints as a clean vertical hole"),
    "tyre":    (Rot(90, 0, 0), "axle vertical; TPU"),
    "arm":     (Rot(0, 0, 0), "fork flat, plates on edge (16 mm tall); small support under the pivot discs"),
    "hubkey":  (Rot(90, 0, 0), "axis vertical; key flats print as walls"),
}


def print_key(item_name: str) -> str:
    return next(k for k in PRINT if item_name == k or item_name.startswith(k + "_") or item_name.startswith(k))


def collision_pieces(p: P = DEFAULT):
    """Convex collision proxies for parts whose convex hull is wrong, in PART-local frames.
    Only concave parts that meet the outside world need them; everything else collides as its hull."""
    pc = arm_pieces(p)
    return {"arm": [pc["plate_L"], pc["plate_R"], pc["wedge"]]}


def hub_key(p: P = DEFAULT, driven: bool = True):
    """Keyed pivot hub, one per side, in the ARM's local frame (so it takes the arm's pivot
    Location). Bearing section rides in the chassis wall slot; the key (flats |x| <= key_w/2)
    carries the fork. driven=True: +y side, bored for the servo spline (pressed on, horn-screw
    retained). driven=False: -y side, bored for the M2 axle screw into the pivot-boss insert."""
    sgn = 1 if driven else -1
    y0, y1, y2 = p.inner_y + 0.2, p.arm_y, p.arm_y + p.arm_plate_t
    bearing = _ycyl(p.hub_r, sgn * y0, sgn * y1)
    key = _ycyl(p.hub_r, sgn * y1, sgn * y2) - (
        Pos(0, sgn * (y1 + y2) / 2, 0) * (Box(20, 10, 20) - Box(p.key_w, 10, 20)))
    body = bearing + key
    bore_r = 2.6 if driven else p.screw_clear_d / 2
    return body - _ycyl(bore_r, sgn * (y0 - EPS), sgn * (y2 + EPS))


def arm_mates(p: P = DEFAULT):
    return {"pivot": Pos(0, 0, 0), "tip": Pos(p.arm_len, 0, 0)}


if __name__ == "__main__":
    import os
    os.makedirs("build", exist_ok=True)
    from build123d import export_stl
    for name, fn in (("lid", lid), ("hub", hub), ("tyre", tyre), ("arm", arm), ("hub_key", hub_key)):
        s = fn()
        export_stl(s, f"build/{name}.stl")
        print(f"{name:5s} vol {s.volume:8.0f} mm^3  solids {len(s.solids())}  bbox {s.bounding_box().size}")
    print("shaft engagement into hub:", round(inboard_engagement(DEFAULT), 2), "mm")
