"""bom.py — bought-in components as ENVELOPE models (keep-out solid + mass + mates).

Every number here is NOMINAL — vendor/listing-claimed, not bench-measured — and carries
`verified=False` until weighed/measured. These entries are staged for `robot-bom/antweight/`
(primary-sourced, the robot-bom convention); this file is the working copy until they land.

Each component: a catalog record + `envelope(p)` returning a build123d solid in the part's
LOCAL frame + `mates(p)` (named Locations in that frame, +Z of a mate = joining direction).
"""
from __future__ import annotations

from dataclasses import dataclass, field

from build123d import Box, Cylinder, Location, Pos, Rot, Align

KGCM = 0.0980665  # kg·cm -> N·m


@dataclass(frozen=True)
class Servo:
    key: str
    mass_g: float
    body_l: float        # along the case, spline end ... far end
    body_w: float        # thickness (installed: vertical)
    body_h: float        # case bottom -> top face (installed: along +y, spline side outward)
    ear_span: float      # mounting-ear tip-to-tip along body_l
    ear_t: float
    ear_at: float        # ear plane distance from case bottom
    spline_off: float    # spline axis distance from the spline-end of the case
    spline_len: float    # spline + horn hub protrusion above top face
    stall_Nm: float      # at the supply voltage below
    volts: float
    s_per_60: float      # no-load speed
    source: str = ""
    verified: bool = False


@dataclass(frozen=True)
class Motor:
    key: str
    mass_g: float
    w: float             # gearbox width (installed: x)
    h: float             # gearbox height (installed: z)
    body_len: float      # gearbox face -> end of can incl. terminals (installed: along y)
    shaft_d: float
    shaft_len: float
    rpm_noload: float    # at `volts`
    stall_Nm: float
    volts: float
    source: str = ""
    verified: bool = False


@dataclass(frozen=True)
class Battery:
    key: str
    mass_g: float
    size: tuple          # installed (x, y, z)
    volts: float
    mAh: float
    source: str = ""
    verified: bool = False


CATALOG = {
    # --- lifter servos (micro, metal gear). Torques are vendor-claimed stall at 6 V via buck.
    "MG90S": Servo("MG90S", 13.4, 22.8, 12.2, 28.5, 32.5, 2.5, 18.5, 6.0, 4.0,
                   2.2 * KGCM, 6.0, 0.10, "TowerPro listing"),
    "MG92B": Servo("MG92B", 13.8, 22.8, 12.0, 31.0, 32.5, 2.5, 20.5, 6.0, 4.0,
                   3.5 * KGCM, 6.0, 0.13, "TowerPro listing"),
    "ES08MA2": Servo("ES08MA2", 12.0, 23.0, 11.5, 24.0, 32.5, 2.5, 16.0, 6.0, 4.0,
                     2.0 * KGCM, 6.0, 0.10, "EMAX listing"),
    # --- drive: N20 gearmotors run direct from 2S (7.4 V). Speed/stall scaled from rated V.
    "N20_300": Motor("N20_300", 10.0, 12.0, 10.0, 26.0, 3.0, 10.0,
                     370.0, 0.06, 7.4, "generic 6V 300rpm N20, scaled to 7.4V"),
    "N20_1000": Motor("N20_1000", 10.0, 12.0, 10.0, 26.0, 3.0, 10.0,
                      620.0, 0.025, 7.4, "generic 12V 1000rpm N20, scaled to 7.4V"),
    # --- power
    "LIPO_2S_300": Battery("LIPO_2S_300", 19.0, (30.0, 50.0, 11.0), 7.4, 300,
                           "typical 2S 300mAh pack incl. short leads"),
}

# Lumped small stuff, grams. Fasteners are ONE family (DFA): M2 screw + M2 heat-set insert.
M2_SCREW_G = 0.25
M2_INSERT_G = 0.30
FR4_RHO = 1.85e-3            # g/mm^3
BOARD_PARTS_G = 6.0          # ESP32-C3 module, DRV8833, buck, connectors (estimate)
WIRING_G = 6.0               # motor + servo + battery leads (estimate; goal: zero loose wires)


# ------------------------------------------------------------------------------ envelopes
def servo_envelope(s: Servo):
    """Local frame: origin ON the spline axis at the case top face; spline along +Y
    (installed pointing out through the +y wall), case long axis along +X with the spline end
    at +X, case thickness along Z (centred)."""
    x0 = s.spline_off - s.body_l            # case far end
    body = Pos(x0 + s.body_l / 2, -s.body_h / 2, 0) * Box(s.body_l, s.body_h, s.body_w)
    ears = Pos(x0 + s.body_l / 2, -(s.body_h - s.ear_at), 0) * Box(s.ear_span, s.ear_t, s.body_w)
    spline = Pos(0, s.spline_len / 2, 0) * (Rot(90, 0, 0) * Cylinder(2.5, s.spline_len))
    return body + ears + spline


def servo_mates(s: Servo):
    return {
        "spline": Location((0, s.spline_len, 0)),                 # arm hub seats here
        "bottom": Pos(s.spline_off - s.body_l / 2, -s.body_h / 2, -s.body_w / 2),  # rests on floor
    }


def motor_envelope(m: Motor):
    """Local frame: origin on the shaft axis at the gearbox face; shaft along +Y, body along -Y."""
    body = Pos(0, -m.body_len / 2, 0) * Box(m.w, m.body_len, m.h)
    shaft = Pos(0, m.shaft_len / 2, 0) * (Rot(90, 0, 0) * Cylinder(m.shaft_d / 2, m.shaft_len))
    flat = Pos(0, m.shaft_len / 2 + 1, m.shaft_d / 2 - 0.25 + 2) * Box(4, m.shaft_len, 4)   # D-flat
    return body + (shaft - flat)


def battery_envelope(b: Battery):
    """Local frame: centre of the bottom face."""
    return Box(*b.size, align=(Align.CENTER, Align.CENTER, Align.MIN))


def board_envelope(p):
    """Carrier board + top-side component keep-out. Local frame: centre of the board underside."""
    pcb = Box(p.board_len, p.board_w, p.board_t, align=(Align.CENTER, Align.CENTER, Align.MIN))
    comps = Pos(0, 0, p.board_t) * Box(p.board_len - 16, p.board_w - 20, p.board_comp_h,
                                        align=(Align.CENTER, Align.CENTER, Align.MIN))
    board = pcb + comps
    cx = p.board_x0 + p.board_len / 2                    # locating-pin holes at the post positions
    from layout import board_post_xy
    for x, y in board_post_xy(p):
        board = board - Pos(x - cx, y, -1) * Cylinder(p.board_pin_d / 2 + 0.1, p.board_t + 2,
                                                       align=(Align.CENTER, Align.CENTER, Align.MIN))
    return board


def board_mass_g(p):
    return p.board_len * p.board_w * p.board_t * FR4_RHO + BOARD_PARTS_G
