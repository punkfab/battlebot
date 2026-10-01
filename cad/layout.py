"""layout.py — the datums the LAYOUT owns, in the CHASSIS frame.

Where each bought-in component sits is a layout decision, derived only from `P` + the
catalog. The chassis grows its locating features (cradles, ear slots, posts, holes) FROM these
Locations, and the assembly places components ONTO them — so a feature and the part it locates
cannot drift apart.
"""
from __future__ import annotations

from build123d import Location, Pos, Rot

from bom import CATALOG
from params import P


def motor_locs(p: P):
    """Shaft-axis point on each gearbox face; gearbox face against the inner wall, shaft out."""
    return {
        "motor_L": Pos(p.wheel_x, p.inner_y, p.axle_z),
        "motor_R": Pos(p.wheel_x, -p.inner_y, p.axle_z) * Rot(0, 0, 180),
    }


def servo_loc(p: P):
    """Spline axis on the servo case top face; case top flat against the +y inner wall."""
    return Pos(p.pivot_x, p.inner_y, p.pivot_z)


def battery_loc(p: P):
    b = CATALOG[p.battery]
    return Pos(-p.inner_x + b.size[0] / 2 + 0.5, 0, p.floor)


def board_loc(p: P):
    return Pos(p.board_x0 + p.board_len / 2, 0, p.board_z)


def board_post_xy(p: P):
    xs = (p.board_x0 + 4, p.board_x0 + p.board_len - 4)
    ys = (p.board_w / 2 - 3.5, -(p.board_w / 2 - 3.5))
    return [(x, y) for x in xs for y in ys]


def corner_post_xy(p: P):
    r = p.post_d / 2
    return [(sx * (p.inner_x - r + 1), sy * (p.inner_y - r + 1)) for sx in (1, -1) for sy in (1, -1)]


def wheel_locs(p: P):
    """Wheel centre (hub mid-plane on the axle). Wheel-local +y points INBOARD (bore side)."""
    return {
        "wheel_L": Pos(p.wheel_x, p.wheel_y, p.axle_z) * Rot(0, 0, 180),
        "wheel_R": Pos(p.wheel_x, -p.wheel_y, p.axle_z),
    }


def pivot_loc(p: P, lift_deg: float | None = None):
    """Arm pivot axis (along y). `lift_deg` = tip-up angle; None = resting on the wedge."""
    a = p.arm_rest_deg if lift_deg is None else lift_deg
    return Pos(p.pivot_x, 0, p.pivot_z) * Rot(0, -a, 0)


def lid_loc(p: P):
    return Pos(0, 0, p.ch_h)
