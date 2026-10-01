"""params.py — SINGLE SOURCE OF TRUTH for the battlebot.

Every dimension any part, the assembly, the recipe or the sim shares lives here as a field of
`P` (a frozen dataclass, so a design variant is just `P(arm_len=60)` — sweeps build variants,
never edit constants). Derived dimensions are properties, so geometry and mates can never drift.

Frames (mm):
  BOT frame   : origin on the GROUND under the chassis centre, +x forward, +y left, +z up.
  CHASSIS frame: origin at the centre of the floor's underside. BOT = Pos(0,0,ground_clear)*CHASSIS.
  Every part is authored in its own local frame and placed BY MATES (see place()).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace

from build123d import Location, Pos, Rot


def place(solid, frm: Location, onto: Location):
    """Snap a part so its local mate `frm` coincides with the world Location `onto`."""
    return (onto * frm.inverse()) * solid


@dataclass(frozen=True)
class P:
    # --- class / budget ------------------------------------------------------------------
    mass_limit_g: float = 454.0        # antweight: 1 lb
    mass_target_g: float = 400.0       # "low end of antweight" — gate threshold, keep headroom

    # --- chassis tub (printed) -------------------------------------------------------------
    ch_len: float = 112.0              # outer, x
    ch_w: float = 84.0                 # outer, y (wall-to-wall; wheels + arm plates outside)
    ch_h: float = 30.0                 # floor underside -> rim
    wall: float = 2.0
    floor: float = 1.6
    ground_clear: float = 3.0          # floor underside above ground
    post_d: float = 7.0                # corner posts (lid screws into heat-set inserts)

    # --- one fastener family (DFA): M2 screw + M2 heat-set insert -------------------------
    insert_hole_d: float = 3.2         # M2 heat-set insert pilot
    insert_depth: float = 4.0
    screw_clear_d: float = 2.4

    # --- lid ---------------------------------------------------------------------------------
    lid_t: float = 1.6

    # --- drive: 2WD skid steer, N20 gearmotors, wheels OUTSIDE the walls --------------------
    wheel_d: float = 38.0              # tyre OD
    hub_d: float = 30.0                # printed hub OD (TPU tyre stretches over it)
    wheel_w: float = 10.0
    wheel_gap: float = 1.5             # tyre inner face <-> chassis outer wall
    wheel_x: float = -12.0             # axle x (behind centre: the lifter lives up front)

    # --- lifter: servo-driven U-fork with a full-width wedge crossbar ------------------------
    pivot_x: float = 30.0              # arm pivot (servo spline axis) x, chassis frame
    arm_len: float = 52.0              # pivot -> wedge tip
    arm_plate_t: float = 3.0
    arm_plate_h: float = 12.0
    arm_gap: float = 0.5               # plate inner face <-> chassis outer wall
    wedge_len: float = 14.0            # crossbar depth along the arm
    wedge_h: float = 7.5               # crossbar rise at its back edge
    arm_min_clear: float = 1.5         # crossbar must clear the chassis front-top corner by this
    arm_max_deg: float = 100.0         # mechanical up-stop
    hub_r: float = 4.5                 # arm hub bearing radius (rides in the rim-open wall slot)
    key_w: float = 7.0                 # hub key across-flats (arm plate notch drops over it)

    # --- print material -----------------------------------------------------------------------
    rho_petg: float = 1.27e-3          # g/mm^3
    rho_tpu: float = 1.21e-3
    tyre_stretch: float = 0.4          # tyre bore is printed this much under the hub radius
    fill: float = 0.85                 # printed/solid mass ratio (walls ~solid; CALIBRATE by weighing)

    # --- electronics carrier (custom board: ESP32 + dual H-bridge + buck + board-mount connectors)
    board_len: float = 56.0            # x (front posts must clear the servo ears on the way down)
    board_w: float = 66.0              # y (clears the corner posts)
    board_t: float = 1.0
    board_comp_h: float = 5.0          # tallest part on the top side (ESP32-C3 module + USB-C)
    board_x0: float = -50.0            # rear edge, chassis frame
    board_pin_d: float = 2.0           # locating pins (board drops onto them, lid clamps it)

    # --- component selection (keys into bom.CATALOG) --------------------------------------------
    servo: str = "MG92B"
    motor: str = "N20_1000"
    battery: str = "LIPO_2S_300"

    def with_(self, **kw) -> "P":
        return replace(self, **kw)

    # ------------------------------------------------------------------------ derived
    @property
    def inner_x(self):            # half inner length
        return self.ch_len / 2 - self.wall

    @property
    def inner_y(self):            # half inner width
        return self.ch_w / 2 - self.wall

    @property
    def axle_z(self):             # axle height, chassis frame
        return self.wheel_d / 2 - self.ground_clear

    @property
    def wheel_y(self):            # wheel centre-plane |y|
        return self.ch_w / 2 + self.wheel_gap + self.wheel_w / 2

    @property
    def board_z(self):            # board underside, chassis frame (sits just above the motors)
        from bom import CATALOG
        m = CATALOG[self.motor]
        return self.axle_z + m.h / 2 + 0.3

    @property
    def top_z(self):              # chassis top (lid top), chassis frame
        return self.ch_h + self.lid_t

    @property
    def arm_y(self):              # arm plate inner face |y|
        return self.ch_w / 2 + self.arm_gap

    @property
    def pivot_z(self):            # servo spline axis height, chassis frame
        from bom import CATALOG
        s = CATALOG[self.servo]
        return self.floor + s.body_w / 2

    @property
    def arm_rest_deg(self):       # arm angle (+ = tip up) with the wedge tip on the ground
        z_ground = self.pivot_z + self.ground_clear
        return -math.degrees(math.asin(min(1.0, (z_ground - 0.5) / self.arm_len)))

    @property
    def plate_span(self):         # arm plate, arm-local: x in [0, len], z in [z0, z0 + arm_plate_h]
        """Plates run pivot -> wedge back and sit ABOVE the pivot-tip line (which, at rest, runs
        down to the floor) — so only the wedge ever touches the ground."""
        return self.arm_len - self.wedge_len + 2.0, -1.5

    @property
    def front_corner_r(self):     # chassis front-top corner distance from the pivot (arm sweep check)
        return math.hypot(self.ch_len / 2 - self.pivot_x, self.top_z - self.pivot_z)

    def validate(self):
        """Cheap geometric invariants — fail before building anything."""
        errs = []
        inner_r = self.arm_len - self.wedge_len
        if inner_r < self.front_corner_r + self.arm_min_clear:
            errs.append(f"wedge crossbar inner radius {inner_r:.1f} < front corner "
                        f"{self.front_corner_r:.1f}+{self.arm_min_clear} — arm can't swing past the nose")
        if self.pivot_x - self.arm_plate_h < self.wheel_x + self.wheel_d / 2:
            errs.append("arm plate root overlaps the wheel")
        if self.top_z + self.ground_clear >= self.wheel_d:
            errs.append("chassis taller than wheels: not invertible")
        return errs


DEFAULT = P()
