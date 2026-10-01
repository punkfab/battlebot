"""recipe.py — the battlebot ASSEMBLY RECIPE: an ordered op list routed across BUILD CELLS.

A product is not a cell; it is a recipe. Each step names:
  * the build-cell OPERATION TYPE it needs (the vocabulary ../buildcell will implement),
  * the parts it moves, and the APPROACH vector they travel along (BOT frame) — so the
    assembly gate can sweep them in against everything already installed,
  * the fixture/orientation, and a nominal duration for the scheduler.

Design-for-assembly targets (checked by scripts/check_assembly.py): minimise distinct op
types, minimise non-top-down approaches (each one is a reorientation or a side-acting tool),
zero loose-wire steps (today: one, flagged MANUAL — the open DFA problem).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import layout as L
from params import P

# Build-cell operation types — the ../buildcell vocabulary. One cell type per op class.
OPS = {
    "place":           "pick-and-place drop-in onto locating features",
    "press_fit":       "linear press to a stop (hub onto spline/shaft, tyre onto hub)",
    "heat_set_insert": "heated-tip insert press",
    "drive_screw":     "M2 screwdriver (one fastener family)",
    "manual":          "NOT YET ROBOTIC — a human step, tracked as DFA debt",
}

DOWN, PLUS_Y, MINUS_Y = (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (0.0, -1.0, 0.0)


@dataclass(frozen=True)
class Step:
    name: str
    op: str
    parts: tuple = ()             # item names moved by this step (sweep-checked)
    approach: tuple = DOWN        # unit direction of travel, BOT frame
    count: int = 1                # repetitions (e.g. 4 screws)
    needs: tuple = ()
    fixture: str = "bot_upright"
    against: tuple | None = None  # sub-assembly steps: sweep only against these parts
    seconds: float = 5.0          # nominal, per repetition
    note: str = ""


def arm_drop_dir(p: P):
    """The fork drops along its OWN -z at the rest angle (notches open that way)."""
    a = math.radians(-p.arm_rest_deg)          # tip-down angle
    # arm frame at rest = Rot(0, -rest_deg, 0) = rotate about +y by +a (tip down)
    # local -z  -> world (-sin a, 0, -cos a)
    return (-math.sin(a), 0.0, -math.cos(a))


def recipe(p: P):
    return [
        Step("chassis", "place", parts=("chassis",), fixture="fixture_load", against=(),
             note="load the printed tub into the assembly fixture (datum part)"),
        Step("inserts_posts", "heat_set_insert", count=4, seconds=8, note="corner posts, from the rim"),
        Step("insert_pivot", "heat_set_insert", approach=PLUS_Y, seconds=8,
             note="passive pivot boss, through the -y wall slot"),
        Step("servo_hub", "press_fit", parts=("hubkey_L",), approach=MINUS_Y, fixture="servo_jig",
             against=("servo",), note="sub-assembly: keyed hub onto the servo spline (key aligned to rest angle)"),
        Step("tyres", "press_fit", parts=("tyre_L",), approach=MINUS_Y, fixture="wheel_jig", count=2,
             against=("hub_L",), note="sub-assembly: TPU tyre over printed hub (R identical)"),
        Step("motors", "place", parts=("motor_L", "motor_R"), needs=("inserts_posts", "insert_pivot"), count=2,
             note="drop into U-saddles; shaft passes down the rim-open axle slot"),
        Step("servo", "place", parts=("servo", "hubkey_L"), needs=("servo_hub", "inserts_posts"),
             note="ears drop into the ear slots; hub drops into the +y pivot slot"),
        Step("hub_passive", "place", parts=("hubkey_R",), needs=("insert_pivot",),
             note="drops into the -y pivot slot"),
        Step("battery", "place", parts=("battery",), needs=("inserts_posts",)),
        Step("harness", "manual", needs=("motors", "servo", "battery"), seconds=120,
             note="connect motor/servo/battery leads to the board — LOOSE WIRES: the open DFA problem"),
        Step("board", "place", parts=("board",), needs=("harness",),
             note="drops onto 4 locating pins"),
        Step("lid", "place", parts=("lid",), needs=("board", "hub_passive"),
             note="clamps board + servo, caps the rim slots (retains motors and hubs vertically)"),
        Step("lid_screws", "drive_screw", count=4, needs=("lid",), seconds=6),
        Step("wheel_L", "press_fit", parts=("hub_L", "tyre_L"), approach=MINUS_Y, needs=("motors", "tyres"),
             note="D-bore onto the motor shaft"),
        Step("wheel_R", "press_fit", parts=("hub_R", "tyre_R"), approach=PLUS_Y, needs=("motors", "tyres")),
        Step("arm", "place", parts=("arm",), approach=arm_drop_dir(p), needs=("servo", "hub_passive"),
             note="fork notches drop over the hub keys"),
        Step("arm_screws", "drive_screw", count=2, needs=("arm",), approach=MINUS_Y, seconds=6,
             note="horn screw (-y approach) + passive axle screw (+y approach) retain the fork"),
    ]


def opgraph(p: P, opgraph_mod):
    """Emit the recipe onto software-mfg's OperationGraph: resource = build-cell op type."""
    ops = [opgraph_mod.Operation(s.name, s.op, s.seconds * s.count, needs=s.needs) for s in recipe(p)]
    return opgraph_mod.OperationGraph(ops)
