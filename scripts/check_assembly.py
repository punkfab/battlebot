"""check_assembly.py — FIT + ASSEMBLY-SEQUENCE gates (design for assembly).

  1. static fit   : at rest, no two parts solidly overlap (interference < 1 mm^3).
  2. arm swing    : the lifter (fork + hubs) sweeps rest -> up-stop without hitting anything.
  3. recipe sweep : every recipe step moves its parts in along its approach vector, from a
                    standoff, against everything installed before it — first hit fails.
  4. DFA report   : op types used, off-axis approaches, manual (non-robotic) steps, cycle time
                    on software-mfg's op-graph scheduler (one server per build-cell type).

Collision engine: software-mfg/sim/interference.py, composed by reference (spokes.yaml).
"""
import math
import sys

from _common import meshes, gate, np

import bot
import spokes
from params import DEFAULT
from recipe import recipe, opgraph, OPS

STANDOFF_MM = 40.0
N_WAY = 24


def translate(v):
    T = np.eye(4)
    T[:3, 3] = v
    return T


def rot_about_pivot(p, deg):
    """World transform rotating the arm group from its rest pose by `deg` more tip-up."""
    import layout as L
    c = np.array([p.pivot_x, 0.0, p.pivot_z + p.ground_clear])
    a = math.radians(deg)
    # tip-up = rotation about -y
    R = np.array([[math.cos(a), 0, -math.sin(a)], [0, 1, 0], [math.sin(a), 0, math.cos(a)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = c - R @ c
    return T


def main(p=DEFAULT):
    itf = spokes.load("software-mfg", "interference")
    its = bot.items(p)
    M = meshes(its)
    ok = True
    print("assembly gates")

    # 1. static fit
    scene = itf.Scene()
    for n, m in M.items():
        scene.place(n, m)
    rep = scene.check()
    hits = rep["interferences"]
    ok &= gate(not hits, "static fit (no interferences)",
               "; ".join(f"{h['pair']} {h['volume_mm3']}mm^3" for h in hits[:6]) or
               f"{len(M)} parts, tightest {rep['clearances'][0]['pair']} {rep['clearances'][0]['clearance_mm']}mm"
               if rep["clearances"] else "")

    # 2. arm swing: fork + both hubs rotate together; everything else static
    arm_parts = [i.name for i in its if i.group == "arm"]
    statics = itf.Scene()
    for n, m in M.items():
        if n not in arm_parts:
            statics.place(n, m)
    swing = p.arm_max_deg - p.arm_rest_deg
    Ts = [rot_about_pivot(p, swing * k / 40) for k in range(41)]
    worst = None
    for n in arm_parts:
        # hubs ride IN the chassis slot by design (clearance fit) — the sweep proves it stays clear
        r = itf.sweep(M[n], Ts, statics)
        if not r["ok"]:
            worst = (n, r["first_hit"])
            break
    ok &= gate(worst is None, f"arm swings rest ({p.arm_rest_deg:.1f}°) -> up-stop ({p.arm_max_deg:.0f}°)",
               f"{worst[0]} hits {worst[1]['part']} at {p.arm_rest_deg + swing * worst[1]['index'] / 40:.0f}°"
               if worst else "clear")

    # 3. recipe sweep
    steps = recipe(p)
    installed = itf.Scene()
    placed = set()
    n_fail = 0
    for s in steps:
        if not s.parts:
            continue
        d = np.array(s.approach, float)
        d /= np.linalg.norm(d)
        against = itf.Scene()
        pool = s.against if s.against is not None else [n for n in placed]
        for n in pool:
            if n not in s.parts:
                against.place(n, M[n])
        Ts = [translate(-d * STANDOFF_MM * (1 - k / N_WAY)) for k in range(N_WAY + 1)]
        res = [(n, itf.sweep(M[n], Ts, against)) for n in s.parts]
        bad = [(n, r) for n, r in res if not r["ok"]]
        axis = "top-down" if np.allclose(d, (0, 0, -1)) else "approach (" + ", ".join(f"{x:+.2f}" for x in d) + ")"
        if bad:
            n, r = bad[0]
            n_fail += 1
            gate(False, f"step {s.name:12s} [{s.op}] {axis}",
                 f"{n} hits {r['first_hit']['part']} {STANDOFF_MM * (1 - r['first_hit']['index'] / N_WAY):.1f}mm before seating")
        else:
            gate(True, f"step {s.name:12s} [{s.op}] {axis}")
        if s.against is None or s.name == "chassis":
            placed.update(s.parts)
    ok &= n_fail == 0
    ok &= gate(not (set(M) - placed), "recipe installs every part", f"missing {sorted(set(M) - placed)}" if set(M) - placed else "")

    # 4. DFA report + schedule on software-mfg's op-graph
    og = spokes.load("software-mfg", "opgraph")
    g = opgraph(p, og)
    sched, makespan = og.schedule(g)
    used = sorted({s.op for s in steps})
    off_axis = [s.name for s in steps if not np.allclose(s.approach, (0, 0, -1))]
    manual = [s.name for s in steps if s.op == "manual"]
    print("DFA report")
    print(f"  build-cell op types : {len(used)} {used}")
    print(f"  off-axis approaches : {len(off_axis)} {off_axis}")
    print(f"  manual steps        : {len(manual)} {manual}  <- DFA debt")
    print(f"  fastener family     : M2 only ({sum(s.count for s in steps if s.op == 'drive_screw')} screws, "
          f"{sum(s.count for s in steps if s.op == 'heat_set_insert')} inserts)")
    print(f"  cycle time          : {makespan:.0f}s scheduled vs {og.sequential_makespan(g):.0f}s sequential "
          f"(one server per op type)")
    print("ASSEMBLY", "GREEN" if ok else "RED")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
