"""check_flip.py — the WEAPON gate: can this lifter flip a twin? (MuJoCo, sim/duel_sim.py)

Sweeps the design levers that matter — arm length x servo — against the attack scenarios, then
gates the CURRENT params: it must flip a twin in at least one scenario across the whole servo/
motor torque band (x0.7 .. x1.3 of catalog-nominal). A pass only at nominal is not a pass.

    python scripts/check_flip.py            gate + sweep table   (-> build/flip_sweep.txt)
    python scripts/check_flip.py --quick    gate only
"""
import os
import sys
from concurrent.futures import ProcessPoolExecutor

os.environ.setdefault("MUJOCO_GL", "osmesa")
from _common import ROOT, gate

sys.path.insert(0, os.path.join(ROOT, "sim"))

from params import DEFAULT  # noqa: E402

SCENARIOS = ("rear", "broadside")
ARM_LENS = (52, 64, 76, 88)
SERVOS = ("ES08MA2", "MG90S", "MG92B")
BAND = (0.7, 1.0, 1.3)


def _job(args):
    import duel_sim as S
    arm_len, servo, sc, scale = args
    p = DEFAULT.with_(arm_len=arm_len, servo=servo)
    r = S.run(p, p, S.SimCfg(scenario=sc, servo_scale=scale, motor_scale=scale))
    return args, dict(flipped=r["flipped"], tiltB=r["max_tiltB"], self_flip=r["self_flip"], sat=r["saturated_frac"])


def _runs(jobs):
    with ProcessPoolExecutor(max_workers=min(len(jobs), os.cpu_count() or 4)) as ex:
        return dict(ex.map(_job, jobs))


def main(quick=False, p=DEFAULT):
    print("flip gate (twin vs twin, MuJoCo)")
    ok = True
    gate_jobs = [(p.arm_len, p.servo, sc, s) for sc in SCENARIOS for s in BAND]
    res = _runs(gate_jobs)
    for sc in SCENARIOS:
        cells = [res[(p.arm_len, p.servo, sc, s)] for s in BAND]
        detail = "  ".join(f"x{s}: {'FLIP' if c['flipped'] else 'no'} {c['tiltB']:.0f}°" for s, c in zip(BAND, cells))
        print(f"  {sc:9s} arm {p.arm_len:.0f} mm, {p.servo}: {detail}")
    robust = [sc for sc in SCENARIOS if all(res[(p.arm_len, p.servo, sc, s)]["flipped"] for s in BAND)]
    selfflip = [k for k, v in res.items() if v["self_flip"]]
    ok &= gate(bool(robust), "flips a twin across the torque band", f"robust in: {robust}" if robust else "no scenario")
    ok &= gate(not selfflip, "never flips itself", str(selfflip) if selfflip else "")

    if not quick:
        jobs = [(a, s, sc, 1.0) for a in ARM_LENS for s in SERVOS for sc in SCENARIOS]
        sw = _runs(jobs)
        lines = [f"{'arm':>4} {'servo':8} " + " ".join(f"{sc:>16}" for sc in SCENARIOS)]
        for a in ARM_LENS:
            for s in SERVOS:
                cells = []
                for sc in SCENARIOS:
                    c = sw[(a, s, sc, 1.0)]
                    cells.append(f"{'FLIP' if c['flipped'] else '  - '} {c['tiltB']:4.0f}° sat{c['sat']:.0%}".rjust(16))
                lines.append(f"{a:>4} {s:8} " + " ".join(cells))
        table = "\n".join(lines)
        print("sweep @ nominal torque (max twin tilt; sat = time servo at its torque limit)")
        print(table)
        os.makedirs(os.path.join(ROOT, "build"), exist_ok=True)
        with open(os.path.join(ROOT, "build", "flip_sweep.txt"), "w") as f:
            f.write(table + "\n")
    print("FLIP", "GREEN" if ok else "RED")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main(quick="--quick" in sys.argv) else 1)
