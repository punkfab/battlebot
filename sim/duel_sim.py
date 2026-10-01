"""duel_sim.py — two antweights in MuJoCo: can the lifter FLIP A TWIN?

The model is generated from the bot's ASSEMBLY IR (cad/assembly.py -> featuretree mjcf_emit): bodies,
joints, meshes, colliders and masses all derive from the CAD; nothing is re-described here.
Actuators are NOT ideal:
  * drive  : N20 as a linear DC motor law  tau = u*stall - (stall/w0)*omega  (u=0 -> brake, like
             an H-bridge in brake mode — a gearmotor is barely backdrivable)
  * lifter : servo P-loop saturating at stall torque, derated linearly to 0 at no-load speed
Both are catalog-nominal (bom.py) — the gate re-runs at torque x0.7 and x1.3 (report a band).

Modes:  python sim/duel_sim.py              interactive viewer (A throttle/steer/arm sliders)
        python sim/duel_sim.py --demo       headless: scenario gifs -> build/duel_*.gif
        python sim/duel_sim.py --selftest   build + step, assert finite (CI)
"""
from __future__ import annotations

import math
import os
import sys

_MODE = "demo" if "--demo" in sys.argv else "selftest" if "--selftest" in sys.argv else "interactive"
os.environ.setdefault("MUJOCO_GL", "glfw" if _MODE == "interactive" else "osmesa")

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "cad"))

import numpy as np          # noqa: E402
import mujoco               # noqa: E402
from dataclasses import dataclass, field   # noqa: E402

import assembly             # noqa: E402
import bom                  # noqa: E402
import spokes               # noqa: E402
from params import P, DEFAULT   # noqa: E402

MJ = spokes.load("featuretree", "mjcf_emit")

BUILD = os.path.join(ROOT, "build")
MM = 1e-3


@dataclass
class SimCfg:
    scenario: str = "broadside"       # broadside | rear | head_on
    gap_mm: float = 20.0              # initial wedge-tip -> opponent gap
    servo_scale: float = 1.0          # stall-torque multiplier (domain-randomisation band)
    motor_scale: float = 1.0
    mu_tyre: float = 0.9
    mu_print: float = 0.35
    dt: float = 2e-4
    duration: float = 3.0
    lift_after_contact_s: float = 0.15  # tactics: how deep to drive the wedge in before lifting
    lift_throttle: float = 0.6          # tactics: drive while lifting
    wheel_backdrive_Nm: float = 0.008  # N20 gearbox static friction at the output (GUESS — bench-measure)
    kp_servo: float = 8.0             # N·m/rad before saturation (stiff hobby-servo P loop)


# --------------------------------------------------------------------------------- model
# The model is DERIVED: cad/assembly.py writes the bot as an assembly IR, featuretree's mjcf_emit
# turns it into bodies (rigid groups) / joints (mates) / geometry (each part's own mesh, collision
# via hull or the part's own convex pieces) / mass (CAD roll-up, inertia integrated from the mesh).
# This file only adds what is not a design fact: the arena, actuator laws, and tactics.
def _fragment(p: P, cfg: SimCfg, tag: str, pos, yaw_deg):
    asm, sim = assembly.build(p, mu_tyre=cfg.mu_tyre, mu_print=cfg.mu_print)
    sim.update(free_root=True,
               joint_attrs={"wheel_L": {"frictionloss": cfg.wheel_backdrive_Nm},
                            "wheel_R": {"frictionloss": cfg.wheel_backdrive_Nm},
                            "lifter": {"damping": 2e-4}},
               # zero-gear proxies = labelled UI sliders; the real torques are the laws in Actuation
               actuators=[{"mate": "wheel_L", "kind": "motor", "name": "throttle", "gear": 0, "ctrlrange": "-1 1"},
                          {"mate": "wheel_R", "kind": "motor", "name": "steer", "gear": 0, "ctrlrange": "-1 1"},
                          {"mate": "lifter", "kind": "motor", "name": "arm_cmd", "gear": 0, "ctrlrange": "0 1"}])
    return MJ.emit_fragment(asm, assembly.BUILD, sim, prefix=f"{tag}_",
                            root_pos=(pos[0] * MM, pos[1] * MM, 0.0005), root_yaw_deg=yaw_deg)


def make_xml(pa: P, pb: P, cfg: SimCfg):
    tip_x = pa.pivot_x + pa.arm_len * math.cos(math.radians(pa.arm_rest_deg))
    if cfg.scenario == "broadside":
        xb, yaw_b = tip_x + pb.wheel_y + pb.wheel_w / 2 + cfg.gap_mm, 90.0
    elif cfg.scenario == "rear":
        xb, yaw_b = tip_x + pb.ch_len / 2 + cfg.gap_mm, 0.0
    else:  # head_on
        tip_b = pb.pivot_x + pb.arm_len * math.cos(math.radians(pb.arm_rest_deg))
        xb, yaw_b = tip_x + tip_b + cfg.gap_mm, 180.0
    fa, fb = _fragment(pa, cfg, "A", (0, 0), 0.0), _fragment(pb, cfg, "B", (xb, 0), yaw_b)
    j = lambda k: "\n    ".join(f[k] for f in (fa, fb) if f[k])
    return f"""<mujoco model="antweight_duel">
  <compiler angle="degree" autolimits="true"/>
  <option timestep="{cfg.dt}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual><global offwidth="960" offheight="720"/><quality shadowsize="2048"/></visual>
  <asset>
    {j("asset")}
    <texture name="grid" type="2d" builtin="checker" rgb1=".85 .85 .85" rgb2=".75 .75 .75" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8"/>
  </asset>
  <worldbody>
    <light pos="0 -0.3 0.6" dir="0 0.4 -1" diffuse=".9 .9 .9"/>
    <geom name="floor" type="plane" size="0.6 0.6 0.01" material="grid" friction="1 0.005 0.0001"/>
{j("body")}
  </worldbody>
  <actuator>
    {j("actuator")}
  </actuator>
</mujoco>"""


# ----------------------------------------------------------------------------- actuation
class Actuation:
    """Real-ish actuator laws applied as joint torques (internal: reaction lands on the chassis)."""

    def __init__(self, model, pa: P, pb: P, cfg: SimCfg):
        self.m, self.cfg = model, cfg
        self.bots = {}
        for tag, p in (("A", pa), ("B", pb)):
            mot, srv = bom.CATALOG[p.motor], bom.CATALOG[p.servo]
            j = lambda n: model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            q = lambda n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            self.bots[tag] = dict(
                wL=j(f"{tag}_wheel_L"), wR=j(f"{tag}_wheel_R"), arm=j(f"{tag}_lifter"), arm_q=q(f"{tag}_lifter"),
                stall=mot.stall_Nm * cfg.motor_scale, w0=mot.rpm_noload * 2 * math.pi / 60,
                s_stall=srv.stall_Nm * cfg.servo_scale, s_w0=(math.pi / 3) / srv.s_per_60,
                arm_span=math.radians(p.arm_max_deg - p.arm_rest_deg),
                cmd=[0.0, 0.0, 0.0])      # throttle, steer, arm fraction
        self.arm_torque = {"A": 0.0, "B": 0.0}

    def set(self, tag, throttle, steer, arm_frac):
        self.bots[tag]["cmd"] = [throttle, steer, arm_frac]

    def apply(self, d):
        for tag, b in self.bots.items():
            thr, st, af = b["cmd"]
            for dof, u in ((b["wL"], thr - st), (b["wR"], thr + st)):
                u = max(-1.0, min(1.0, u))
                w = d.qvel[dof]
                d.qfrc_applied[dof] = float(np.clip(u * b["stall"] - b["stall"] / b["w0"] * w, -b["stall"], b["stall"]))
            q, w = d.qpos[b["arm_q"]], d.qvel[b["arm"]]
            lim = b["s_stall"] * max(0.0, 1.0 - abs(w) / b["s_w0"])
            tau = float(np.clip(self.cfg.kp_servo * (af * b["arm_span"] - q) - 0.02 * w, -lim, lim))
            d.qfrc_applied[b["arm"]] = tau
            self.arm_torque[tag] = tau


def tilt_deg(d, model, tag):
    """Angle between the bot's up-axis and world up (0 upright, 180 inverted)."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{tag}_chassis")
    return math.degrees(math.acos(max(-1.0, min(1.0, d.xmat[bid][8]))))


def _touching(model, d, body_a, body_b_root):
    """Any contact between a geom of body `body_a` and anything in the tree rooted at body_b_root."""
    for k in range(d.ncon):
        c = d.contact[k]
        for g1, g2 in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
            if model.geom_bodyid[g1] == body_a and model.body_rootid[model.geom_bodyid[g2]] == body_b_root:
                return True
    return False


# ------------------------------------------------------------------------------ episode
def run(pa: P = DEFAULT, pb: P | None = None, cfg: SimCfg | None = None, render: bool = False):
    """A charges, and on wedge contact lifts to the up-stop while still pushing. B brakes."""
    pb = pb or pa
    cfg = cfg or SimCfg()
    model = mujoco.MjModel.from_xml_string(make_xml(pa, pb, cfg))
    d = mujoco.MjData(model)
    act = Actuation(model, pa, pb, cfg)
    renderer = mujoco.Renderer(model, 480, 640) if render else None   # bound to THIS model
    frames_every = int(1 / 30 / cfg.dt)
    arm_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "A_arm")
    b_root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "B_chassis")
    t_contact, frames = None, []
    log = dict(t=[], tiltA=[], tiltB=[], arm=[], tau=[])
    steps = int(cfg.duration / cfg.dt)
    for k in range(steps):
        t = k * cfg.dt
        if t_contact is None and _touching(model, d, arm_body, b_root):
            t_contact = t
        lifting = t_contact is not None and t - t_contact >= cfg.lift_after_contact_s
        act.set("A", cfg.lift_throttle if lifting else 1.0, 0.0, 1.0 if lifting else 0.0)
        act.set("B", 0.0, 0.0, 0.0)
        act.apply(d)
        mujoco.mj_step(model, d)
        if k % 50 == 0:
            log["t"].append(t)
            log["tiltA"].append(tilt_deg(d, model, "A"))
            log["tiltB"].append(tilt_deg(d, model, "B"))
            log["arm"].append(math.degrees(d.qpos[act.bots["A"]["arm_q"]]) + pa.arm_rest_deg)
            log["tau"].append(act.arm_torque["A"])
        if renderer is not None and k % frames_every == 0:
            renderer.update_scene(d, camera=_cam(d, model))
            frames.append(renderer.render())
    if not np.all(np.isfinite(d.qpos)):
        raise FloatingPointError("sim diverged")
    res = dict(
        flipped=max(log["tiltB"]) > 90.0,
        max_tiltB=max(log["tiltB"]),
        max_tiltA=max(log["tiltA"]),
        self_flip=max(log["tiltA"]) > 60.0,
        contact_t=t_contact,
        max_arm=max(log["arm"]),
        saturated_frac=float(np.mean(np.abs(log["tau"]) >= 0.98 * act.bots["A"]["s_stall"])),
        log=log, frames=frames)
    return res


def _cam(d, model):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.09, 0, 0.02]
    cam.distance, cam.azimuth, cam.elevation = 0.42, -60, -22
    return cam


# -------------------------------------------------------------------------------- modes
def selftest():
    r = run(cfg=SimCfg(duration=0.3))
    print(f"selftest ok: finite, contact_t={r['contact_t']}, max tilt A {r['max_tiltA']:.1f}°")


def run_demo():
    import imageio
    os.makedirs(BUILD, exist_ok=True)
    for sc in ("broadside", "rear", "head_on"):
        r = run(cfg=SimCfg(scenario=sc), render=True)
        path = os.path.join(BUILD, f"duel_{sc}.gif")
        imageio.mimsave(path, r["frames"], fps=30, loop=0)
        print(f"{sc:9s} flipped={r['flipped']} maxtiltB={r['max_tiltB']:.0f}° arm={r['max_arm']:.0f}° -> {path}")


def run_interactive():
    """Drive A with the Control-pane sliders (throttle / steer / arm); B holds still."""
    import time
    import mujoco.viewer
    cfg = SimCfg()
    model = mujoco.MjModel.from_xml_string(make_xml(DEFAULT, DEFAULT, cfg))
    d = mujoco.MjData(model)
    act = Actuation(model, DEFAULT, DEFAULT, cfg)
    with mujoco.viewer.launch_passive(model, d) as v:
        while v.is_running():
            t0 = time.time()
            act.set("A", d.ctrl[0], d.ctrl[1], d.ctrl[2])
            act.set("B", d.ctrl[3], d.ctrl[4], d.ctrl[5])
            for _ in range(int(1 / 60 / cfg.dt)):
                act.apply(d)
                mujoco.mj_step(model, d)
            v.sync()
            time.sleep(max(0.0, 1 / 60 - (time.time() - t0)))


if __name__ == "__main__":
    {"interactive": run_interactive, "demo": run_demo, "selftest": selftest}[_MODE]()
