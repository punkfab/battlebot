# battlebot

**punkfab's antweight demo platform.** A parametric 1 lb combat robot designed as code, checked
in simulation, and **designed for robotic assembly**. It's the first whole machine to go all the
way through the punkfab pipeline: design blocks → sim gates → build cells. The goal and the
decisions behind it are in [BRIEF.md](./BRIEF.md).

## Status (2026-09-30): first slice

| gate | status | what it proves |
|---|---|---|
| `make parts-check` | green | every printed part is one watertight body; params valid; only the tyres and wedge touch the floor; drives inverted (wheels stand 3.4 mm proud of the lid) |
| `make mass-check` | green | **176 g**: 278 g of headroom under the 454 g limit (75 g of it is catalog-nominal, not yet weighed) |
| `make assembly-check` | green | no interferences; the arm swings rest→100° clear; **every recipe step inserts along a straight line** against everything already installed |
| `make ir-check` | green | the assembly IR is valid (mobility 3), and the MJCF and 3MF generated from it match the CAD: mass exact, placement within 0.006 mm |
| `make flip-check` | **red** | a servo lifter **cannot flip a same-size twin** from flat ground (see findings) |

### Findings so far
- **Flipping a twin with a rotary lifter is ruled out by geometry, not torque.** The servo never
  saturates, at any arm length from 52 to 88 mm or with any of the three servos tried. The twin
  tops out at 12–20° of tilt: the fork tip swings backward as it rises, so the twin slides off
  long before its centre of mass crosses its pivot (that needs about 75°, i.e. about 130 mm of
  rear lift). The hand calculation agrees.
- **The passive wedge does most of the work.** With the arm at rest, the wedge lifts a reversing
  twin's wheels off the floor 75% of the time and shoves it 73 mm. A full arm swing makes it
  *worse* (37 mm, 7%): the twin pops off and gets its traction back. Holding 5–20° keeps its wheels
  off the floor 100% of the time but pushes it less (21–31 mm), which isn't explained yet.
  (Re-measured on the model generated from the IR; the first hand-built sim gave 92% / 122 mm,
  the same pattern.)
- **The DFA gate found real problems before any printing:**
  - Motor shafts and the servo spline passing through the walls can't be inserted top-down. Fixed
    with slots in the walls that are open at the rim; the lid caps them.
  - The servo ear passed through a board post on the way down. Fixed by shortening the board.
  - The arm plates dipped below the floor. Fixed, and now covered by a ground-clearance gate.
- **DFA debt:** 1 manual step (connecting the wire harness). 5 build-cell operation types, one
  fastener family (M2 screw + heat-set insert). Cycle time is 214 s scheduled vs 266 s
  sequential, on software-mfg's op-graph scheduler.

## Assemblies are the top-level artifact

`cad/assembly.py` writes the bot as a **featuretree assembly IR** (`build/battlebot.asm.json`):
- **Occurrences:** each part's own local geometry plus its placement.
- **Mates:** `fastened` within each rigid group; `revolute` for the wheels and the lifter.

Everything downstream is generated from it (featuretree, upstream):

| output | generator | what it is |
|---|---|---|
| MuJoCo model | `mjcf_emit` | bodies = fastened groups, joints = mates, geometry = the parts' own meshes; collision = convex hull, or the part's own convex pieces (`parts.collision_pieces`); inertia integrated from the mesh |
| `build/battlebot.3mf` | `threemf_emit` | the assembly as a 3MF components object, with the IR embedded in the package |
| `build/battlebot_lift.3mf` | `assembly_kin` + `threemf_emit` | the same assembly posed by the kinematic solver (handles closed loops) |
| `build/plate_PETG.3mf`, `plate_TPU.3mf` | `threemf_emit` | print plates: parts in their print orientation (`parts.PRINT`), packed on the bed, one plate per material |

The printed parts are not yet in featuretree's part IR; their geometry is carried as part-local STLs
until they move over.

## Layout

```
cad/        params.py (SINGLE SOURCE OF TRUTH, P dataclass → variants), bom.py (component
            envelopes, nominal), layout.py (datums), chassis.py, parts.py, bot.py (parts placed by mates),
            assembly.py (→ assembly IR)
mfg/        recipe.py — assembly recipe: steps × build-cell operation type × approach vector
            export_3mf.py — assembly / posed / print-plate 3MF
sim/        duel_sim.py — twin vs twin; model generated from the assembly IR; DC-motor + servo laws
scripts/    check_parts / check_mass / check_assembly / check_flip
renders/    dated render log (make snap NOTE=...)
spokes.yaml sibling repos composed by reference (software-mfg interference + opgraph, ...)
```

**Vocabulary:** *design block* = reusable design unit; *build cell* = factory station (`../buildcell`);
*spoke* = another repo composed by reference. The word "cell" is reserved for build cells.

## Setup

battlebot imports two sibling repos by reference (see `spokes.yaml`), so clone them next to it:

```bash
git clone git@github.com:punkfab/battlebot.git
git clone git@github.com:punkfab/featuretree.git     # assembly IR, pose solver, MJCF + 3MF emitters
git clone git@github.com:punkfab/software-mfg.git    # interference checker, op-graph scheduler
cd battlebot && uv sync                              # or: pip install -e .
```

Override a spoke's location with `BATTLEBOT_FEATURETREE=/path` or `BATTLEBOT_SOFTWARE_MFG=/path`.
The renders need OpenSCAD; headless sim renders use OSMesa.

## Run

```bash
make check        # all gates
make cad          # STLs + renders -> build/
make asm          # assembly IR -> build/battlebot.asm.json
make 3mf          # assembly + posed + print-plate 3MF -> build/
make sim          # interactive duel (sliders: A throttle / steer / arm)
make sim-demo     # scenario gifs -> build/duel_*.gif
make flip-sweep   # arm length × servo × scenario table
```

## Open / next
- **Weapon:** staying with the simple rotary lifter for now (decided 2026-09-30). A four-bar is
  now possible in the IR: featuretree's assembly IR supports loop-closing mates, and the pose
  solver and MJCF emitter handle them (tested against the closed-form four-bar).
- **Bench-verify:** servo stall torque, N20 speed/stall/backdrive, masses (`fill` calibration).
- **Part IR:** move the printed parts into featuretree's part IR. This needs one new primitive,
  an additive extrude on an arbitrary plane (the mirror of `prism_cut`), for the pivot boss and the
  off-centre arm plates. Then FreeCAD and Onshape files come for free.
- **Upstream:** move antweight parts into `robot-bom/antweight/`; skid-steer into `robot-locomotion`.
- **Wiring:** a board-mount connector scheme that removes the manual harness step.
