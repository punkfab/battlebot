# battlebot — kickoff brief (2026-09-30)

- **Problem:** punkfab needs an attention-getting demo that exercises the whole pipeline —
  design blocks → sim gates → build cells — ending in two real antweights that can fight.
- **Done looks like (v1):** two printed antweights at the low end of the class (~350–400 g,
  limit 454 g): skid-steer drive, wedge + servo lifter, ESP32 phone control, able to battle
  each other. Every claim is a runnable gate (`make check`).
- **Roadmap (post-v1):**
  1. Robotic assembly via task build cells driven by an assembly recipe (`../buildcell`).
  2. Second-process variant (CNC / bent-wire frame, TPU armor) from the same parametric source.
  3. `autobot`-style configuration search over bot designs.
- **Not now:** spinners / stored-energy weapons, electronic scoring, autonomy, second process,
  actually building any build cell. But v1 is **designed for robotic assembly from day one**:
  top-down stacking, one fastener type, no loose wires (connectorized carrier board).
- **First slice:** component envelope models (mass + keep-out) → parametric chassis tub that
  locates them → gates: mass budget, MuJoCo "flip a twin" (lifter torque vs arm length),
  assembly-sequence check (every part inserts top-down, nothing collides).
- **Open questions:** can a light servo flip ~400 g? Do printed parts survive hits? Build-cell
  granularity (task cells vs product cells) — TBD until the battlebot recipe shows which
  operations it actually needs.

## Vocabulary (don't let "cell" mean three things)

| term | means | lives in |
|---|---|---|
| **design block** | reusable design unit with mates, mass, controller (wheel module, lifter, chassis) | `autobot/blocks/`, this repo's `blocks/` |
| **build cell** | physical factory station doing one class of operation (drive fastener, set insert, press-fit, mate connector) | `../buildcell`, `software-mfg/sim/*_cell.py` |
| **spoke / reference** | another repo composed by reference | `cells.yaml` today (legacy name) |

New code reserves the word **cell** for build cells only. A product is not a cell; it is a
**recipe** (an op-graph routed across build cells).

## Decisions

- **Antweight over fairyweight:** fairyweight's size constrains battery + ESP32 packaging too hard;
  antweight still prints fast.
- **Servo lifter + wedge** over spinner / balloon / sumo-only: safe, visible, and contact/push
  dynamics are what MuJoCo does well.
- **ESP32 + phone control** over off-the-shelf RC: enables a connectorized carrier board (robotic
  assembly), telemetry and scoring later. Accepts firmware + radio-latency risk.
- **Skid steer, not omni/mecanum:** rollers have no lateral grip; a pusher gets shoved sideways.
  Skid-steer becomes a new modality contributed to `robot-locomotion`.
- **Antweight BOM** (N20 gearmotors, micro servo, 2S LiPo, small driver, ESP32) belongs in
  `robot-bom/antweight/`, primary-sourced like the rest of that repo.
