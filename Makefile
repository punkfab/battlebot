# battlebot — punkfab antweight demo platform. Build the design, run the gates.
#   make check   every gate (parts, mass, assembly, flip)
#   make cad     build STLs + renders
#   make sim     interactive MuJoCo duel (drive A with the Control-pane sliders)
#   make help    list all targets
PY ?= python3

.PHONY: help check parts-check mass-check assembly-check ir-check flip-check flip-sweep cad asm 3mf sim sim-demo sim-selftest snap clean
.DEFAULT_GOAL := help

help: ## list targets
	@grep -hE '^[a-zA-Z0-9_-]+:.*?## ' $(MAKEFILE_LIST) | \
	  awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-16s\033[0m %s\n", $$1, $$2}'

check: parts-check mass-check assembly-check ir-check flip-check ## run every gate
	@echo "all battlebot gates green"

parts-check: ## printed parts single-body + watertight, params valid, ground clearance, invertible
	$(PY) scripts/check_parts.py

mass-check: ## antweight mass budget (CAD volume x density x fill + catalog parts)
	$(PY) scripts/check_mass.py

assembly-check: ## static fit, arm swing, recipe sweep (design for robotic assembly) + DFA report
	$(PY) scripts/check_assembly.py

ir-check: ## assembly IR well-formed; MJCF + 3MF derived from it agree with the CAD
	$(PY) scripts/check_ir.py

flip-check: ## weapon gate: flips a twin across the torque band (MuJoCo)
	$(PY) scripts/check_flip.py --quick

flip-sweep: ## arm length x servo x scenario sweep -> build/flip_sweep.txt
	$(PY) scripts/check_flip.py

cad: ## build every part STL + assembly renders -> build/
	cd cad && $(PY) chassis.py && $(PY) parts.py && $(PY) bot.py

asm: ## write the top-level ASSEMBLY IR -> build/battlebot.asm.json
	$(PY) cad/assembly.py

3mf: ## 3MF: assembly package (IR embedded), posed assembly, print plates per material
	$(PY) mfg/export_3mf.py

sim: ## INTERACTIVE MuJoCo duel viewer
	$(PY) sim/duel_sim.py

sim-demo: ## headless duel scenarios -> build/duel_*.gif
	$(PY) sim/duel_sim.py --demo

sim-selftest: ## build + step the duel, no window (CI)
	$(PY) sim/duel_sim.py --selftest

snap: ## log the current render: make snap NOTE=short-note
	@test -n "$(NOTE)" || (echo "usage: make snap NOTE=short-note" && exit 1)
	@mkdir -p renders/bot; n=$$(printf '%04d' $$(( $$(ls renders/bot/*.png 2>/dev/null | wc -l) + 1 ))); \
	  cp build/bot_iso.png renders/bot/$$n-$(NOTE).png; \
	  echo "- $$n-$(NOTE).png — $$(date -I) — $(NOTE)" >> renders/bot/INDEX.md; echo "logged renders/bot/$$n-$(NOTE).png"

clean: ## remove generated artifacts
	rm -rf build/*.stl build/*.scad build/*.tmp __pycache__ */__pycache__
