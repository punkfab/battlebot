"""spokes.py — import modules from sibling punkfab repos BY REFERENCE (see ../spokes.yaml).

Loads by file path under a namespaced module name, so a spoke's `sim/` package can't collide
with this repo's `sim/`.
"""
from __future__ import annotations

import importlib.util
import os
import sys

import yaml

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _spokes():
    with open(os.path.join(ROOT, "spokes.yaml")) as f:
        return yaml.safe_load(f)["spokes"]


def spoke_path(name: str) -> str:
    env = "BATTLEBOT_" + name.upper().replace("-", "_")
    p = os.environ.get(env) or os.path.join(ROOT, _spokes()[name]["path"])
    p = os.path.abspath(p)
    if not os.path.isdir(p):
        raise FileNotFoundError(f"spoke '{name}' not found at {p} (set {env})")
    return p


def load(spoke: str, what: str):
    """load('software-mfg', 'interference') -> the module named in spokes.yaml `provides`.
    A spoke marked `syspath: true` goes on sys.path and loads under its own module names (its
    modules import each other by name)."""
    cfg = _spokes()[spoke]
    rel = cfg["provides"][what]
    if cfg.get("syspath"):
        root = spoke_path(spoke)
        if root not in sys.path:
            sys.path.insert(0, root)
        return importlib.import_module(os.path.splitext(os.path.basename(rel))[0])
    modname = f"spoke_{spoke.replace('-', '_')}_{what}"
    if modname in sys.modules:
        return sys.modules[modname]
    spec = importlib.util.spec_from_file_location(modname, os.path.join(spoke_path(spoke), rel))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[modname] = mod
    spec.loader.exec_module(mod)
    return mod
