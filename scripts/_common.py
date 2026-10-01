"""Shared setup for the gate scripts: import paths + build123d -> trimesh conversion."""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for sub in ("cad", "mfg"):
    sys.path.insert(0, os.path.join(ROOT, sub))

import numpy as np          # noqa: E402
import trimesh              # noqa: E402


def to_mesh(solid, tol=0.05):
    """build123d solid (mm) -> watertight trimesh via its own tessellation."""
    verts, tris = solid.tessellate(tol)
    m = trimesh.Trimesh(np.array([(v.X, v.Y, v.Z) for v in verts]), np.array(tris), process=True)
    m.merge_vertices()
    return m


def meshes(items):
    return {i.name: to_mesh(i.solid) for i in items if i.solid is not None}


def gate(ok: bool, label: str, detail: str = ""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {label}" + (f" — {detail}" if detail else ""))
    return ok
