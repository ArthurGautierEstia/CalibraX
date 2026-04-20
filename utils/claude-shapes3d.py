"""
shapes3d.py — Intersection testing for 3D primitive shapes.

Supported shapes
────────────────
  Box(l, L, h)       Rectangular box  (l=width, L=depth, h=height in local frame)
  Cylinder(r, h)     Cylinder          (r=radius, h=height along local Z)
  Sphere(r)          Sphere            (r=radius)

Each shape accepts:
  position=(x, y, z)   World-space centre           (default: origin)
  up=(x, y, z)         Orientation – local Z axis   (default: (0, 0, 1))

Public API
──────────
  intersects(shape_a, shape_b) -> bool

Algorithm
─────────
GJK (Gilbert–Johnson–Keerthi) works for any pair of convex shapes via
their support functions, so no combinatorial case switching is needed.

Usage examples
──────────────
  from shapes3d import Box, Cylinder, Sphere, intersects

  # Two overlapping boxes
  a = Box(2, 2, 2, position=(0, 0, 0))
  b = Box(2, 2, 2, position=(1, 0, 0))
  intersects(a, b)   # True

  # Sphere just missing a box
  s = Sphere(0.4, position=(0, 0, 3))
  b = Box(2, 2, 2, position=(0, 0, 0))
  intersects(s, b)   # False

  # Tilted cylinder touching a sphere
  cyl = Cylinder(0.5, 4, position=(0, 0, 0), up=(1, 0, 0))
  sph = Sphere(0.6, position=(2.9, 0, 0))
  intersects(cyl, sph)   # True
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Internal: rotation helpers
# ─────────────────────────────────────────────────────────────────────────────

def _rotation_from_up(up: np.ndarray) -> np.ndarray:
    """
    Build a 3×3 rotation matrix R such that R @ [0, 0, 1] == normalise(up).
    Uses Rodrigues' rotation formula.
    """
    up = np.asarray(up, dtype=float)
    norm = np.linalg.norm(up)
    if norm < 1e-12:
        raise ValueError("up vector must be non-zero")
    up = up / norm

    z = np.array([0., 0., 1.])

    if np.allclose(up, z, atol=1e-9):
        return np.eye(3)

    if np.allclose(up, -z, atol=1e-9):
        # 180° rotation around X axis
        return np.diag([1., -1., -1.])

    v = np.cross(z, up)               # rotation axis (un-normalised)
    c = float(np.dot(z, up))           # cos θ
    s = np.linalg.norm(v)              # sin θ

    # Skew-symmetric matrix of v
    vx = np.array([[ 0.,   -v[2],  v[1]],
                   [ v[2],  0.,   -v[0]],
                   [-v[1],  v[0],  0.  ]])

    return np.eye(3) + vx + (vx @ vx) * ((1.0 - c) / (s * s))


# ─────────────────────────────────────────────────────────────────────────────
# Shape classes
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Box:
    """
    Axis-aligned rectangular box in *local* space, then oriented via 'up'.

    Parameters
    ----------
    l, L, h    : dimensions along local X, Y, Z axes
    position   : world-space centre
    up         : world-space direction that maps to local +Z
    """
    l: float
    L: float
    h: float
    position: Tuple[float, float, float] = (0., 0., 0.)
    up: Tuple[float, float, float]       = (0., 0., 1.)

    def __post_init__(self) -> None:
        self._pos  = np.array(self.position, dtype=float)
        self._rot  = _rotation_from_up(np.array(self.up, dtype=float))
        self._half = np.array([self.l / 2., self.L / 2., self.h / 2.])

    def support(self, d: np.ndarray) -> np.ndarray:
        """Furthest point of the box in world direction *d*."""
        d_local = self._rot.T @ d
        # Component-wise ±half-extent in the sign of d_local
        s_local = np.where(d_local >= 0, self._half, -self._half)
        return self._pos + self._rot @ s_local


@dataclass
class Cylinder:
    """
    Cylinder whose axis runs along local Z.

    Parameters
    ----------
    r       : radius
    h       : total height
    position: world-space centre
    up      : world-space direction that maps to local +Z (cylinder axis)
    """
    r: float
    h: float
    position: Tuple[float, float, float] = (0., 0., 0.)
    up: Tuple[float, float, float]       = (0., 0., 1.)

    def __post_init__(self) -> None:
        self._pos = np.array(self.position, dtype=float)
        self._rot = _rotation_from_up(np.array(self.up, dtype=float))
        self._hh  = self.h / 2.

    def support(self, d: np.ndarray) -> np.ndarray:
        """Furthest point of the cylinder in world direction *d*."""
        d_local = self._rot.T @ d

        # Z component: top or bottom cap
        sz = self._hh if d_local[2] >= 0 else -self._hh

        # XY component: point on the rim in the direction of the XY projection
        dxy  = d_local[:2]
        nxy  = np.linalg.norm(dxy)
        sxy  = (dxy / nxy * self.r) if nxy > 1e-12 else np.array([self.r, 0.])

        s_local = np.array([sxy[0], sxy[1], sz])
        return self._pos + self._rot @ s_local


@dataclass
class Sphere:
    """
    Sphere (orientation is irrelevant; 'up' is accepted for API symmetry).

    Parameters
    ----------
    r       : radius
    position: world-space centre
    up      : ignored (sphere is rotationally symmetric)
    """
    r: float
    position: Tuple[float, float, float] = (0., 0., 0.)
    up: Tuple[float, float, float]       = (0., 0., 1.)

    def __post_init__(self) -> None:
        self._pos = np.array(self.position, dtype=float)

    def support(self, d: np.ndarray) -> np.ndarray:
        """Furthest point of the sphere in world direction *d*."""
        n = np.linalg.norm(d)
        if n < 1e-12:
            return self._pos + np.array([self.r, 0., 0.])
        return self._pos + (d / n) * self.r


# ─────────────────────────────────────────────────────────────────────────────
# Internal: GJK algorithm
# ─────────────────────────────────────────────────────────────────────────────

def _cso_support(a, b, d: np.ndarray) -> np.ndarray:
    """Support point of the Minkowski difference A ⊖ B."""
    return a.support(d) - b.support(-d)


# ── Nearest-simplex sub-routines ──────────────────────────────────────────────
# Convention: the *last* element of `pts` is always the newest point A.
# Each function returns (new_pts, new_direction).
# The tetrahedron case additionally returns (pts, None) when origin is enclosed.

def _line(pts):
    """Process a 2-point simplex [B, A], A = pts[-1]."""
    B, A = pts[0], pts[1]
    AB = B - A
    AO = -A
    if np.dot(AB, AO) > 0:
        # Origin projects onto segment AB → search perpendicular to AB toward origin
        d = np.cross(np.cross(AB, AO), AB)
        if np.linalg.norm(d) < 1e-12:
            # AO is parallel to AB; pick any perpendicular
            d = np.array([-AB[1] - AB[2], AB[0] - AB[2], AB[0] + AB[1]])
        return [B, A], d
    else:
        # Origin is "behind" A; simplex collapses to just A
        return [A], AO


def _triangle(pts):
    """Process a 3-point simplex [C, B, A], A = pts[-1]."""
    C, B, A = pts[0], pts[1], pts[2]
    AB = B - A
    AC = C - A
    AO = -A
    ABC = np.cross(AB, AC)      # triangle normal

    # Is origin outside edge AC?
    if np.dot(np.cross(ABC, AC), AO) > 0:
        if np.dot(AC, AO) > 0:
            return [C, A], np.cross(np.cross(AC, AO), AC)
        else:
            return _line([B, A])
    # Is origin outside edge AB?
    elif np.dot(np.cross(AB, ABC), AO) > 0:
        return _line([B, A])
    # Origin is above or below the triangle face
    else:
        if np.dot(ABC, AO) > 0:
            return [C, B, A], ABC
        else:
            # Flip winding so normal faces origin
            return [B, C, A], -ABC


def _tetrahedron(pts):
    """
    Process a 4-point simplex [D, C, B, A], A = pts[-1].
    Returns (pts, None) when origin is enclosed → intersection found.
    """
    D, C, B, A = pts[0], pts[1], pts[2], pts[3]
    AB = B - A
    AC = C - A
    AD = D - A
    AO = -A

    ABC = np.cross(AB, AC)
    ACD = np.cross(AC, AD)
    ADB = np.cross(AD, AB)

    # Ensure each face normal points *outward* (away from the opposite vertex)
    if np.dot(ABC, AD) > 0: ABC = -ABC
    if np.dot(ACD, AB) > 0: ACD = -ACD
    if np.dot(ADB, AC) > 0: ADB = -ADB

    if np.dot(ABC, AO) > 0:
        return _triangle([C, B, A])         # origin outside face ABC
    if np.dot(ACD, AO) > 0:
        return _triangle([D, C, A])         # origin outside face ACD
    if np.dot(ADB, AO) > 0:
        return _triangle([B, D, A])         # origin outside face ADB

    return pts, None                        # origin inside tetrahedron ✓


def _nearest_simplex(pts):
    n = len(pts)
    if n == 2:
        return _line(pts)
    elif n == 3:
        return _triangle(pts)
    else:
        return _tetrahedron(pts)


# ── Main GJK loop ─────────────────────────────────────────────────────────────

def _gjk(shape_a, shape_b, max_iters: int = 64) -> bool:
    """
    Returns True if the convex shapes *shape_a* and *shape_b* overlap,
    using the Gilbert–Johnson–Keerthi distance algorithm.
    """
    # Initial search direction: vector between centres
    d = shape_a._pos - shape_b._pos
    if np.linalg.norm(d) < 1e-10:
        d = np.array([1., 0., 0.])

    # First support point
    A = _cso_support(shape_a, shape_b, d)
    if np.dot(A, d) < 0:
        return False                # separating direction found immediately

    simplex = [A]
    d = -A                          # search toward the origin

    for _ in range(max_iters):
        if np.linalg.norm(d) < 1e-10:
            return True             # simplex is at / contains origin

        A = _cso_support(shape_a, shape_b, d)
        if np.dot(A, d) < 0:
            return False            # new point didn't reach origin → separated

        simplex.append(A)
        simplex, d = _nearest_simplex(simplex)

        if d is None:               # origin enclosed in tetrahedron
            return True

    return True                     # did not diverge → treat as intersecting


# ─────────────────────────────────────────────────────────────────────────────
# Public API
# ─────────────────────────────────────────────────────────────────────────────

def intersects(shape_a, shape_b) -> bool:
    """
    Return True if *shape_a* and *shape_b* intersect (or touch).

    Parameters
    ----------
    shape_a, shape_b : Box | Cylinder | Sphere

    Returns
    -------
    bool

    Examples
    --------
    >>> intersects(Box(2, 2, 2), Sphere(0.5, position=(0.9, 0, 0)))
    True
    >>> intersects(Box(2, 2, 2), Sphere(0.5, position=(1.6, 0, 0)))
    False
    """
    return _gjk(shape_a, shape_b)


# ─────────────────────────────────────────────────────────────────────────────
# Quick self-test
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    def check(label, result, expected):
        ok = result == expected
        status = "✅" if ok else "❌"
        print(f"  {status}  {label:55s}  got={result}  expected={expected}")
        return ok

    all_ok = True
    print("\n── Box vs Box ──────────────────────────────────────────────────")
    all_ok &= check("overlapping boxes",
        intersects(Box(2, 2, 2), Box(2, 2, 2, position=(1, 0, 0))), True)
    all_ok &= check("touching boxes (edge)",
        intersects(Box(2, 2, 2), Box(2, 2, 2, position=(2, 0, 0))), True)
    all_ok &= check("separate boxes",
        intersects(Box(2, 2, 2), Box(2, 2, 2, position=(2.01, 0, 0))), False)
    all_ok &= check("tilted box overlapping",
        intersects(
            Box(2, 2, 2),
            Box(2, 2, 2, position=(1, 0, 0), up=(0, 1, 1))), True)

    print("\n── Sphere vs Sphere ────────────────────────────────────────────")
    all_ok &= check("overlapping spheres",
        intersects(Sphere(1), Sphere(1, position=(1.5, 0, 0))), True)
    all_ok &= check("touching spheres",
        intersects(Sphere(1), Sphere(1, position=(2, 0, 0))), True)
    all_ok &= check("separate spheres",
        intersects(Sphere(1), Sphere(1, position=(2.01, 0, 0))), False)

    print("\n── Box vs Sphere ───────────────────────────────────────────────")
    all_ok &= check("sphere inside box",
        intersects(Box(4, 4, 4), Sphere(0.5, position=(0, 0, 0))), True)
    all_ok &= check("sphere touching box face",
        intersects(Box(2, 2, 2), Sphere(1, position=(2, 0, 0))), True)
    all_ok &= check("sphere just outside box",
        intersects(Box(2, 2, 2), Sphere(0.9, position=(2, 0, 0))), False)
    all_ok &= check("sphere clipping box corner  (dist≈0.35 < r=0.5)",
        intersects(Box(2, 2, 2), Sphere(0.5, position=(1.2, 1.2, 1.2))), True)
    all_ok &= check("sphere missing box corner   (dist≈0.35 > r=0.3)",
        intersects(Box(2, 2, 2), Sphere(0.3, position=(1.2, 1.2, 1.2))), False)

    print("\n── Cylinder vs Sphere ──────────────────────────────────────────")
    all_ok &= check("sphere overlapping cylinder side",
        intersects(Cylinder(1, 4), Sphere(0.5, position=(1.2, 0, 0))), True)
    all_ok &= check("sphere above cylinder cap",
        intersects(Cylinder(1, 4), Sphere(0.5, position=(0, 0, 2.4))), True)
    all_ok &= check("sphere clear of cylinder",
        intersects(Cylinder(1, 4), Sphere(0.5, position=(0, 0, 3))), False)
    all_ok &= check("horizontal cylinder touching sphere (dist=0.5 < r=0.6)",
        intersects(
            Cylinder(0.5, 4, position=(0, 0, 0), up=(1, 0, 0)),
            Sphere(0.6, position=(2.5, 0, 0))), True)
    all_ok &= check("horizontal cylinder missing sphere  (dist=0.9 > r=0.6)",
        intersects(
            Cylinder(0.5, 4, position=(0, 0, 0), up=(1, 0, 0)),
            Sphere(0.6, position=(2.9, 0, 0))), False)

    print("\n── Cylinder vs Box ─────────────────────────────────────────────")
    all_ok &= check("cylinder inside box",
        intersects(Box(4, 4, 4), Cylinder(0.5, 2)), True)
    all_ok &= check("cylinder just outside box",
        intersects(Box(2, 2, 2), Cylinder(0.5, 2, position=(2, 0, 0))), False)
    all_ok &= check("tilted cylinder clipping box corner",
        intersects(
            Box(2, 2, 2),
            Cylinder(0.3, 3, position=(1.1, 0, 0), up=(0, 1, 0))), True)

    print("\n── Cylinder vs Cylinder ────────────────────────────────────────")
    all_ok &= check("parallel cylinders overlapping",
        intersects(Cylinder(1, 4), Cylinder(1, 4, position=(1.5, 0, 0))), True)
    all_ok &= check("parallel cylinders separate",
        intersects(Cylinder(1, 4), Cylinder(1, 4, position=(2.1, 0, 0))), False)
    all_ok &= check("crossed cylinders (X vs Z axis)",
        intersects(
            Cylinder(0.4, 4, up=(1, 0, 0)),
            Cylinder(0.4, 4, up=(0, 0, 1))), True)

    print()
    if all_ok:
        print("All tests passed ✅")
    else:
        print("Some tests FAILED ❌")
        sys.exit(1)
