"""Intersection rayon / triangles — maths pures NumPy, sans Qt ni OpenGL.
Algorithme Möller-Trumbore vectorisé, double face (pas de back-face culling).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RayHit:
    """Résultat d'un ray-cast. distance = t le long du rayon (mm)."""

    point_world: np.ndarray  # shape (3,)
    distance: float          # t > 0
    triangle_index: int


def ray_aabb_hit(
    origin: np.ndarray,
    direction: np.ndarray,
    aabb_min: np.ndarray,
    aabb_max: np.ndarray,
    *,
    t_max: float = float("inf"),
) -> bool:
    """Test de slab AABB. Retourne True si le rayon traverse la boîte englobante."""
    with np.errstate(divide="ignore", invalid="ignore"):
        inv_dir = np.where(np.abs(direction) > 1e-12, 1.0 / direction, np.inf)
    t1 = (aabb_min - origin) * inv_dir
    t2 = (aabb_max - origin) * inv_dir
    t_near = float(np.max(np.minimum(t1, t2)))
    t_far = float(np.min(np.maximum(t1, t2)))
    return t_far >= max(t_near, 0.0) and t_near <= t_max


def ray_triangles_intersect(
    origin_world: np.ndarray,     # (3,)
    direction_world: np.ndarray,  # (3,) normalisé
    triangles_world: np.ndarray,  # (T, 3, 3) : T triangles, 3 sommets, xyz
    *,
    epsilon: float = 1e-7,
) -> RayHit | None:
    """Möller-Trumbore vectorisé, double face.
    Retourne le hit le plus proche avec t > epsilon, ou None."""
    if triangles_world.size == 0:
        return None

    v0 = triangles_world[:, 0, :]   # (T, 3)
    v1 = triangles_world[:, 1, :]
    v2 = triangles_world[:, 2, :]
    edge1 = v1 - v0                  # (T, 3)
    edge2 = v2 - v0                  # (T, 3)

    h = np.cross(direction_world, edge2)           # (T, 3)
    a = np.einsum("ij,ij->i", edge1, h)            # (T,)
    valid = np.abs(a) > epsilon
    f = np.zeros_like(a)
    f[valid] = 1.0 / a[valid]

    s = origin_world - v0                          # (T, 3)
    u = f * np.einsum("ij,ij->i", s, h)           # (T,)

    q = np.cross(s, edge1)                         # (T, 3)
    v = f * (q @ direction_world)                  # (T,)

    t = f * np.einsum("ij,ij->i", edge2, q)       # (T,)

    hit = valid & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (t > epsilon)
    if not np.any(hit):
        return None

    t_masked = np.where(hit, t, np.inf)
    idx = int(np.argmin(t_masked))
    t_hit = float(t_masked[idx])
    point = origin_world + t_hit * direction_world
    return RayHit(point_world=point, distance=t_hit, triangle_index=idx)
