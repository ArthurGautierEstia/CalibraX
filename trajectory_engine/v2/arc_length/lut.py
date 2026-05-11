from __future__ import annotations

import bisect

from models.types import XYZ3
from trajectory_engine.models import BuildCancelToken
from trajectory_engine.v2.geometry import Bezier7Curve3D
from trajectory_engine.v2.models import ArcLengthLut


def _distance(a: XYZ3, b: XYZ3) -> float:
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return float((dx * dx + dy * dy + dz * dz) ** 0.5)


def _cancelled(cancel_token: BuildCancelToken | None) -> bool:
    return cancel_token is not None and cancel_token.is_cancelled()


def build_arc_length_lut(
    curve: Bezier7Curve3D,
    sample_count: int,
    cancel_token: BuildCancelToken | None = None,
) -> ArcLengthLut:
    count = max(2, int(sample_count))
    parameters_u = [0.0]
    distances_mm = [0.0]
    previous = curve.point(0.0)
    cumulative = 0.0

    for index in range(1, count + 1):
        if _cancelled(cancel_token):
            break
        u = index / count
        point = curve.point(u)
        cumulative += _distance(previous, point)
        parameters_u.append(float(u))
        distances_mm.append(float(cumulative))
        previous = point

    return ArcLengthLut(
        parameters_u=parameters_u,
        distances_mm=distances_mm,
        total_length_mm=float(cumulative),
    )


def parameter_at_distance(lut: ArcLengthLut, distance_mm: float) -> float:
    if not lut.parameters_u or not lut.distances_mm:
        return 0.0
    if len(lut.parameters_u) != len(lut.distances_mm):
        return 0.0
    if distance_mm <= 0.0:
        return 0.0
    if lut.total_length_mm <= 1e-9:
        return 0.0
    if distance_mm >= lut.total_length_mm:
        return 1.0

    index = bisect.bisect_left(lut.distances_mm, float(distance_mm))
    if index <= 0:
        return float(lut.parameters_u[0])
    if index >= len(lut.distances_mm):
        return float(lut.parameters_u[-1])

    s0 = float(lut.distances_mm[index - 1])
    s1 = float(lut.distances_mm[index])
    u0 = float(lut.parameters_u[index - 1])
    u1 = float(lut.parameters_u[index])
    span = s1 - s0
    if abs(span) <= 1e-12:
        return u0
    alpha = (float(distance_mm) - s0) / span
    return u0 + (u1 - u0) * alpha
