from __future__ import annotations

from enum import Enum


class TrajectoryBezierDegree(Enum):
    BEZIER3 = "BEZIER3"
    BEZIER5 = "BEZIER5"

    @staticmethod
    def from_value(value: object, default: "TrajectoryBezierDegree" | None = None) -> "TrajectoryBezierDegree":
        fallback = TrajectoryBezierDegree.BEZIER5 if default is None else default
        if isinstance(value, TrajectoryBezierDegree):
            return value
        try:
            return TrajectoryBezierDegree(str(value))
        except (TypeError, ValueError):
            return fallback
