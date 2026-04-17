from __future__ import annotations

from enum import Enum


class TrajectoryBezierDegree(Enum):
    BEZIER3 = "BEZIER3"
    BEZIER5 = "BEZIER5"

    @staticmethod
    def from_value(value: object) -> "TrajectoryBezierDegree":
        if isinstance(value, TrajectoryBezierDegree):
            return value
        return TrajectoryBezierDegree(str(value))
