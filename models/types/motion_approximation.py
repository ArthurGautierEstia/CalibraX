from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ApproximationMode(Enum):
    NONE = "NONE"
    C_DIS = "C_DIS"
    C_VEL = "C_VEL"


@dataclass(frozen=True)
class MotionApproximation:
    mode: ApproximationMode
    value: float  # mm pour C_DIS, % pour C_VEL

    @classmethod
    def none(cls) -> MotionApproximation:
        return cls(mode=ApproximationMode.NONE, value=0.0)
