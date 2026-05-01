from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from models.types.pose6 import Pose6


@dataclass(slots=True)
class FkResult:
    dh_matrices: list[np.ndarray]
    corrected_matrices: list[np.ndarray]
    dh_pose: Pose6
    corrected_pose: Pose6
    deviation: Pose6


@dataclass(slots=True)
class TrajectorySampleKinematics:
    dh_pose: Pose6
    corrected_matrices: list[np.ndarray]

    @classmethod
    def from_fk_result(cls, fk_result: FkResult) -> "TrajectorySampleKinematics":
        return cls(
            dh_pose=fk_result.dh_pose.copy(),
            corrected_matrices=list(fk_result.corrected_matrices),
        )
