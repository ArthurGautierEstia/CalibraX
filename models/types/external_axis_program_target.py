from __future__ import annotations

from dataclasses import dataclass

from models.types.external_axis_joint_type import ExternalAxisJointType


@dataclass(frozen=True)
class ExternalAxisJointValue:
    axis_id: str
    joint_index: int
    value: float  # mm si LINEAR, degrés si ROTARY
    joint_type: ExternalAxisJointType


@dataclass(frozen=True)
class ExternalAxisProgramTarget:
    # Taille variable (dépend du nombre d'axes/joints configurés) → list acceptable
    values: tuple[ExternalAxisJointValue, ...]

    @classmethod
    def from_list(cls, values: list[ExternalAxisJointValue]) -> ExternalAxisProgramTarget:
        return cls(values=tuple(values))
