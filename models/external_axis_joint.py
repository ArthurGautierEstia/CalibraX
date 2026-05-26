from __future__ import annotations

import numpy as np

from models.types.external_axis_joint_type import ExternalAxisJointType
from models.types.pose6 import Pose6
from utils.math_utils import (
    normalize3,
    vector_norm3,
    pose_zyx_to_matrix,
    rot_z,
    rot_y,
    rot_x,
)


class ExternalAxisJoint:
    """Un degré de liberté (prismatique ou rotoïde) d'un axe externe.

    La transformation articulaire J(q) est calculée dans le repère du joint :
    - LINEAR : Trans(axis * (value + offset))
    - ROTARY : Rot(axis, value + offset)

    Le repère du joint i est exprimé dans le repère du joint i-1 (ou de la base de l'axe
    externe si i = 0) via link_pose_in_prev, *après* application de J_{i-1}.
    """

    def __init__(
        self,
        joint_type: ExternalAxisJointType = ExternalAxisJointType.LINEAR,
        axis: tuple[float, float, float] = (0.0, 0.0, 1.0),
        link_pose_in_prev: Pose6 | None = None,
        cad_model: str = "",
        cad_color: tuple[float, float, float, float] = (0.25, 0.45, 0.65, 1.0),
        cad_offset_in_joint: Pose6 | None = None,
        q_min: float = -1000.0,
        q_max: float = 1000.0,
        offset: float = 0.0,
        value: float = 0.0,
    ) -> None:
        self.joint_type = ExternalAxisJointType(joint_type)
        ax = normalize3(list(axis))
        self.axis: tuple[float, float, float] = (ax[0], ax[1], ax[2])
        self.link_pose_in_prev: Pose6 = (link_pose_in_prev or Pose6.zeros()).copy()
        self.cad_model: str = str(cad_model)
        self.cad_color: tuple[float, float, float, float] = tuple(float(c) for c in cad_color)
        self.cad_offset_in_joint: Pose6 = (cad_offset_in_joint or Pose6.zeros()).copy()
        self.q_min: float = float(q_min)
        self.q_max: float = float(q_max)
        self.offset: float = float(offset)
        self.value: float = float(np.clip(value, q_min, q_max))

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    def unit(self) -> str:
        return "mm" if self.joint_type == ExternalAxisJointType.LINEAR else "°"

    def joint_transform(self) -> np.ndarray:
        """Matrice 4×4 de la transformation articulaire J(value + offset)."""
        q = self.value + self.offset
        ax, ay, az = self.axis
        T = np.eye(4, dtype=float)
        if self.joint_type == ExternalAxisJointType.LINEAR:
            T[0, 3] = ax * q
            T[1, 3] = ay * q
            T[2, 3] = az * q
        else:
            q_rad = np.radians(q)
            c = float(np.cos(q_rad))
            s = float(np.sin(q_rad))
            n = vector_norm3(self.axis)
            if n < 1e-9:
                return T
            ux, uy, uz = ax / n, ay / n, az / n
            T[:3, :3] = np.array([
                [c + ux*ux*(1-c),   ux*uy*(1-c) - uz*s, ux*uz*(1-c) + uy*s],
                [uy*ux*(1-c) + uz*s, c + uy*uy*(1-c),   uy*uz*(1-c) - ux*s],
                [uz*ux*(1-c) - uy*s, uz*uy*(1-c) + ux*s, c + uz*uz*(1-c)],
            ], dtype=float)
        return T

    def link_pose_matrix(self) -> np.ndarray:
        return pose_zyx_to_matrix(self.link_pose_in_prev)

    # ------------------------------------------------------------------
    # Serialization helpers
    # ------------------------------------------------------------------

    def copy(self) -> "ExternalAxisJoint":
        return ExternalAxisJoint(
            joint_type=self.joint_type,
            axis=self.axis,
            link_pose_in_prev=self.link_pose_in_prev,
            cad_model=self.cad_model,
            cad_color=self.cad_color,
            cad_offset_in_joint=self.cad_offset_in_joint,
            q_min=self.q_min,
            q_max=self.q_max,
            offset=self.offset,
            value=self.value,
        )

    def to_dict(self) -> dict:
        return {
            "joint_type": self.joint_type.value,
            "axis": list(self.axis),
            "link_pose_in_prev": self.link_pose_in_prev.to_list(),
            "cad_model": self.cad_model,
            "cad_color": list(self.cad_color),
            "cad_offset_in_joint": self.cad_offset_in_joint.to_list(),
            "q_min": self.q_min,
            "q_max": self.q_max,
            "offset": self.offset,
            "value": self.value,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ExternalAxisJoint":
        axis_raw = data.get("axis", [0.0, 0.0, 1.0])
        color_raw = data.get("cad_color", [0.25, 0.45, 0.65, 1.0])
        return cls(
            joint_type=ExternalAxisJointType(data.get("joint_type", "linear")),
            axis=(float(axis_raw[0]), float(axis_raw[1]), float(axis_raw[2])),
            link_pose_in_prev=Pose6(*[float(v) for v in data.get("link_pose_in_prev", [0]*6)]),
            cad_model=str(data.get("cad_model", "")),
            cad_color=tuple(float(c) for c in color_raw),
            cad_offset_in_joint=Pose6(*[float(v) for v in data.get("cad_offset_in_joint", [0]*6)]),
            q_min=float(data.get("q_min", -1000.0)),
            q_max=float(data.get("q_max", 1000.0)),
            offset=float(data.get("offset", 0.0)),
            value=float(data.get("value", 0.0)),
        )

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, ExternalAxisJoint):
            return False
        return self.to_dict() == other.to_dict()
