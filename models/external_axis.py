from __future__ import annotations

import uuid

import numpy as np

from models.external_axis_joint import ExternalAxisJoint
from models.types.external_axis_joint_type import ExternalAxisJointType
from models.types.external_axis_mount_mode import ExternalAxisMountMode
from models.types.pose6 import Pose6
from utils.math_utils import pose_zyx_to_matrix


class ExternalAxis:
    """Un système d'axe(s) externe(s) à N degrés de liberté (N+1 éléments CAO).

    Chaîne cinématique:
        T_world_end = T_world_parent_end
                    · base_pose_in_parent
                    · axis_frame_in_base
                    · ∏_{i=0..N-1} [ joint_i.link_pose_matrix · joint_i.joint_transform() ]
    """

    def __init__(
        self,
        name: str = "Axe externe",
        axis_id: str | None = None,
        mount_parent_id: str | None = None,
        mount_mode: ExternalAxisMountMode = ExternalAxisMountMode.POSITIONED,
        base_cad_model: str = "",
        base_cad_color: tuple[float, float, float, float] = (0.3, 0.3, 0.35, 1.0),
        base_pose_in_parent: Pose6 | None = None,
        axis_frame_in_base: Pose6 | None = None,
        joints: list[ExternalAxisJoint] | None = None,
    ) -> None:
        self.id: str = str(axis_id) if axis_id else str(uuid.uuid4())
        self.name: str = str(name).strip() or "Axe externe"
        self.mount_parent_id: str | None = mount_parent_id
        self.mount_mode: ExternalAxisMountMode = ExternalAxisMountMode(mount_mode)
        self.base_cad_model: str = str(base_cad_model)
        self.base_cad_color: tuple[float, float, float, float] = tuple(float(c) for c in base_cad_color)
        self.base_pose_in_parent: Pose6 = (base_pose_in_parent or Pose6.zeros()).copy()
        self.axis_frame_in_base: Pose6 = (axis_frame_in_base or Pose6.zeros()).copy()
        self.joints: list[ExternalAxisJoint] = [j.copy() for j in (joints or [])]

    # ------------------------------------------------------------------
    # Kinematic computation
    # ------------------------------------------------------------------

    def compute_chain(self, world_parent_transform: np.ndarray) -> dict:
        """Calcule les matrices monde pour chaque lien (valeurs live des joints)."""
        return self.compute_chain_with_values(world_parent_transform, [j.value for j in self.joints])

    def compute_chain_with_values(self, world_parent_transform: np.ndarray, joint_values: list[float]) -> dict:
        """Calcule les matrices monde pour des valeurs articulaires explicites.

        Returns:
            {
                "base": T_world_base (4×4),
                "joint_links": [T_world_link_i, ...],   # len = N
                "end": T_world_end (4×4),
            }
        """
        T_world_base = world_parent_transform @ pose_zyx_to_matrix(self.base_pose_in_parent)
        T = T_world_base @ pose_zyx_to_matrix(self.axis_frame_in_base)

        joint_matrices: list[np.ndarray] = []
        for i, joint in enumerate(self.joints):
            q = joint_values[i] if i < len(joint_values) else joint.value
            T = T @ joint.link_pose_matrix() @ joint.joint_transform_for_value(q)
            joint_matrices.append(T.copy())

        T_world_end = T
        return {
            "base": T_world_base,
            "joint_links": joint_matrices,
            "end": T_world_end,
        }

    # ------------------------------------------------------------------
    # Templates
    # ------------------------------------------------------------------

    @classmethod
    def make_linear_rail(cls, name: str = "Rail linéaire X") -> "ExternalAxis":
        joint = ExternalAxisJoint(
            joint_type=ExternalAxisJointType.LINEAR,
            axis=(1.0, 0.0, 0.0),
            link_pose_in_prev=Pose6.zeros(),
            q_min=0.0,
            q_max=2000.0,
        )
        return cls(name=name, joints=[joint])

    @classmethod
    def make_rotary_1axis(cls, name: str = "Positionneur 1 axe") -> "ExternalAxis":
        joint = ExternalAxisJoint(
            joint_type=ExternalAxisJointType.ROTARY,
            axis=(0.0, 0.0, 1.0),
            link_pose_in_prev=Pose6.zeros(),
            q_min=-180.0,
            q_max=180.0,
        )
        return cls(name=name, joints=[joint])

    @classmethod
    def make_rotary_2axis(cls, name: str = "Positionneur 2 axes") -> "ExternalAxis":
        joint_a = ExternalAxisJoint(
            joint_type=ExternalAxisJointType.ROTARY,
            axis=(0.0, 0.0, 1.0),
            link_pose_in_prev=Pose6.zeros(),
            q_min=-180.0,
            q_max=180.0,
        )
        joint_b = ExternalAxisJoint(
            joint_type=ExternalAxisJointType.ROTARY,
            axis=(1.0, 0.0, 0.0),
            link_pose_in_prev=Pose6.zeros(),
            q_min=-120.0,
            q_max=120.0,
        )
        return cls(name=name, joints=[joint_a, joint_b])

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------

    def copy(self) -> "ExternalAxis":
        return ExternalAxis(
            name=self.name,
            axis_id=self.id,
            mount_parent_id=self.mount_parent_id,
            mount_mode=self.mount_mode,
            base_cad_model=self.base_cad_model,
            base_cad_color=self.base_cad_color,
            base_pose_in_parent=self.base_pose_in_parent,
            axis_frame_in_base=self.axis_frame_in_base,
            joints=[j.copy() for j in self.joints],
        )

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "mount_parent_id": self.mount_parent_id,
            "mount_mode": self.mount_mode.value,
            "base_cad_model": self.base_cad_model,
            "base_cad_color": list(self.base_cad_color),
            "base_pose_in_parent": self.base_pose_in_parent.to_list(),
            "axis_frame_in_base": self.axis_frame_in_base.to_list(),
            "joints": [j.to_dict() for j in self.joints],
        }

    @classmethod
    def from_dict(cls, data: dict) -> "ExternalAxis":
        color_raw = data.get("base_cad_color", [0.3, 0.3, 0.35, 1.0])
        return cls(
            name=str(data.get("name", "Axe externe")),
            axis_id=str(data.get("id", "")),
            mount_parent_id=data.get("mount_parent_id"),
            mount_mode=ExternalAxisMountMode(data.get("mount_mode", "positioned")),
            base_cad_model=str(data.get("base_cad_model", "")),
            base_cad_color=tuple(float(c) for c in color_raw),
            base_pose_in_parent=Pose6(*[float(v) for v in data.get("base_pose_in_parent", [0]*6)]),
            axis_frame_in_base=Pose6(*[float(v) for v in data.get("axis_frame_in_base", [0]*6)]),
            joints=[ExternalAxisJoint.from_dict(j) for j in data.get("joints", [])],
        )

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, ExternalAxis):
            return False
        return self.to_dict() == other.to_dict()
