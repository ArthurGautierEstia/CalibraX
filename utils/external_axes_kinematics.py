"""Utilitaires cinématiques pour les axes externes.

Tous les calculs produisent des matrices homogènes NumPy 4×4.
Les longueurs sont en mm, les angles en degrés (stockage) / radians (calcul interne).
"""
from __future__ import annotations

import numpy as np

from models.external_axes_model import ExternalAxesModel
from models.types.pose6 import Pose6
from utils.math_utils import invert_homogeneous_transform, pose_zyx_to_matrix


def get_effective_robot_base_in_world(workspace_model, external_axes_model: ExternalAxesModel) -> np.ndarray:
    """Retourne la matrice 4x4 de la base robot dans le repère monde.

    Honore l'override d'axe externe (rail qui porte le robot) si présent ;
    sinon retombe sur la pose nominale du workspace.
    """
    override = external_axes_model.get_robot_world_base_matrix()
    if override is not None:
        return np.array(override, dtype=float)
    return np.array(workspace_model.get_robot_base_transform_world().matrix, dtype=float)


def world_robot_base(
    external_axes_model: ExternalAxesModel,
    workspace_robot_base_matrix: np.ndarray,
) -> np.ndarray:
    """Retourne T_world_robotBase.

    - Si aucun axe externe ne porte le robot : retourne la pose workspace inchangée.
    - Sinon : retourne T_world_axisEnd (extrémité de l'axe porteur).
    """
    override = external_axes_model.get_robot_world_base_matrix()
    if override is not None:
        return override
    return workspace_robot_base_matrix.copy()


def world_workpiece(
    piece_mount_parent_id: str | None,
    world_transforms: dict[str, dict],
    piece_local_pose: Pose6,
    workspace_robot_base_matrix: np.ndarray,
) -> np.ndarray:
    """Retourne T_world_workpiece.

    - Si piece_mount_parent_id est None : pose locale dans le monde (workspace normal).
    - Sinon : T_world_axisEnd · T_local_piece.
    """
    T_local = pose_zyx_to_matrix(piece_local_pose)
    if piece_mount_parent_id is None:
        return workspace_robot_base_matrix @ T_local
    if piece_mount_parent_id in world_transforms:
        return world_transforms[piece_mount_parent_id]["end"] @ T_local
    return T_local


def compute_target_in_robot_base(
    target_world_pose: Pose6,
    external_axes_model: ExternalAxesModel,
    workspace_robot_base_matrix: np.ndarray,
) -> Pose6:
    """Convertit une pose TCP exprimée dans le monde en repère base robot.

    Utilisé par le solveur en mode « axes positionnés » pour appeler le MGI analytique
    existant (qui travaille dans le repère base robot) sans modifier utils/mgi.py.
    """
    from utils.math_utils import matrix_to_pose_zyx

    T_world_robotBase = world_robot_base(external_axes_model, workspace_robot_base_matrix)
    T_robotBase_world = invert_homogeneous_transform(T_world_robotBase)
    T_target_world = pose_zyx_to_matrix(target_world_pose)
    T_target_in_base = T_robotBase_world @ T_target_world
    return matrix_to_pose_zyx(T_target_in_base)
