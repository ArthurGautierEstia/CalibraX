"""Utilitaires cinématiques pour les axes externes.

Tous les calculs produisent des matrices homogènes NumPy 4×4.
Les longueurs sont en mm, les angles en degrés (stockage) / radians (calcul interne).
"""
from __future__ import annotations

import numpy as np

from models.external_axes_model import ExternalAxesModel
from models.types.pose6 import Pose6
from models.workspace_model import WorkspaceModel
from utils.math_utils import invert_homogeneous_transform, pose_zyx_to_matrix

PREFIX_EXT = "ext:"
PREFIX_WS = "ws:"
FRAME_ROBOT = "robot"


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
        return np.array(override, dtype=float)
    return np.array(workspace_robot_base_matrix, dtype=float)


def get_effective_robot_base_in_world(
    workspace_model: WorkspaceModel,
    external_axes_model: ExternalAxesModel,
) -> np.ndarray:
    """Retourne la matrice 4x4 de la base robot dans le repère monde.

    Honore l'override d'axe externe (rail qui porte le robot) si présent ;
    sinon retombe sur la pose nominale du workspace.
    """
    nominal = np.array(workspace_model.get_robot_base_transform_world().matrix, dtype=float)
    return world_robot_base(external_axes_model, nominal)


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


def piece_frame_world(
    piece_parent_id: str,
    world_transforms: dict[str, dict],
    piece_pose_in_parent: Pose6,
    piece_frame_pose: Pose6,
    workspace_robot_base_matrix: np.ndarray,
    world_robot_base_matrix: np.ndarray,
) -> np.ndarray:
    """Retourne T_world_pieceFrame pour un état d'axes simulé.

    Args:
        piece_parent_id: ID du repère parent (WorkpieceModel.get_parent_frame_id()).
        world_transforms: dict retourné par ExternalAxesModel.compute_world_transforms_for().
        piece_pose_in_parent: pose de la pièce dans son repère parent.
        piece_frame_pose: repère pièce dans la CAO pièce.
        workspace_robot_base_matrix: T_world_robotBase issue du workspace (statique).
        world_robot_base_matrix: T_world_robotBase effectif (avec rail si présent).
    """
    T_pose = pose_zyx_to_matrix(piece_pose_in_parent)
    T_frame = pose_zyx_to_matrix(piece_frame_pose)

    if piece_parent_id == "" or piece_parent_id is None:
        T_parent = np.eye(4, dtype=float)
    elif piece_parent_id == FRAME_ROBOT:
        T_parent = world_robot_base_matrix
    elif piece_parent_id.startswith(PREFIX_EXT):
        axis_id = piece_parent_id[len(PREFIX_EXT):]
        t = world_transforms.get(axis_id)
        T_parent = t["end"] if t is not None else np.eye(4, dtype=float)
    elif piece_parent_id.startswith(PREFIX_WS):
        # Repère workspace statique — non accessible ici sans workspace_model ;
        # le simulateur doit fournir un world_transforms vide pour ce cas.
        T_parent = np.eye(4, dtype=float)
    else:
        T_parent = np.eye(4, dtype=float)

    return T_parent @ T_pose @ T_frame


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
