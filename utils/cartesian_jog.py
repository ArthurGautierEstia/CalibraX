from __future__ import annotations

import numpy as np

import utils.math_utils as math_utils
from models.reference_frame import ReferenceFrame
from models.types import Pose6
from utils.reference_frame_utils import FrameTransform


def _rotation_about_axis_deg(axis_index: int, delta_deg: float) -> np.ndarray:
    """Rotation 3x3 autour de l'axe d'orientation demandé (convention Kuka ZYX).

    axis_index : 3 -> A (rot Z), 4 -> B (rot Y), 5 -> C (rot X).
    """
    if axis_index == 3:
        return math_utils.rot_z(delta_deg)
    if axis_index == 4:
        return math_utils.rot_y(delta_deg)
    return math_utils.rot_x(delta_deg)


def _require_frame(robot_base_transform: FrameTransform | Pose6 | None) -> FrameTransform:
    if robot_base_transform is None:
        raise ValueError("robot_base_transform est requis pour le repère WORLD")
    if isinstance(robot_base_transform, FrameTransform):
        return robot_base_transform
    return FrameTransform.from_pose(robot_base_transform)


def compute_cartesian_jog_target(
    tcp_pose_base: Pose6,
    reference_frame: ReferenceFrame,
    axis_index: int,
    delta: float,
    robot_base_transform: FrameTransform | Pose6 | None = None,
) -> Pose6:
    """Calcule la pose TCP cible (en repère base robot) pour un pas de jog cartésien.

    L'orientation cible est obtenue par **composition** d'une rotation autour de
    l'axe demandé (on pilote le *sens* de rotation), jamais par écrasement d'une
    composante Euler de la pose courante. Le jog reste donc continu à la traversée
    d'une singularité de représentation (gimbal lock B = ±90°) : l'outil continue
    de tourner dans le même sens même si l'affichage A/B/C se replie.

    Sémantique des repères :
        - TOOL  : rotation autour des axes de l'outil  -> R_tcp @ Rdelta (post-mult).
        - ROBOT : rotation autour des axes de la base    -> Rdelta @ R_tcp (pré-mult).
        - WORLD : rotation autour des axes du monde, exprimée dans la base via la
                  transformation base->monde.

    Args:
        tcp_pose_base: pose TCP courante en repère base robot (mm / deg).
        reference_frame: repère du jog (ROBOT, WORLD ou TOOL ; PROGRAM traité comme ROBOT).
        axis_index: 0=X, 1=Y, 2=Z (translation) ; 3=A, 4=B, 5=C (rotation).
        delta: incrément (mm pour translation, deg pour rotation).
        robot_base_transform: pose de la base robot dans le monde (requis si WORLD).

    Returns:
        Pose6 cible exprimée en repère base robot.
    """
    r_tcp_base = math_utils.euler_to_rotation_matrix(
        tcp_pose_base.a, tcp_pose_base.b, tcp_pose_base.c, degrees=True
    )
    x, y, z = tcp_pose_base.x, tcp_pose_base.y, tcp_pose_base.z
    is_rotation = axis_index >= 3

    if reference_frame == ReferenceFrame.TOOL:
        if is_rotation:
            r_new = r_tcp_base @ _rotation_about_axis_deg(axis_index, delta)
        else:
            delta_tool = np.zeros(3, dtype=float)
            delta_tool[axis_index] = delta
            delta_base = r_tcp_base @ delta_tool
            x += float(delta_base[0])
            y += float(delta_base[1])
            z += float(delta_base[2])
            r_new = r_tcp_base

    elif reference_frame == ReferenceFrame.WORLD:
        frame = _require_frame(robot_base_transform)
        if is_rotation:
            r_delta_base = (
                frame.inverse_rotation
                @ _rotation_about_axis_deg(axis_index, delta)
                @ frame.rotation
            )
            r_new = r_delta_base @ r_tcp_base
        else:
            delta_world = np.zeros(3, dtype=float)
            delta_world[axis_index] = delta
            delta_base = frame.inverse_rotation @ delta_world
            x += float(delta_base[0])
            y += float(delta_base[1])
            z += float(delta_base[2])
            r_new = r_tcp_base

    else:  # ROBOT (base) / PROGRAM
        if is_rotation:
            r_new = _rotation_about_axis_deg(axis_index, delta) @ r_tcp_base
        else:
            if axis_index == 0:
                x += delta
            elif axis_index == 1:
                y += delta
            else:
                z += delta
            r_new = r_tcp_base

    abc = math_utils.rotation_matrix_to_euler_zyx(r_new)
    return Pose6(x, y, z, float(abc[0]), float(abc[1]), float(abc[2]))
