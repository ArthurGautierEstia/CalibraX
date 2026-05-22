"""
Calcul des couples articulaires et déformations TCP induits par les efforts de coupe.

Références :
    Caron S., Joint torques and Jacobian transpose, https://scaron.info/blog/...
    Dumas C. et al., Joint Stiffness Identification of a Heavy Kuka Robot, Autonomous Robots, 2014.

Convention d'unités interne (SI) :
    - Positions : m (le Jacobien brut du projet est en mm/rad — conversion appliquée à l'entrée)
    - Angles    : rad
    - Forces    : N
    - Couples   : N·m
    - Raideurs  : N·m/rad

La déviation TCP est restituée en mm à la sortie (reconversion m → mm).

Aucune dépendance PyQt — module pur calcul, transposable en C++.
"""

from __future__ import annotations

import numpy as np

from utils.mgi_jacobien import compute_jacobian_numeric


def compute_tcp_jacobian(robot_model, q_deg: list[float], tool,
                         epsilon_rad: float = 1e-6) -> np.ndarray:
    """Retourne le Jacobien numérique 6×6 en unités SI (m/rad ; rad/rad).

    Le Jacobien brut du projet est calculé en mm/rad pour les 3 premières lignes.
    On divise J[0:3, :] par 1000 pour travailler en m/rad dans ce module.

    Args:
        robot_model: Instance RobotModel.
        q_deg:       Configuration articulaire (degrés), longueur 6.
        tool:        Instance RobotTool ou None.
        epsilon_rad: Pas de différentiation (rad).

    Returns:
        Jacobien J (6×6), lignes 0-2 en m/rad, lignes 3-5 en rad/rad.
    """
    J = compute_jacobian_numeric(q_deg, robot_model, epsilon_rad, tool)
    assert J.shape == (6, 6), f"Jacobien inattendu : shape={J.shape}"
    J_si = J.copy()
    J_si[:3, :] /= 1000.0  # mm/rad → m/rad
    return J_si


def force_tool_to_base(force_tool_N: tuple[float, float, float],
                       R_tool_to_base: np.ndarray) -> np.ndarray:
    """Exprime les efforts de coupe (repère outil) dans le repère base robot.

    Args:
        force_tool_N:    (F_t, F_r, F_a) en N dans le repère outil.
        R_tool_to_base:  Matrice de rotation 3×3 repère outil → repère base.

    Returns:
        Vecteur force 3D dans le repère base (N), shape (3,).
    """
    f = np.array(force_tool_N, dtype=float)
    return R_tool_to_base @ f


def compute_joint_torques_from_force(jacobian_si: np.ndarray,
                                     force_base_N: np.ndarray,
                                     moment_base_Nm: np.ndarray | None = None) -> np.ndarray:
    """Calcule les couples articulaires par la transposée du Jacobien.

    τ = Jᵀ · [F; M]  avec F en N, M en N·m, J en (m/rad ; rad/rad).
    → τ en N·m.

    Args:
        jacobian_si:    Jacobien 6×6 en unités SI (m/rad ; rad/rad).
        force_base_N:   Effort de coupe au TCP dans le repère base (N), shape (3,).
        moment_base_Nm: Moment au TCP dans le repère base (N·m), shape (3,). None → zéro (v1).

    Returns:
        Vecteur de couples articulaires (N·m), shape (6,).
    """
    if moment_base_Nm is None:
        moment_base_Nm = np.zeros(3)
    wrench = np.concatenate([force_base_N, moment_base_Nm])  # shape (6,)
    return jacobian_si.T @ wrench


def compute_joint_deflections(torque_Nm: np.ndarray,
                              stiffness_Nm_per_rad: list[float]) -> np.ndarray:
    """Calcule les déformations articulaires élastiques.

    δθ_i = τ_i / k_i

    Args:
        torque_Nm:           Couples articulaires (N·m), shape (6,).
        stiffness_Nm_per_rad: Raideurs k_i (N·m/rad), longueur 6.

    Returns:
        Déformations δθ (rad), shape (6,).
    """
    k = np.array(stiffness_Nm_per_rad, dtype=float)
    return torque_Nm / k


def compute_tcp_deviation(jacobian_si: np.ndarray,
                          delta_theta_rad: np.ndarray) -> tuple[np.ndarray, float]:
    """Calcule la déviation TCP induite par les déformations articulaires.

    δx_TCP = J · δθ  (6 composantes en unités SI).
    La norme de position est restituée en mm.

    Args:
        jacobian_si:    Jacobien 6×6 en unités SI (m/rad ; rad/rad).
        delta_theta_rad: Déformations articulaires (rad), shape (6,).

    Returns:
        (delta_6, norm_pos_mm) :
            delta_6      — déviation TCP complète (6,) [m pour pos, rad pour ori]
            norm_pos_mm  — ‖δx_TCP[0:3]‖ en mm.
    """
    delta_6 = jacobian_si @ delta_theta_rad
    norm_pos_mm = float(np.linalg.norm(delta_6[:3]) * 1000.0)  # m → mm
    return delta_6, norm_pos_mm


def gravity_torques_placeholder(q_deg: list[float]) -> np.ndarray:
    """v1 : couples gravitaires nuls. Prépare l'ajout du modèle dynamique en v2.

    Args:
        q_deg: Configuration articulaire (degrés), longueur 6. Non utilisé en v1.

    Returns:
        Vecteur nul (N·m), shape (6,).
    """
    return np.zeros(6)
