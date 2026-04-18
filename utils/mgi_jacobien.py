"""
Solveur MGI optimisé par la méthode de la Jacobienne inverse (Levenberg-Marquardt).

Ce solveur affine itérativement les valeurs articulaires fournies par le MGI analytique
en utilisant le MGD corrigé pour tenir compte de TOUTES les corrections de calibration
(translations, rotations, rotations parasites bêta non modélisables par DH standard).

Algorithme Levenberg-Marquardt (Damped Least Squares) :
    1. Partir de q_initial (issu du MGI analytique)
    2. Calculer la pose actuelle via le MGD CORRIGÉ
    3. Calculer l'erreur résiduelle 6D (position + orientation)
    4. Si convergé → retourner
    5. Calculer la Jacobienne numérique (différences finies centrées)
    6. Calculer la correction articulaire : Δq = Jᵀ·(J·Jᵀ + λ²·I)⁻¹·erreur
    7. Mettre à jour q et clamper aux limites articulaires
    8. Répéter depuis 2

L'erreur d'orientation est calculée via le logarithme de la matrice de rotation
relative (vecteur rotation axis×angle), ce qui évite les singularités de type
gimbal lock inhérentes à la différence d'angles d'Euler.
"""

import numpy as np
from dataclasses import dataclass, field

import utils.math_utils as math_utils
from utils.mgi import RobotTool


# ============================================================================
# RÉGION: Paramètres du solveur
# ============================================================================

@dataclass
class MgiJacobienParams:
    """Paramètres configurables du solveur MGI Jacobienne (Levenberg-Marquardt)."""

    max_iterations: int = 20
    """Nombre maximum d'itérations de raffinement."""

    seuil_position: float = 0.001
    """Seuil de convergence en position (mm). Critère : ‖erreur_pos‖ < seuil."""

    seuil_orientation: float = 0.001
    """Seuil de convergence en orientation (degrés). Critère : ‖erreur_ori‖ < seuil."""

    epsilon: float = 1e-6
    """Pas de différentiation pour la Jacobienne numérique (radians)."""

    lambda_damping: float = 0.01
    """Facteur d'amortissement λ (Levenberg-Marquardt). Plus λ est grand, plus
    la correction est amortie — utile près des singularités cinématiques."""


# ============================================================================
# RÉGION: Résultat du solveur
# ============================================================================

@dataclass
class MgiJacobienResultat:
    """Résultat du solveur MGI Jacobienne avec métriques de convergence."""

    joints: list[float] = field(default_factory=lambda: [0.0] * 6)
    """Valeurs articulaires finales (degrés)."""

    joints_analytiques: list[float] | None = None
    """Valeurs articulaires du MGI analytique (point de départ), pour comparaison."""

    converge: bool = False
    """True si le solveur a convergé avant d'atteindre max_iterations."""

    nb_mises_a_jour: int = 0
    """Nombre de mises à jour Jacobienne effectuées (0 = point initial déjà OK)."""

    erreur_position: float = float('inf')
    """Norme de l'erreur de position finale (mm)."""

    erreur_orientation: float = float('inf')
    """Norme de l'erreur d'orientation finale (degrés)."""

    message: str = ""
    """Message informatif sur la convergence."""


def _build_T_cible(x: float, y: float, z: float,
                   a: float, b: float, c: float) -> np.ndarray:
    """
    Construit la matrice homogène 4×4 cible à partir d'une pose [x,y,z,a,b,c].

    Args:
        x, y, z: Position en mm
        a, b, c: Orientation en degrés (angles d'Euler ZYX — convention Kuka)

    Returns:
        Matrice homogène 4×4
    """
    return math_utils.pose_zyx_to_matrix([x, y, z, a, b, c])


def _compute_erreur_pose(T_cible: np.ndarray, T_actuel: np.ndarray) -> np.ndarray:
    """
    Calcule le vecteur d'erreur 6D entre la pose cible et la pose actuelle.

    Composantes :
        [0:3] Erreur de position (mm) : p_cible - p_actuel
        [3:6] Erreur d'orientation (rad) : log(R_cible · R_actuelᵀ)

    L'erreur d'orientation est calculée via le logarithme de la matrice de
    rotation relative, ce qui évite les singularités des angles d'Euler
    (gimbal lock). Le vecteur résultant est dans l'espace de la tâche.

    Args:
        T_cible:  Matrice homogène cible 4×4
        T_actuel: Matrice homogène courante 4×4

    Returns:
        Vecteur d'erreur 6D [dx, dy, dz, wx, wy, wz]
        - dx, dy, dz en mm
        - wx, wy, wz en radians (axis-angle)
    """
    # --- Erreur de position ---
    erreur_pos = T_cible[:3, 3] - T_actuel[:3, 3]

    # --- Erreur d'orientation via rotation relative ---
    # R_err = R_cible · R_actuelᵀ  →  rotation qui amène R_actuel vers R_cible
    R_cible = T_cible[:3, :3]
    R_actuel = T_actuel[:3, :3]
    R_err = R_cible @ R_actuel.T
    erreur_ori = math_utils.rotation_matrix_to_rotation_vector(R_err)  # radians

    return np.concatenate([erreur_pos, erreur_ori])


def _compute_jacobienne_numerique(q_deg: list[float],
                                  robot_model,
                                  epsilon_rad: float,
                                  tool: RobotTool | None = None) -> np.ndarray:
    """
    Calcule la Jacobienne numérique 6×6 par différences finies centrées.

    Pour chaque joint j ∈ {0..5} :
        colonne j = [pose(q + ε·eⱼ) − pose(q − ε·eⱼ)] / (2ε)

    La variation de pose est calculée de façon cohérente avec _compute_erreur_pose :
    - Position : différence euclidienne (mm/rad)
    - Orientation : vecteur rotation de T_minus vers T_plus (rad/rad)

    Le MGD utilisé est le MGD CORRIGÉ (incluant toutes les corrections de
    calibration), ce qui est la clé de l'amélioration par rapport au MGI analytique.

    Args:
        q_deg:       Valeurs articulaires courantes en degrés
        robot_model: Instance RobotModel (fournit compute_fk_joints)
        epsilon_rad: Pas de différentiation en radians

    Returns:
        Jacobienne numérique J (6×6), unités : [mm/rad, mm/rad, ..., rad/rad, ...]
    """
    J = np.zeros((6, 6))
    epsilon_deg = np.degrees(epsilon_rad)

    for j in range(6):
        # --- Perturbation positive ---
        q_plus = list(q_deg)
        q_plus[j] += epsilon_deg

        # --- Perturbation négative ---
        q_minus = list(q_deg)
        q_minus[j] -= epsilon_deg

        # --- MGD corrigé pour les deux perturbations ---
        _, corrected_plus, _, _, _ = robot_model.compute_fk_joints(q_plus, tool=tool)
        _, corrected_minus, _, _, _ = robot_model.compute_fk_joints(q_minus, tool=tool)

        # Matrice TCP corrigée (dernier élément = flange + outil)
        T_plus = corrected_plus[-1]
        T_minus = corrected_minus[-1]

        # --- Différence de position (mm/rad) ---
        delta_pos = (T_plus[:3, 3] - T_minus[:3, 3]) / (2.0 * epsilon_rad)

        # --- Différence d'orientation (rad/rad) via vecteur rotation ---
        # Rotation de T_minus vers T_plus : R_diff = R_plus · R_minusᵀ
        R_diff = T_plus[:3, :3] @ T_minus[:3, :3].T
        delta_ori = math_utils.rotation_matrix_to_rotation_vector(R_diff) / (2.0 * epsilon_rad)

        J[:, j] = np.concatenate([delta_pos, delta_ori])

    return J


def _resolution_amortie(J: np.ndarray,
                         erreur: np.ndarray,
                         lambda_damping: float) -> np.ndarray:
    """
    Résout la correction articulaire Δq par la méthode de Levenberg-Marquardt.

    Formule (Damped Least Squares — forme tâche) :
        Δq = Jᵀ · (J·Jᵀ + λ²·I₆)⁻¹ · erreur

    Cette formulation est numériquement robuste aux singularités :
    - En configuration régulière : J·Jᵀ est bien conditionné → solution précise
    - Près d'une singularité : λ² régularise la matrice → correction amortie,
      évitant des Δq instables qui violeraient les limites articulaires

    Note : pour une Jacobienne carrée J (6×6), cette forme est équivalente à
        Δq = (JᵀJ + λ²·I₆)⁻¹ · Jᵀ · erreur
    mais la forme tâche est préférée car elle reste valide pour J rectangulaire.

    Args:
        J:             Jacobienne 6×6
        erreur:        Vecteur d'erreur 6D [dx_mm, dy_mm, dz_mm, wx_rad, wy_rad, wz_rad]
        lambda_damping: Facteur d'amortissement λ

    Returns:
        Correction articulaire Δq en radians
    """
    lambda2 = lambda_damping ** 2
    # A = J·Jᵀ + λ²·I₆  (matrice 6×6 symétrique définie positive)
    A = J @ J.T + lambda2 * np.eye(6)
    # Résolution : A · x = erreur  →  x = A⁻¹ · erreur
    x = np.linalg.solve(A, erreur)
    # Correction articulaire : Δq = Jᵀ · x
    return J.T @ x


def _clamper_limites(q_deg: list[float],
                     axis_limits: list[tuple[float, float]]) -> list[float]:
    """
    Contraint les angles articulaires aux limites mécaniques du robot.

    Args:
        q_deg:       Valeurs articulaires en degrés
        axis_limits: Liste de (min_deg, max_deg) par joint

    Returns:
        Valeurs articulaires clampées
    """
    q_clamp = list(q_deg)
    for i in range(6):
        q_min, q_max = axis_limits[i]
        q_clamp[i] = max(q_min, min(q_max, q_clamp[i]))
    return q_clamp


# ============================================================================
# RÉGION: Solveur principal
# ============================================================================

def mgi_jacobien(target: list[float],
                 robot_model,
                 q_initial: list[float],
                 params: MgiJacobienParams | None = None,
                 tool: RobotTool | None = None) -> MgiJacobienResultat:
    """
    Solveur MGI optimisé par la Jacobienne inverse avec amortissement
    de Levenberg-Marquardt.

    Affine itérativement une estimation initiale (issue du MGI analytique) en
    utilisant le MGD CORRIGÉ pour tenir compte de toutes les corrections de
    calibration (translations Tx/Ty/Tz, rotations Rx/Ry/Rz, rotations parasites
    bêta non modélisables par la convention DH standard).

    Validation attendue :
        - Corrections nulles → résultats identiques au MGI analytique
        - Corrections non nulles → corrected_FK(q_opt) ≈ target (err < seuils)
        - Près des singularités (q5 ≈ 0°) → amortissement LM évite divergence

    Args:
        target:       Pose cible [x, y, z, a, b, c] en mm et degrés (ZYX Euler)
        robot_model:  Instance RobotModel avec MGD corrigé et limites articulaires
        q_initial:    Estimation initiale des joints en degrés (issue du MGI analytique)
        params:       Paramètres du solveur (None → valeurs par défaut)

    Returns:
        MgiJacobienResultat avec joints raffinés, métriques et statut de convergence
    """
    if params is None:
        params = MgiJacobienParams()

    # Matrice homogène cible (construite une seule fois)
    T_cible = _build_T_cible(*target)

    # Valeurs articulaires courantes (degrés) — copie pour ne pas modifier l'original
    q_deg = list(q_initial)

    resultat = MgiJacobienResultat(joints=list(q_deg))
    erreur_pos_mm = float('inf')
    erreur_ori_deg = float('inf')

    for i in range(params.max_iterations):
        # ----------------------------------------------------------------
        # 1. Calcul de la pose actuelle via le MGD CORRIGÉ
        # ----------------------------------------------------------------
        fk_result = robot_model.compute_fk_joints(q_deg, tool=tool)
        if fk_result is None:
            resultat.message = "Erreur : compute_fk_joints a retourné None"
            return resultat

        _, corrected_matrices, _, _, _ = fk_result
        T_actuel = corrected_matrices[-1]  # TCP corrigé (flange + outil)

        # ----------------------------------------------------------------
        # 2. Vecteur d'erreur 6D
        # ----------------------------------------------------------------
        erreur = _compute_erreur_pose(T_cible, T_actuel)

        erreur_pos_mm = float(np.linalg.norm(erreur[:3]))
        erreur_ori_deg = float(np.degrees(np.linalg.norm(erreur[3:])))

        # ----------------------------------------------------------------
        # 3. Test de convergence
        # ----------------------------------------------------------------
        if (erreur_pos_mm < params.seuil_position
                and erreur_ori_deg < params.seuil_orientation):
            resultat.joints = list(q_deg)
            resultat.converge = True
            resultat.nb_mises_a_jour = i
            resultat.erreur_position = erreur_pos_mm
            resultat.erreur_orientation = erreur_ori_deg
            if i == 0:
                resultat.message = "Point analytique déjà dans la tolérance du FK corrigé"
            else:
                resultat.message = f"Convergé en {i} mise(s) à jour"
            return resultat

        # ----------------------------------------------------------------
        # 4. Jacobienne numérique (différences finies centrées)
        # ----------------------------------------------------------------
        J = _compute_jacobienne_numerique(q_deg, robot_model, params.epsilon, tool=tool)

        # ----------------------------------------------------------------
        # 5. Correction articulaire (Levenberg-Marquardt)
        # ----------------------------------------------------------------
        delta_q_rad = _resolution_amortie(J, erreur, params.lambda_damping)
        delta_q_deg = np.degrees(delta_q_rad)

        # ----------------------------------------------------------------
        # 6. Mise à jour des joints + clamping aux limites articulaires
        # ----------------------------------------------------------------
        q_deg = [q_deg[j] + delta_q_deg[j] for j in range(6)]
        q_deg = _clamper_limites(q_deg, robot_model.get_axis_limits())

    # --- Non convergé après max_iterations ---
    resultat.joints = list(q_deg)
    resultat.converge = False
    resultat.nb_mises_a_jour = params.max_iterations
    resultat.erreur_position = erreur_pos_mm
    resultat.erreur_orientation = erreur_ori_deg
    resultat.message = (
        f"Non convergé après {params.max_iterations} mises à jour "
        f"(err_pos={erreur_pos_mm:.4f} mm, err_ori={erreur_ori_deg:.4f}°)"
    )
    return resultat
