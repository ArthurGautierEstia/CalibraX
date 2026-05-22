"""
Modèle mécanistique d'Altintas pour le fraisage (efforts de coupe moyennés sur un tour).

Référence : Altintas Y., Manufacturing Automation, Cambridge UP, 2nd Ed., 2012.
            Rivière-Lorphèvre et al., CIRP, 2019.

Modèle linéaire moyenné (forces intégrées sur l'arc d'engagement) :
    F_t = K_tc · a_p · h_mean + K_te · a_p
    F_r = K_rc · a_p · h_mean + K_re · a_p
    F_a = K_ac · a_p · h_mean + K_ae · a_p

Convention repère outil :
    F_t : direction tangentielle (sens d'avance de l'outil)
    F_r : direction radiale (vers l'axe de la broche)
    F_a : direction axiale (axe broche Z+, parallèle à l'axe outil)

Aucune dépendance PyQt — module pur calcul, transposable en C++.
"""

from __future__ import annotations

import math

from models.types.machining_params import CuttingParams


def compute_engagement_angles(a_e: float, diameter: float) -> tuple[float, float]:
    """Calcule les angles d'entrée et de sortie de l'outil (fraisage en avalant).

    Pour du fraisage en avalant (down-milling) :
        phi_enter = 0
        phi_exit  = arccos(1 − 2·a_e/D)

    Args:
        a_e:      Engagement radial (mm). Doit satisfaire 0 < a_e <= diameter.
        diameter: Diamètre de l'outil D (mm).

    Returns:
        (phi_enter, phi_exit) en radians.
    """
    ratio = max(0.0, min(1.0, 2.0 * a_e / diameter))
    phi_enter = 0.0
    phi_exit = math.acos(1.0 - ratio)
    return phi_enter, phi_exit


def compute_h_mean(f_z: float, phi_enter: float, phi_exit: float) -> float:
    """Calcule l'épaisseur de copeau moyenne sur l'arc d'engagement.

    h_mean = (f_z / π) · (cos φ_enter − cos φ_exit)

    Args:
        f_z:       Avance par dent (mm/dent).
        phi_enter: Angle d'entrée (rad).
        phi_exit:  Angle de sortie (rad).

    Returns:
        Épaisseur de copeau moyenne h_mean (mm).
    """
    return (f_z / math.pi) * (math.cos(phi_enter) - math.cos(phi_exit))


def compute_cutting_forces_tool_frame(params: CuttingParams) -> tuple[float, float, float]:
    """Calcule les efforts de coupe moyennés dans le repère outil.

    Modèle linéaire d'Altintas (efforts totaux sur les z dents, moyennés sur un tour) :
        F_t = K_tc · a_p · h_mean + K_te · a_p
        F_r = K_rc · a_p · h_mean + K_re · a_p
        F_a = K_ac · a_p · h_mean + K_ae · a_p

    Les coefficients K_xc sont en MPa (= N/mm²), a_p et h_mean en mm.
    K_xc · a_p (mm) · h_mean (mm) → N/mm² · mm² = N. Unités cohérentes.
    Les coefficients K_xe sont en N/mm, a_p en mm → N.

    Note : le modèle est stationnaire (pas de dépendance à la position angulaire de la broche).
    Le nombre de dents z est implicitement intégré dans les coefficients identifiés K_xc, K_xe.

    Args:
        params: Paramètres de coupe (CuttingParams).

    Returns:
        (F_t, F_r, F_a) en N, exprimés dans le repère outil.
    """
    phi_enter, phi_exit = compute_engagement_angles(params.a_e, params.diameter)
    h_mean = compute_h_mean(params.f_z, phi_enter, phi_exit)

    a_p = params.a_p
    f_t = params.K_tc * a_p * h_mean + params.K_te * a_p
    f_r = params.K_rc * a_p * h_mean + params.K_re * a_p
    f_a = params.K_ac * a_p * h_mean + params.K_ae * a_p

    return f_t, f_r, f_a
