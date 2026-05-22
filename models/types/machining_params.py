from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class CuttingParams:
    """Paramètres du modèle mécanistique d'Altintas (fraisage)."""

    a_p: float = 2.0
    """Profondeur axiale de passe (mm)."""

    a_e: float = 5.0
    """Engagement radial (mm)."""

    f_z: float = 0.1
    """Avance par dent (mm/dent)."""

    diameter: float = 20.0
    """Diamètre de l'outil D (mm)."""

    z_teeth: int = 4
    """Nombre de dents."""

    K_tc: float = 550.0
    """Coefficient de cisaillement tangentiel (MPa = N/mm²). Défaut : Al7075."""

    K_rc: float = 200.0
    """Coefficient de cisaillement radial (MPa)."""

    K_ac: float = 150.0
    """Coefficient de cisaillement axial (MPa)."""

    K_te: float = 0.0
    """Coefficient d'arête tangentielle (N/mm). Zéro = simplification initiale."""

    K_re: float = 0.0
    """Coefficient d'arête radiale (N/mm)."""

    K_ae: float = 0.0
    """Coefficient d'arête axiale (N/mm)."""


@dataclass
class RobotMechanicalParams:
    """Paramètres mécaniques du robot pour le calcul de déformation articulaire."""

    joint_stiffness_Nm_per_rad: list[float] = field(
        default_factory=lambda: [1.5e6, 1.2e6, 0.8e6, 0.5e6, 0.4e6, 0.3e6]
    )
    """Raideurs articulaires k_i (N·m/rad). Valeurs KR500-3 identifiées par Dumas et al., 2014."""

    joint_torque_max_Nm: list[float] = field(
        default_factory=lambda: [5000.0, 5000.0, 3000.0, 1500.0, 1000.0, 600.0]
    )
    """Couples moteur maximaux τ_max_i (N·m). Valeurs indicatives KR500-3 — à confirmer fiche constructeur."""


@dataclass
class MachiningSimulationParams:
    """Paramètres complets pour la simulation d'usinage robotisé."""

    cutting: CuttingParams = field(default_factory=CuttingParams)
    mechanical: RobotMechanicalParams = field(default_factory=RobotMechanicalParams)
