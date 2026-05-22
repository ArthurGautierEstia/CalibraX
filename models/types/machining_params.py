from __future__ import annotations

from dataclasses import dataclass, field


# ===========================================================================
# Structures de paramètres (utilisées dans les calculs)
# ===========================================================================

@dataclass
class CuttingParams:
    """Paramètres du modèle mécanistique d'Altintas (fraisage).

    La vitesse de broche N (tr/min) et la vitesse d'avance v_f (mm/min) sont
    les grandées saisies par l'utilisateur. L'avance par dent f_z est déduite :
        f_z = v_f / (N × z)
    """

    a_p: float = 2.0
    """Profondeur axiale de passe (mm)."""

    a_e: float = 5.0
    """Engagement radial (mm)."""

    diameter: float = 20.0
    """Diamètre de l'outil D (mm)."""

    z_teeth: int = 4
    """Nombre de dents."""

    spindle_speed_rpm: float = 3000.0
    """Vitesse de rotation de la broche N (tr/min)."""

    feed_rate_mm_min: float = 1200.0
    """Vitesse d'avance v_f (mm/min)."""

    K_tc: float = 550.0
    """Coefficient de cisaillement tangentiel (MPa = N/mm²). Défaut : Al7075-T6."""

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

    @property
    def f_z(self) -> float:
        """Avance par dent calculée : f_z = v_f / (N × z) (mm/dent)."""
        if self.spindle_speed_rpm > 0.0 and self.z_teeth > 0:
            return self.feed_rate_mm_min / (self.spindle_speed_rpm * self.z_teeth)
        return 0.0


@dataclass
class RobotMechanicalParams:
    """Paramètres mécaniques du robot pour le calcul de déformation articulaire."""

    joint_stiffness_Nm_per_rad: list[float] = field(
        default_factory=lambda: [6.5e6, 7.6e6, 3.08e6, 5.0e5, 6.0e5, 4.0e5]
    )
    """Raideurs articulaires k_i (N·m/rad). Défaut KR500 / KR500 MT.
    k1, k2 : mesurés (Jubien, Abba, Gautier — ICINCO 2014 + COROUSSO LI4.3).
    k3     : mesuré (COROUSSO LI4.3).
    k4–k6  : estimés par analogie KR270-2F — à confirmer."""

    joint_torque_max_Nm: list[float] = field(
        default_factory=lambda: [5000.0, 5000.0, 3000.0, 1500.0, 1000.0, 600.0]
    )
    """Couples moteur maximaux τ_max_i (N·m). Valeurs indicatives KR500 — à confirmer."""


@dataclass
class MachiningSimulationParams:
    """Paramètres complets pour la simulation d'usinage robotisé."""

    cutting: CuttingParams = field(default_factory=CuttingParams)
    mechanical: RobotMechanicalParams = field(default_factory=RobotMechanicalParams)


# ===========================================================================
# Presets matériaux
# ===========================================================================

@dataclass
class MaterialPreset:
    """Profil de coefficients de coupe Altintas pour un matériau donné."""

    name: str
    K_tc: float
    """Coefficient de cisaillement tangentiel (MPa)."""
    K_rc: float
    K_ac: float
    K_te: float = 0.0
    """Coefficient d'arête tangentielle (N/mm)."""
    K_re: float = 0.0
    K_ae: float = 0.0
    source: str = ""


MATERIAL_PRESETS: list[MaterialPreset] = [
    MaterialPreset(
        name="Al7075-T6",
        K_tc=550.0, K_rc=200.0, K_ac=150.0,
        source="Altintas 2012 (valeurs de référence)",
    ),
    MaterialPreset(
        name="Al6061-T6",
        K_tc=420.0, K_rc=160.0, K_ac=120.0,
        source="Estimation analogie Al 6xxx — à valider",
    ),
    MaterialPreset(
        name="Al2024-T3",
        K_tc=480.0, K_rc=180.0, K_ac=135.0,
        source="Estimation analogie Al 2xxx — à valider",
    ),
]


# ===========================================================================
# Presets robots
# ===========================================================================

@dataclass
class RobotPreset:
    """Profil de raideurs et de couples maximaux pour un modèle de robot."""

    name: str
    stiffness_Nm_per_rad: list[float]
    """Raideurs k_i (N·m/rad), longueur 6."""
    torque_max_Nm: list[float]
    """Couples moteur maximaux τ_max_i (N·m), longueur 6."""
    stiffness_sources: list[str]
    """Source de chaque valeur (une entrée par axe)."""


ROBOT_PRESETS: list[RobotPreset] = [
    RobotPreset(
        name="KUKA KR500 / KR500 MT",
        stiffness_Nm_per_rad=[6.5e6, 7.6e6, 3.08e6, 5.0e5, 6.0e5, 4.0e5],
        torque_max_Nm=[5000.0, 5000.0, 3000.0, 1500.0, 1000.0, 600.0],
        stiffness_sources=[
            "Jubien 2014 + COROUSSO LI4.3 (mesuré)",
            "Jubien 2014 + COROUSSO LI4.3 (mesuré)",
            "COROUSSO LI4.3 (mesuré)",
            "Analogie KR270-2F (estimé ⚠)",
            "Analogie KR270-2F (estimé ⚠)",
            "Analogie KR270-2F (estimé ⚠)",
        ],
    ),
    RobotPreset(
        name="KUKA KR240 / KR270-2F",
        stiffness_Nm_per_rad=[3.8e6, 6.6e6, 3.9e6, 5.6e5, 6.6e5, 4.7e5],
        torque_max_Nm=[3000.0, 3000.0, 2000.0, 1000.0, 800.0, 400.0],
        stiffness_sources=[
            "COROUSSO LI4.3 (mesuré)",
            "COROUSSO LI4.3 (mesuré)",
            "COROUSSO LI4.3 (mesuré)",
            "COROUSSO LI4.3 (mesuré)",
            "COROUSSO LI4.3 (mesuré)",
            "COROUSSO LI4.3 (mesuré)",
        ],
    ),
]
