from __future__ import annotations

from dataclasses import dataclass, field

from models.types.joint_angles6 import JointAngles6


@dataclass
class MachiningSamplePoint:
    """Résultat de simulation d'usinage pour un échantillon articulaire."""

    time_s: float
    """Instant de l'échantillon (s)."""

    joints_deg: JointAngles6
    """Configuration articulaire (degrés)."""

    force_tool_N: tuple[float, float, float]
    """Efforts de coupe dans le repère outil (F_t, F_r, F_a) en N.
    Convention : F_t = tangentiel (sens avance), F_r = radial, F_a = axial (axe broche Z+)."""

    force_base_N: tuple[float, float, float]
    """Efforts de coupe exprimés dans le repère base robot (N)."""

    torque_cut_Nm: list[float]
    """Couples articulaires dus à la coupe τ_cut (N·m), longueur 6."""

    torque_total_Nm: list[float]
    """Couples articulaires totaux τ_total = τ_cut (v1, gravité ignorée) (N·m), longueur 6."""

    delta_theta_rad: list[float]
    """Déformations articulaires δθ_i = τ_i / k_i (rad), longueur 6."""

    delta_tcp_mm: float
    """Norme de la déviation de position TCP ‖δx_TCP[0:3]‖ (mm)."""

    torque_ratio: list[float]
    """Ratios de charge |τ_total_i| / τ_max_i, longueur 6. Valeur > 1.0 = dépassement."""

    overload: bool
    """True si au moins un ratio dépasse 1.0."""


@dataclass
class MachiningResult:
    """Résultat de simulation d'usinage sur l'ensemble d'une trajectoire."""

    samples: list[MachiningSamplePoint] = field(default_factory=list)
    """Liste des résultats par échantillon articulaire."""

    warnings: list[str] = field(default_factory=list)
    """Avertissements générés pendant la simulation (singularités, paramètres limites…)."""

    overload_count: int = 0
    """Nombre d'échantillons avec au moins un axe en dépassement de couple."""
