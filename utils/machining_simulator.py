"""
Orchestrateur de simulation d'usinage robotisé.

Pour chaque échantillon articulaire d'une trajectoire simulée (ProgramSimulationResult),
calcule les efforts de coupe, les couples articulaires, les déformations et l'écart TCP.

Aucune dépendance PyQt — module pur calcul, transposable en C++.
"""

from __future__ import annotations

import numpy as np

from models.robot_program import ProgramSimulationResult
from models.types.machining_params import MachiningSimulationParams
from models.types.machining_result import MachiningSamplePoint, MachiningResult
from utils.machining_forces import compute_cutting_forces_tool_frame
from utils.machining_torques import (
    compute_joint_deflections,
    compute_joint_torques_from_force,
    compute_tcp_deviation,
    compute_tcp_jacobian,
    force_tool_to_base,
    gravity_torques_placeholder,
)

_SINGULARITY_COND_THRESHOLD = 1e8


def simulate_machining(
    program_result: ProgramSimulationResult,
    params: MachiningSimulationParams,
    robot_model,
    tool,
) -> MachiningResult:
    """Simule les efforts d'usinage sur l'ensemble d'une trajectoire robot.

    Args:
        program_result: Résultat de simulation de programme robot (nominal_samples).
        params:         Paramètres usinage (coupe + mécanique robot).
        robot_model:    Instance RobotModel (MGD corrigé + Jacobien).
        tool:           Instance RobotTool ou None (TCP flange si None).

    Returns:
        MachiningResult avec un MachiningSamplePoint par échantillon nominal.
    """
    samples = program_result.nominal_samples
    if not samples:
        return MachiningResult(
            samples=[],
            warnings=["Aucun échantillon dans la trajectoire."],
            overload_count=0,
        )

    # Efforts de coupe constants sur la trajectoire (modèle moyenné v1)
    forces_tool = compute_cutting_forces_tool_frame(params.cutting)
    k_stiffness = params.mechanical.joint_stiffness_Nm_per_rad
    tau_max = params.mechanical.joint_torque_max_Nm

    samples_out: list[MachiningSamplePoint] = []
    warnings: list[str] = []
    overload_count = 0

    for s in samples:
        q_deg = s.joints_deg.to_list()

        # MGD corrigé pour obtenir la matrice TCP
        fk = robot_model.compute_fk_joints(q_deg, tool=tool)
        if fk is None:
            warnings.append(f"MGD nul à t={s.time_s:.3f} s — échantillon ignoré.")
            continue

        T_tcp = fk.corrected_matrices[-1]
        R_tool_to_base = T_tcp[:3, :3]

        # Jacobien en unités SI (m/rad ; rad/rad)
        J_si = compute_tcp_jacobian(robot_model, q_deg, tool)

        if np.linalg.cond(J_si) > _SINGULARITY_COND_THRESHOLD:
            warnings.append(
                f"Jacobien quasi singulier à t={s.time_s:.3f} s "
                f"(cond={np.linalg.cond(J_si):.2e}) — résultats potentiellement instables."
            )

        # Efforts dans le repère base
        F_base = force_tool_to_base(forces_tool, R_tool_to_base)

        # Couples articulaires (τ_cut)
        tau_cut = compute_joint_torques_from_force(J_si, F_base)

        # Gravité (nulle en v1)
        tau_gravity = gravity_torques_placeholder(q_deg)
        tau_total = tau_cut + tau_gravity

        # Déformation articulaire et déviation TCP
        delta_theta = compute_joint_deflections(tau_total, k_stiffness)
        _, delta_tcp_mm = compute_tcp_deviation(J_si, delta_theta)

        # Ratios de charge
        ratios = [abs(tau_total[i]) / tau_max[i] for i in range(6)]
        overload = any(r > 1.0 for r in ratios)
        if overload:
            overload_count += 1

        samples_out.append(MachiningSamplePoint(
            time_s=s.time_s,
            joints_deg=s.joints_deg,
            force_tool_N=(forces_tool[0], forces_tool[1], forces_tool[2]),
            force_base_N=(float(F_base[0]), float(F_base[1]), float(F_base[2])),
            torque_cut_Nm=tau_cut.tolist(),
            torque_total_Nm=tau_total.tolist(),
            delta_theta_rad=delta_theta.tolist(),
            delta_tcp_mm=delta_tcp_mm,
            torque_ratio=ratios,
            overload=overload,
        ))

    return MachiningResult(
        samples=samples_out,
        warnings=warnings,
        overload_count=overload_count,
    )
