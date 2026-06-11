from __future__ import annotations

from bisect import bisect_left
from collections import defaultdict
from dataclasses import dataclass, replace
import hashlib
import math
from typing import TYPE_CHECKING

import numpy as np

import utils.math_utils as math_utils
from models.robot_program import (
    ProgramCompensationOutputMode,
    ProgramSimulationResult,
    ProgramSimulationSample,
    RobotProgram,
    RobotProgramBrand,
    RobotProgramMotion,
    RobotProgramMotionMode,
    RobotProgramTarget,
    RobotProgramTargetType,
)
from models.external_axes_model import ExternalAxesModel
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.types import JointAngles6, Pose6
from models.workpiece_model import WorkpieceModel
from models.workspace_model import WorkspaceModel
from utils.external_axes_kinematics import piece_frame_world
from utils.math_utils import invert_homogeneous_transform, pose_zyx_to_matrix
from utils.mgi import MGI, MgiAxisLimits, MgiConfigurationFilter, MgiGeometricParams, MgiParams, RobotTool
from utils.reference_frame_utils import pose_to_matrix, matrix_to_pose


@dataclass(frozen=True)
class _PtpProbeSample:
    time_s: float
    joints_deg: JointAngles6
    pose_base: Pose6


@dataclass(frozen=True)
class _ProgramStartState:
    initial_joints_deg: JointAngles6
    initial_pose_base: Pose6
    remaining_motions: list[RobotProgramMotion]
    initial_sample_motion: RobotProgramMotion | None


@dataclass(frozen=True)
class _MotionSimCacheEntry:
    """Cache d'un motion simulé (indépendant du temps absolu)."""
    signature: str              # hash des champs métier (hors approximation)
    start_joints: tuple         # joints au début du motion
    start_pose: Pose6           # pose TCP au début du motion
    relative_samples: list[ProgramSimulationSample]  # temps relatifs à partir de 0
    end_joints: tuple           # joints après le motion
    end_pose: Pose6             # pose TCP après le motion
    duration_s: float           # durée totale du motion


def _motion_signature(motion: RobotProgramMotion) -> str:
    """Hash stable des champs influençant la simulation (hors approximation)."""
    ext_key = None
    if motion.external_axis_target is not None:
        ext_key = tuple(
            (v.axis_id, v.joint_index, v.value, v.speed)
            for v in motion.external_axis_target.values
        )
    key = (
        motion.mode.value,
        motion.target.target_type.value,
        motion.target.cartesian_pose.to_list() if motion.target.cartesian_pose else None,
        motion.target.joint_angles.to_list() if motion.target.joint_angles else None,
        motion.cp_speed_mps,
        motion.tool_pose.to_list() if motion.tool_pose else None,
        motion.base_pose.to_list() if motion.base_pose else None,
        motion.via_target.target_type.value if motion.via_target else None,
        motion.via_target.cartesian_pose.to_list() if (motion.via_target and motion.via_target.cartesian_pose) else None,
        ext_key,
    )
    return hashlib.md5(str(key).encode(), usedforsecurity=False).hexdigest()


class ProgramSimulator:
    DEFAULT_DT_S = 0.02
    DEFAULT_LINEAR_SPEED_MPS = 0.2
    DEFAULT_PTP_SPEED_PERCENT = 50.0
    MAX_TRAJECTORY_SAMPLES = 3000
    MIN_CARTESIAN_SAMPLE_STEP_MM = 1.0

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        external_axes_model: ExternalAxesModel | None = None,
        workspace_model: WorkspaceModel | None = None,
        workpiece_model: WorkpieceModel | None = None,
    ) -> None:
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.external_axes_model = external_axes_model
        self.workspace_model = workspace_model
        self.workpiece_model = workpiece_model
        # Lot A.1 : cache DH mesuré valide pendant une passe de simulate_program
        self._cached_measured_dh: list[list[float]] | None = None
        # Lot F : arrays numpy pré-extraits pour FK mesuré vectorisé
        self._cached_measured_dh_arrays: tuple | None = None
        self._cached_axis_reversed: np.ndarray | None = None
        # Cache incrémental par motion (parallèle à program.motions)
        self._motion_cache: list[_MotionSimCacheEntry] = []
        # Lot B : pas d'interpolation actif pour la simulation courante
        self._active_cartesian_step_mm: float = self.MIN_CARTESIAN_SAMPLE_STEP_MM
        # État courant des axes externes pendant la simulation (axis_id, joint_idx) → valeur
        self._current_ext_axis_values: dict[tuple[str, int], float] = {}

    def simulate_program(self, program: RobotProgram, include_compensation: bool = True) -> ProgramSimulationResult:
        if program.brand != RobotProgramBrand.KUKA:
            return ProgramSimulationResult(warnings=["Format de programme non supporte."])
        if not self.robot_model.get_has_configuration():
            return ProgramSimulationResult(warnings=["Charger une configuration robot avant de simuler un programme."])

        self._init_ext_axis_state()
        # Lot A.1 : snapshot DH mesuré pour toute la passe
        self._cached_measured_dh = self._compute_normalized_measured_dh_table()
        # Lot F : pré-extraire les arrays numpy pour FK mesuré vectorisé
        self._rebuild_measured_dh_arrays()
        # Lot B : calcul du pas adaptatif basé sur la longueur totale estimée
        total_length_mm = self._estimate_total_path_length_mm(program.motions)
        self._active_cartesian_step_mm = max(
            self.MIN_CARTESIAN_SAMPLE_STEP_MM,
            total_length_mm / max(1, self.MAX_TRAJECTORY_SAMPLES),
        )
        try:
            nominal_samples = self._simulate_motion_list(program.motions)
            warnings = list(program.warnings)
            cartesian_program: RobotProgram | None = None
            articular_program: RobotProgram | None = None
            cartesian_samples: list[ProgramSimulationSample] = []
            articular_samples: list[ProgramSimulationSample] = []

            measured_dh = self._cached_measured_dh
            if measured_dh is not None and include_compensation:
                cartesian_program = self._build_compensated_program(program, ProgramCompensationOutputMode.CARTESIAN, measured_dh)
                articular_program = self._build_compensated_program(program, ProgramCompensationOutputMode.ARTICULAR, measured_dh)
                if cartesian_program is not None:
                    cartesian_samples = self._simulate_motion_list(cartesian_program.motions)
                if articular_program is not None:
                    articular_samples = self._simulate_motion_list(articular_program.motions)

            return ProgramSimulationResult(
                nominal_samples=nominal_samples,
                cartesian_compensated_samples=cartesian_samples,
                articular_compensated_samples=articular_samples,
                cartesian_compensated_program=cartesian_program,
                articular_compensated_program=articular_program,
                warnings=warnings,
                compensation_computed=include_compensation or measured_dh is None,
            )
        finally:
            # Reconstruire le cache après une simulation complète
            self._motion_cache = self._build_motion_cache(program.motions, nominal_samples)
            self._cached_measured_dh = None
            self._cached_measured_dh_arrays = None
            self._cached_axis_reversed = None

    def simulate_program_incremental(
        self,
        program: RobotProgram,
        dirty_indices: list[int],
    ) -> ProgramSimulationResult:
        """Simulation incrémentale : re-simule uniquement les motions dirty et cascade si nécessaire.

        Ne recalcule pas la compensation (appeler compute_compensation séparément si besoin).
        """
        if program.brand != RobotProgramBrand.KUKA:
            return ProgramSimulationResult(warnings=["Format de programme non supporte."])
        if not self.robot_model.get_has_configuration():
            return ProgramSimulationResult(warnings=["Charger une configuration robot avant de simuler un programme."])

        self._init_ext_axis_state()
        self._cached_measured_dh = self._compute_normalized_measured_dh_table()
        self._rebuild_measured_dh_arrays()
        total_length_mm = self._estimate_total_path_length_mm(program.motions)
        self._active_cartesian_step_mm = max(
            self.MIN_CARTESIAN_SAMPLE_STEP_MM,
            total_length_mm / max(1, self.MAX_TRAJECTORY_SAMPLES),
        )

        try:
            nominal_samples = self._simulate_incremental(program.motions, set(dirty_indices))
        finally:
            self._cached_measured_dh = None
            self._cached_measured_dh_arrays = None
            self._cached_axis_reversed = None

        return ProgramSimulationResult(
            nominal_samples=nominal_samples,
            warnings=list(program.warnings),
        )

    def _build_motion_cache(
        self,
        motions: list[RobotProgramMotion],
        all_samples: list[ProgramSimulationSample],
    ) -> list[_MotionSimCacheEntry]:
        """Construit le cache à partir des samples d'une simulation complète."""
        if not motions:
            return []

        # Grouper les samples par source_line (correspondance approximative)
        samples_by_line: dict[int, list[ProgramSimulationSample]] = defaultdict(list)
        for s in all_samples:
            samples_by_line[int(s.source_line)].append(s)

        cache: list[_MotionSimCacheEntry] = []
        prev_joints: tuple = tuple(self.robot_model.get_joints())
        prev_pose: Pose6 = Pose6.zeros()

        for motion in motions:
            sig = _motion_signature(motion)
            motion_samples = samples_by_line.get(int(motion.line_number), [])

            if not motion_samples:
                cache.append(_MotionSimCacheEntry(
                    signature=sig,
                    start_joints=prev_joints,
                    start_pose=prev_pose,
                    relative_samples=[],
                    end_joints=prev_joints,
                    end_pose=prev_pose,
                    duration_s=0.0,
                ))
                continue

            t0 = float(motion_samples[0].time_s)
            relative = [
                replace(s, time_s=float(s.time_s) - t0) for s in motion_samples
            ]
            end_joints = tuple(motion_samples[-1].joints_deg.to_list())
            end_pose = motion_samples[-1].nominal_pose_base
            duration_s = float(motion_samples[-1].time_s) - float(motion_samples[0].time_s)

            cache.append(_MotionSimCacheEntry(
                signature=sig,
                start_joints=prev_joints,
                start_pose=prev_pose,
                relative_samples=relative,
                end_joints=end_joints,
                end_pose=end_pose,
                duration_s=duration_s,
            ))
            prev_joints = end_joints
            prev_pose = end_pose

        return cache

    def _simulate_incremental(
        self,
        motions: list[RobotProgramMotion],
        dirty: set[int],
    ) -> list[ProgramSimulationSample]:
        """Simule la liste de motions en réutilisant le cache pour les motions inchangés."""
        start_state = self._resolve_program_start_state(motions)
        current_joints = start_state.initial_joints_deg.to_list()
        current_pose = start_state.initial_pose_base.copy()
        current_time_s = 0.0
        all_samples: list[ProgramSimulationSample] = []

        # Redimensionner le cache si le nombre de motions a changé
        while len(self._motion_cache) < len(motions):
            self._motion_cache.append(_MotionSimCacheEntry(
                signature="", start_joints=(), start_pose=Pose6.zeros(),
                relative_samples=[], end_joints=(), end_pose=Pose6.zeros(), duration_s=0.0,
            ))

        if start_state.initial_sample_motion is not None:
            initial_tool = self._tool_from_pose(start_state.initial_sample_motion.tool_pose)
            all_samples.append(self._build_sample(0.0, start_state.initial_sample_motion, current_joints, initial_tool))

        new_cache: list[_MotionSimCacheEntry] = []

        for idx, motion in enumerate(start_state.remaining_motions):
            actual_idx = idx + (1 if start_state.initial_sample_motion is not None else 0)
            sig = _motion_signature(motion)
            cached = self._motion_cache[actual_idx] if actual_idx < len(self._motion_cache) else None

            start_joints_tuple = tuple(current_joints)
            cache_valid = (
                cached is not None
                and actual_idx not in dirty
                and cached.signature == sig
                and cached.start_joints == start_joints_tuple
                and cached.start_pose == current_pose
            )

            if cache_valid and cached is not None:
                # Réutiliser les samples du cache en décalant les temps
                restitched = [
                    replace(s, time_s=float(s.time_s) + current_time_s)
                    for s in cached.relative_samples
                ]
                all_samples.extend(restitched)
                current_joints = list(cached.end_joints)
                current_pose = cached.end_pose
                current_time_s += cached.duration_s
                # Synchroniser l'état des axes externes depuis le dernier sample du cache
                if cached.relative_samples and cached.relative_samples[-1].ext_axis_values:
                    self._update_ext_axis_state_from_snapshot(cached.relative_samples[-1].ext_axis_values)
                new_cache.append(cached)
                continue

            # Resimulation nécessaire
            motion_tool = self._tool_from_pose(motion.tool_pose)
            generated = self._simulate_motion(motion, current_pose, current_joints, current_time_s, motion_tool)

            if generated:
                t0 = float(generated[0].time_s)
                relative = [replace(s, time_s=float(s.time_s) - t0) for s in generated]
                end_joints = tuple(generated[-1].joints_deg.to_list())
                end_pose = generated[-1].nominal_pose_base
                duration_s = float(generated[-1].time_s) - t0

                # Cascade si end_state diffère
                if cached is not None and (cached.end_joints != end_joints or cached.end_pose != end_pose):
                    if actual_idx + 1 < len(motions):
                        dirty.add(actual_idx + 1)

                new_entry = _MotionSimCacheEntry(
                    signature=sig,
                    start_joints=start_joints_tuple,
                    start_pose=current_pose,
                    relative_samples=relative,
                    end_joints=end_joints,
                    end_pose=end_pose,
                    duration_s=duration_s,
                )
                all_samples.extend(generated)
                current_joints = list(end_joints)
                current_pose = end_pose
                current_time_s = float(generated[-1].time_s)
            else:
                new_entry = _MotionSimCacheEntry(
                    signature=sig,
                    start_joints=start_joints_tuple,
                    start_pose=current_pose,
                    relative_samples=[],
                    end_joints=start_joints_tuple,
                    end_pose=current_pose,
                    duration_s=0.0,
                )

            new_cache.append(new_entry)

        self._motion_cache = new_cache
        return all_samples

    def build_error_curves(
        self,
        nominal_samples: list[ProgramSimulationSample],
        compensated_samples: list[ProgramSimulationSample],
    ) -> tuple[list[float], list[float], list[float]]:
        nominal_ok = [s for s in nominal_samples if s.measured_pose_base is not None]
        if not nominal_ok:
            return [], [], []

        nominal_xyz = [self._pose_xyz(s.nominal_pose_base) for s in nominal_ok]
        abscissa_mm = self._cumulative_arc_lengths_mm(nominal_xyz)

        real_error_mm = [
            self._xyz_distance_mm(self._pose_xyz(s.measured_pose_base), self._pose_xyz(s.nominal_pose_base))
            for s in nominal_ok
        ]

        if not compensated_samples:
            return abscissa_mm, real_error_mm, []

        compensated_error_mm = self._compute_compensated_error_per_segment(nominal_ok, compensated_samples)
        if not any(v != 0.0 for v in compensated_error_mm):
            return abscissa_mm, real_error_mm, []
        return abscissa_mm, real_error_mm, compensated_error_mm

    def _compute_compensated_error_per_segment(
        self,
        nominal_ok: list[ProgramSimulationSample],
        compensated_samples: list[ProgramSimulationSample],
    ) -> list[float]:
        compensated_ok = [s for s in compensated_samples if s.measured_pose_base is not None]

        nom_by_line: dict[int, list[tuple[int, ProgramSimulationSample]]] = defaultdict(list)
        for idx, s in enumerate(nominal_ok):
            nom_by_line[s.source_line].append((idx, s))

        comp_by_line: dict[int, list[ProgramSimulationSample]] = defaultdict(list)
        for s in compensated_ok:
            comp_by_line[s.source_line].append(s)

        result = [0.0] * len(nominal_ok)
        any_written = False

        for source_line, nom_entries in nom_by_line.items():
            comp_entries = comp_by_line.get(source_line)
            if not comp_entries:
                continue

            nom_xyz_list = [self._pose_xyz(s.nominal_pose_base) for _, s in nom_entries]
            nom_lengths = self._cumulative_arc_lengths_mm(nom_xyz_list)
            total_nom = nom_lengths[-1]

            comp_nom_xyz_list = [self._pose_xyz(s.nominal_pose_base) for s in comp_entries]
            comp_meas_xyz_list = [self._pose_xyz(s.measured_pose_base) for s in comp_entries]
            comp_nom_lengths = self._cumulative_arc_lengths_mm(comp_nom_xyz_list)
            total_comp_nom = comp_nom_lengths[-1]

            for local_idx, (global_idx, nom_s) in enumerate(nom_entries):
                nom_len = nom_lengths[local_idx]
                progress = nom_len / total_nom if total_nom > 1e-9 else local_idx / max(1, len(nom_entries) - 1)
                target_len = progress * total_comp_nom
                interp_meas_xyz = self._interp_xyz_at_arc_length(comp_meas_xyz_list, comp_nom_lengths, target_len)
                result[global_idx] = self._xyz_distance_mm(interp_meas_xyz, self._pose_xyz(nom_s.nominal_pose_base))
                any_written = True

        return result if any_written else [0.0] * len(nominal_ok)

    # =========================================================================
    # Lot B : estimation longueur totale pour le pas adaptatif
    # =========================================================================

    def _estimate_total_path_length_mm(self, motions: list[RobotProgramMotion]) -> float:
        start_state = self._resolve_program_start_state(motions)
        current_joints_deg = start_state.initial_joints_deg.to_list()
        current_pose = start_state.initial_pose_base
        total_mm = 0.0
        for motion in start_state.remaining_motions:
            motion_tool = self._tool_from_pose(motion.tool_pose)
            if motion.mode == RobotProgramMotionMode.PTP:
                target_joints = self._resolve_ptp_target_joints(motion, current_joints_deg, motion_tool)
                if target_joints is None:
                    continue
                target_pose = self._fk_nominal_pose_base(target_joints, motion_tool)
                total_mm += self._distance_xyz_mm(current_pose, target_pose)
                current_joints_deg = target_joints
                current_pose = target_pose
            elif motion.mode == RobotProgramMotionMode.LINEAR:
                target_pose = self._target_pose_base(motion, motion.target, motion_tool)
                total_mm += self._distance_xyz_mm(current_pose, target_pose)
                current_pose = target_pose
            elif motion.mode == RobotProgramMotionMode.CIRCULAR and motion.via_target is not None:
                via_pose = self._target_pose_base(motion, motion.via_target, motion_tool)
                target_pose = self._target_pose_base(motion, motion.target, motion_tool)
                total_mm += self._distance_xyz_mm(current_pose, via_pose) + self._distance_xyz_mm(via_pose, target_pose)
                current_pose = target_pose
        return total_mm

    # =========================================================================
    # Simulation de la liste de mouvements
    # =========================================================================

    def _simulate_motion_list(self, motions: list[RobotProgramMotion]) -> list[ProgramSimulationSample]:
        start_state = self._resolve_program_start_state(motions)
        current_joints_deg = start_state.initial_joints_deg.to_list()
        current_pose_base = start_state.initial_pose_base.copy()
        current_time_s = 0.0
        samples: list[ProgramSimulationSample] = []

        if start_state.initial_sample_motion is not None:
            initial_motion_tool = self._tool_from_pose(start_state.initial_sample_motion.tool_pose)
            samples.append(
                self._build_sample(
                    0.0,
                    start_state.initial_sample_motion,
                    current_joints_deg,
                    initial_motion_tool,
                )
            )

        for motion in start_state.remaining_motions:
            motion_tool = self._tool_from_pose(motion.tool_pose)
            generated_samples = self._simulate_motion(
                motion,
                current_pose_base,
                current_joints_deg,
                current_time_s,
                motion_tool,
            )
            if not generated_samples:
                continue
            samples.extend(generated_samples)
            current_joints_deg = generated_samples[-1].joints_deg.to_list()
            current_pose_base = generated_samples[-1].nominal_pose_base.copy()
            current_time_s = float(generated_samples[-1].time_s)
        return samples

    def _resolve_program_start_state(self, motions: list[RobotProgramMotion]) -> _ProgramStartState:
        fallback_joints_deg = JointAngles6.from_values(self._normalize_joints(self.robot_model.get_joints()))
        fallback_tool = self.tool_model.get_tool() if not motions else self._tool_from_pose(motions[0].tool_pose)
        fallback_pose_base = self._fk_nominal_pose_base(fallback_joints_deg.to_list(), fallback_tool)
        if not motions:
            return _ProgramStartState(
                initial_joints_deg=fallback_joints_deg,
                initial_pose_base=fallback_pose_base,
                remaining_motions=[],
                initial_sample_motion=None,
            )

        first_motion = motions[0]
        first_motion_tool = self._tool_from_pose(first_motion.tool_pose)
        first_target_joints_deg = self._resolve_initial_target_joints(first_motion, fallback_joints_deg.to_list(), first_motion_tool)
        if first_target_joints_deg is None:
            return _ProgramStartState(
                initial_joints_deg=fallback_joints_deg,
                initial_pose_base=fallback_pose_base,
                remaining_motions=motions,
                initial_sample_motion=motions[0],
            )

        initial_joints_deg = JointAngles6.from_values(first_target_joints_deg)
        initial_pose_base = self._fk_nominal_pose_base(initial_joints_deg.to_list(), first_motion_tool)
        return _ProgramStartState(
            initial_joints_deg=initial_joints_deg,
            initial_pose_base=initial_pose_base,
            remaining_motions=motions[1:],
            initial_sample_motion=first_motion,
        )

    def _resolve_initial_target_joints(
        self,
        motion: RobotProgramMotion,
        reference_joints_deg: list[float],
        motion_tool: RobotTool,
    ) -> list[float] | None:
        if motion.target.target_type == RobotProgramTargetType.JOINT:
            return motion.target.joint_angles.to_list()
        target_pose_base = self._target_pose_base(motion, motion.target, motion_tool)
        return self._select_joints_for_pose(target_pose_base, reference_joints_deg, motion_tool)

    def _simulate_motion(
        self,
        motion: RobotProgramMotion,
        current_pose_base: Pose6,
        current_joints_deg: list[float],
        current_time_s: float,
        motion_tool: RobotTool,
    ) -> list[ProgramSimulationSample]:
        if motion.mode == RobotProgramMotionMode.PTP:
            return self._simulate_ptp(motion, current_pose_base, current_joints_deg, current_time_s, motion_tool)
        if motion.mode == RobotProgramMotionMode.LINEAR:
            target_pose_base = self._target_pose_base(motion, motion.target, motion_tool)
            distance_mm = self._distance_xyz_mm(current_pose_base, target_pose_base)
            duration_s = max(self.DEFAULT_DT_S, distance_mm / max(1e-6, self._motion_linear_speed_mps(motion)) / 1000.0)
            intervals = self._cartesian_intervals_for_distance(distance_mm)
            step_time_s = duration_s / max(1, intervals)
            orientation_deltas_deg = [
                self._shortest_angle_delta_deg(current_pose_base.to_list()[3 + axis], target_pose_base.to_list()[3 + axis])
                for axis in range(3)
            ]
            path = [
                self._interpolate_linear_pose(current_pose_base, target_pose_base, index / intervals, orientation_deltas_deg)
                for index in range(intervals + 1)
            ]
            return self._simulate_cartesian_path(motion, path, current_joints_deg, current_time_s, motion_tool, step_time_s)
        if motion.mode == RobotProgramMotionMode.CIRCULAR and motion.via_target is not None:
            via_pose_base = self._target_pose_base(motion, motion.via_target, motion_tool)
            target_pose_base = self._target_pose_base(motion, motion.target, motion_tool)
            approximate_length_mm = (
                self._distance_xyz_mm(current_pose_base, via_pose_base)
                + self._distance_xyz_mm(via_pose_base, target_pose_base)
            )
            duration_s = max(
                self.DEFAULT_DT_S,
                approximate_length_mm / max(1e-6, self._motion_linear_speed_mps(motion)) / 1000.0,
            )
            intervals = self._cartesian_intervals_for_distance(approximate_length_mm)
            step_time_s = duration_s / max(1, intervals)
            path = self._circle_pose_points(current_pose_base, via_pose_base, target_pose_base, intervals)
            return self._simulate_cartesian_path(motion, path, current_joints_deg, current_time_s, motion_tool, step_time_s)
        if motion.mode == RobotProgramMotionMode.EXTERNAL_AXIS:
            return self._simulate_external_axis(motion, current_joints_deg, current_time_s, motion_tool)
        return []

    # =========================================================================
    # Lot C : PTP en une seule passe (sonde légère pour estimer, puis sonde fine)
    # =========================================================================

    def _simulate_ptp(
        self,
        motion: RobotProgramMotion,
        current_pose_base: Pose6,
        current_joints_deg: list[float],
        current_time_s: float,
        motion_tool: RobotTool,
    ) -> list[ProgramSimulationSample]:
        _ = current_pose_base
        target_joints_deg = self._resolve_ptp_target_joints(motion, current_joints_deg, motion_tool)
        if target_joints_deg is None:
            return []
        deltas_deg = self._shortest_joint_path(current_joints_deg, target_joints_deg)
        if deltas_deg is None:
            return []
        speed_ratio = max(0.01, min(1.0, self.DEFAULT_PTP_SPEED_PERCENT / 100.0))
        axis_speed_limits = [max(1e-6, float(speed) * speed_ratio) for speed in self.robot_model.get_axis_speed_limits()[:6]]
        duration_s = max(abs(deltas_deg[axis]) / axis_speed_limits[axis] for axis in range(6))
        coarse_probe_intervals = max(1, int(math.ceil(duration_s / self.DEFAULT_DT_S)))

        # Lot C : sonde légère à 8 points pour estimer la longueur TCP
        light_probe = self._build_ptp_probe_samples(
            current_joints_deg, deltas_deg, current_time_s, duration_s, 8, motion_tool
        )
        estimated_tcp_length_mm = self._probe_path_length_mm(light_probe)

        # Une seule sonde fine combinant les deux critères
        final_intervals = max(coarse_probe_intervals, self._cartesian_intervals_for_distance(estimated_tcp_length_mm))
        probe_samples = self._build_ptp_probe_samples(
            current_joints_deg, deltas_deg, current_time_s, duration_s, final_intervals, motion_tool
        )

        resampled_probe_samples = self._resample_ptp_probe_by_distance(probe_samples)
        return [
            self._build_sample(
                sample.time_s,
                motion,
                sample.joints_deg.to_list(),
                motion_tool,
            )
            for sample in resampled_probe_samples
        ]

    def _build_ptp_probe_samples(
        self,
        current_joints_deg: list[float],
        deltas_deg: list[float],
        current_time_s: float,
        duration_s: float,
        intervals: int,
        motion_tool: RobotTool,
    ) -> list[_PtpProbeSample]:
        safe_intervals = max(1, int(intervals))
        samples: list[_PtpProbeSample] = []
        for index in range(safe_intervals + 1):
            progress = index / safe_intervals
            u = self._quintic(progress)
            joints_deg = [float(current_joints_deg[axis]) + deltas_deg[axis] * u for axis in range(6)]
            joint_angles = JointAngles6.from_values(joints_deg)
            samples.append(
                _PtpProbeSample(
                    time_s=current_time_s + duration_s * progress,
                    joints_deg=joint_angles,
                    pose_base=self._fk_nominal_pose_base(joint_angles.to_list(), motion_tool),
                )
            )
        return samples

    def _probe_path_length_mm(self, probe_samples: list[_PtpProbeSample]) -> float:
        if len(probe_samples) < 2:
            return 0.0
        total_length_mm = 0.0
        previous_pose = probe_samples[0].pose_base
        for sample in probe_samples[1:]:
            total_length_mm += self._distance_xyz_mm(previous_pose, sample.pose_base)
            previous_pose = sample.pose_base
        return total_length_mm

    def _resample_ptp_probe_by_distance(self, probe_samples: list[_PtpProbeSample]) -> list[_PtpProbeSample]:
        if len(probe_samples) <= 1:
            return []

        cumulative_lengths_mm = [0.0]
        for previous_sample, current_sample in zip(probe_samples, probe_samples[1:]):
            cumulative_lengths_mm.append(
                cumulative_lengths_mm[-1] + self._distance_xyz_mm(previous_sample.pose_base, current_sample.pose_base)
            )
        total_length_mm = cumulative_lengths_mm[-1]
        intervals = self._cartesian_intervals_for_distance(total_length_mm)
        if intervals <= 1 or total_length_mm <= 1e-9:
            return [probe_samples[-1]]

        resampled: list[_PtpProbeSample] = []
        right_index = 1
        for interval_index in range(1, intervals + 1):
            target_length_mm = total_length_mm * interval_index / intervals
            while right_index < len(cumulative_lengths_mm) and cumulative_lengths_mm[right_index] < target_length_mm:
                right_index += 1
            right_index = min(right_index, len(cumulative_lengths_mm) - 1)
            left_index = max(0, right_index - 1)
            left_length_mm = cumulative_lengths_mm[left_index]
            right_length_mm = cumulative_lengths_mm[right_index]
            if abs(right_length_mm - left_length_mm) <= 1e-12:
                local_progress = 0.0
            else:
                local_progress = (target_length_mm - left_length_mm) / (right_length_mm - left_length_mm)
            resampled.append(
                self._interpolate_ptp_probe_sample(
                    probe_samples[left_index],
                    probe_samples[right_index],
                    local_progress,
                )
            )
        return resampled

    @staticmethod
    def _interpolate_ptp_probe_sample(
        left_sample: _PtpProbeSample,
        right_sample: _PtpProbeSample,
        local_progress: float,
    ) -> _PtpProbeSample:
        clamped = max(0.0, min(1.0, float(local_progress)))
        interpolated_joints = [
            float(left_value) + (float(right_value) - float(left_value)) * clamped
            for left_value, right_value in zip(left_sample.joints_deg.to_list(), right_sample.joints_deg.to_list())
        ]
        return _PtpProbeSample(
            time_s=float(left_sample.time_s) + (float(right_sample.time_s) - float(left_sample.time_s)) * clamped,
            joints_deg=JointAngles6.from_values(interpolated_joints),
            pose_base=ProgramSimulator._interpolate_linear_pose(
                left_sample.pose_base,
                right_sample.pose_base,
                clamped,
                [
                    ProgramSimulator._shortest_angle_delta_deg(left_sample.pose_base.a, right_sample.pose_base.a),
                    ProgramSimulator._shortest_angle_delta_deg(left_sample.pose_base.b, right_sample.pose_base.b),
                    ProgramSimulator._shortest_angle_delta_deg(left_sample.pose_base.c, right_sample.pose_base.c),
                ],
            ),
        )

    def _simulate_cartesian_path(
        self,
        motion: RobotProgramMotion,
        path: list[Pose6],
        current_joints_deg: list[float],
        current_time_s: float,
        motion_tool: RobotTool,
        step_time_s: float,
    ) -> list[ProgramSimulationSample]:
        previous_joints_deg = list(current_joints_deg)
        samples: list[ProgramSimulationSample] = []
        for index, pose_base in enumerate(path[1:], start=1):
            joints_deg = self._select_joints_for_pose(pose_base, previous_joints_deg, motion_tool)
            if joints_deg is None:
                continue
            previous_joints_deg = list(joints_deg)
            samples.append(
                self._build_sample(
                    current_time_s + index * float(step_time_s),
                    motion,
                    joints_deg,
                    motion_tool,
                )
            )
        return samples

    # =========================================================================
    # Lot B : pas d'interpolation adaptatif (méthode d'instance)
    # =========================================================================

    def _cartesian_intervals_for_distance(self, distance_mm: float) -> int:
        step_mm = max(1e-6, float(self._active_cartesian_step_mm))
        if distance_mm <= step_mm:
            return 1
        return max(1, int(math.ceil(float(distance_mm) / step_mm)))

    def _world_robot_base_for(self, ext_values: dict[tuple[str, int], float]) -> np.ndarray:
        """T_world_robotBase pour l'état d'axes simulé donné."""
        if self.workspace_model is None:
            return np.eye(4, dtype=float)
        workspace_base = np.array(self.workspace_model.get_robot_base_transform_world().matrix, dtype=float)
        if self.external_axes_model is None:
            return workspace_base
        override = self.external_axes_model.get_robot_world_base_matrix_for(ext_values)
        return override if override is not None else workspace_base

    def _piece_frame_world_for(self, ext_values: dict[tuple[str, int], float]) -> np.ndarray | None:
        """T_world_pieceFrame pour l'état d'axes simulé donné. None si aucun modèle pièce."""
        if self.workpiece_model is None or self.workspace_model is None:
            return None
        workspace_base = np.array(self.workspace_model.get_robot_base_transform_world().matrix, dtype=float)
        world_base = self._world_robot_base_for(ext_values)
        world_transforms: dict[str, dict] = {}
        if self.external_axes_model is not None:
            world_transforms = self.external_axes_model.compute_world_transforms_for(ext_values)
        # Pour les repères workspace (PREFIX_WS), on ne les a pas ici — fallback identité
        return piece_frame_world(
            piece_parent_id=self.workpiece_model.get_parent_frame_id(),
            world_transforms=world_transforms,
            piece_pose_in_parent=self.workpiece_model.get_pose_in_parent(),
            piece_frame_pose=self.workpiece_model.get_workpiece_frame_pose(),
            workspace_robot_base_matrix=workspace_base,
            world_robot_base_matrix=world_base,
        )

    def _build_sample(
        self,
        time_s: float,
        motion: RobotProgramMotion,
        joints_deg: list[float],
        motion_tool: RobotTool,
    ) -> ProgramSimulationSample:
        nominal_pose_base = self._fk_nominal_pose_base(joints_deg, motion_tool)
        measured_pose_base = self._fk_measured_pose_base(joints_deg, motion_tool)
        # Pose monde : T_world_robotBase · T_base_tcp
        T_world_robotBase = self._world_robot_base_for(self._current_ext_axis_values)
        T_base_tcp = pose_to_matrix(nominal_pose_base)
        nominal_pose_world = matrix_to_pose(T_world_robotBase @ T_base_tcp)
        return ProgramSimulationSample(
            time_s=float(time_s),
            motion_mode=motion.mode,
            source_line=int(motion.line_number),
            joints_deg=JointAngles6.from_values(joints_deg),
            nominal_pose_base=nominal_pose_base,
            measured_pose_base=measured_pose_base,
            ext_axis_values=self._build_ext_axis_snapshot(),
            nominal_pose_world=nominal_pose_world,
        )

    def _init_ext_axis_state(self) -> None:
        self._current_ext_axis_values = {}
        if self.external_axes_model is None:
            return
        # Initialiser à zéro (home programme) pour une simulation déterministe et auto-portée
        for axis in self.external_axes_model.get_axes():
            for joint_idx in range(len(axis.joints)):
                self._current_ext_axis_values[(axis.id, joint_idx)] = 0.0

    def _build_ext_axis_snapshot(self) -> tuple[float, ...]:
        if self.external_axes_model is None:
            return ()
        return tuple(
            self._current_ext_axis_values.get((axis.id, joint_idx), joint.value)
            for axis in self.external_axes_model.get_axes()
            for joint_idx, joint in enumerate(axis.joints)
        )

    def _update_ext_axis_state_from_snapshot(self, snapshot: tuple[float, ...]) -> None:
        if self.external_axes_model is None:
            return
        idx = 0
        for axis in self.external_axes_model.get_axes():
            for joint_idx in range(len(axis.joints)):
                if idx < len(snapshot):
                    self._current_ext_axis_values[(axis.id, joint_idx)] = snapshot[idx]
                idx += 1

    def _simulate_external_axis(
        self,
        motion: RobotProgramMotion,
        current_joints_deg: list[float],
        current_time_s: float,
        motion_tool: RobotTool,
    ) -> list[ProgramSimulationSample]:
        if motion.external_axis_target is None or not motion.external_axis_target.values:
            return [self._build_sample(current_time_s + self.DEFAULT_DT_S, motion, current_joints_deg, motion_tool)]

        # Calculer la durée totale : max(|delta_i| / speed_i) sur tous les joints
        max_duration_s = self.DEFAULT_DT_S
        for jv in motion.external_axis_target.values:
            start_val = self._current_ext_axis_values.get((jv.axis_id, jv.joint_index), 0.0)
            delta = abs(jv.value - start_val)
            if delta < 1e-9:
                continue

            # Vitesse cible : celle du mouvement si renseignée, sinon default_speed de l'axe
            speed = jv.speed
            if speed is None and self.external_axes_model is not None:
                axis_obj = self.external_axes_model.get_axis(jv.axis_id)
                if axis_obj is not None and jv.joint_index < len(axis_obj.joints):
                    speed = axis_obj.joints[jv.joint_index].default_speed
            speed = float(speed) if speed is not None else 500.0

            # Plafonner à max_speed
            if self.external_axes_model is not None:
                axis_obj = self.external_axes_model.get_axis(jv.axis_id)
                if axis_obj is not None and jv.joint_index < len(axis_obj.joints):
                    speed = min(speed, axis_obj.joints[jv.joint_index].max_speed)
            speed = max(1e-6, speed)

            max_duration_s = max(max_duration_s, delta / speed)

        # Générer les samples linéairement interpolés
        n_steps = max(1, int(math.ceil(max_duration_s / self.DEFAULT_DT_S)))
        start_values = {
            (jv.axis_id, jv.joint_index): self._current_ext_axis_values.get((jv.axis_id, jv.joint_index), 0.0)
            for jv in motion.external_axis_target.values
        }

        samples: list[ProgramSimulationSample] = []
        for step in range(1, n_steps + 1):
            alpha = step / n_steps
            for jv in motion.external_axis_target.values:
                start = start_values[(jv.axis_id, jv.joint_index)]
                self._current_ext_axis_values[(jv.axis_id, jv.joint_index)] = start + (jv.value - start) * alpha
            t = current_time_s + max_duration_s * alpha
            samples.append(self._build_sample(t, motion, current_joints_deg, motion_tool))

        # Fixer les valeurs finales exactes
        for jv in motion.external_axis_target.values:
            self._current_ext_axis_values[(jv.axis_id, jv.joint_index)] = jv.value

        return samples

    def _resolve_ptp_target_joints(
        self,
        motion: RobotProgramMotion,
        current_joints_deg: list[float],
        motion_tool: RobotTool,
    ) -> list[float] | None:
        if motion.target.target_type == RobotProgramTargetType.JOINT:
            return motion.target.joint_angles.to_list()
        target_pose_base = self._target_pose_base(motion, motion.target, motion_tool)
        return self._select_joints_for_pose(target_pose_base, current_joints_deg, motion_tool)

    def _select_joints_for_pose(self, target_pose_base: Pose6, reference_joints_deg: list[float], motion_tool: RobotTool) -> list[float] | None:
        mgi_result = self.robot_model.compute_ik_target(target_pose_base, tool=motion_tool)
        if mgi_result is None:
            return None
        best_solution = mgi_result.get_best_solution_from_current(
            [math.radians(float(value)) for value in reference_joints_deg[:6]],
            self.robot_model.get_joint_weights(),
        )
        if best_solution is None:
            return None
        return self._normalize_joints(best_solution[1].joints)

    # =========================================================================
    # Lot A.2+A.3 : compensation avec MGI réutilisé par tool et cache IK
    # =========================================================================

    def _build_compensated_program(
        self,
        program: RobotProgram,
        output_mode: ProgramCompensationOutputMode,
        measured_dh: list[list[float]],
    ) -> RobotProgram | None:
        corrected_motions: list[RobotProgramMotion] = []
        current_joints_deg = self._normalize_joints(self.robot_model.get_joints())
        previous_reference_joints_deg = list(current_joints_deg)

        # Lot A.2 : un solveur MGI mesuré par tool (clé = tuple des 6 composantes)
        mgi_solvers_by_tool: dict[tuple[float, ...], MGI] = {}
        # Lot A.3 : cache IK local pour cette passe de compensation
        ik_cache: dict[tuple, list[float]] = {}

        for motion in program.motions:
            motion_tool = self._tool_from_pose(motion.tool_pose)
            if motion.mode == RobotProgramMotionMode.PTP and motion.target.target_type == RobotProgramTargetType.JOINT:
                corrected_motions.append(motion)
                previous_reference_joints_deg = motion.target.joint_angles.to_list()
                continue

            # Récupérer ou créer le solveur MGI pour ce tool
            tool_key = (motion_tool.x, motion_tool.y, motion_tool.z, motion_tool.a, motion_tool.b, motion_tool.c)
            solver = mgi_solvers_by_tool.get(tool_key)
            if solver is None:
                solver = self._build_measured_geometry_mgi(measured_dh, motion_tool)
                mgi_solvers_by_tool[tool_key] = solver

            if motion.mode == RobotProgramMotionMode.CIRCULAR and motion.via_target is not None:
                via_result = self._build_compensated_motion_variant(
                    motion,
                    motion.via_target,
                    output_mode,
                    previous_reference_joints_deg,
                    motion_tool,
                    measured_dh,
                    solver,
                    ik_cache,
                    replace_mode=RobotProgramMotionMode.PTP if output_mode == ProgramCompensationOutputMode.ARTICULAR else motion.mode,
                )
                if via_result is not None:
                    corrected_via_motion, via_q_measured = via_result
                    if output_mode == ProgramCompensationOutputMode.ARTICULAR:
                        corrected_motions.append(corrected_via_motion)
                        previous_reference_joints_deg = via_q_measured
                    elif corrected_via_motion.via_target is not None:
                        motion = replace(motion, via_target=corrected_via_motion.via_target)
                        previous_reference_joints_deg = via_q_measured

            result = self._build_compensated_motion_variant(
                motion,
                motion.target,
                output_mode,
                previous_reference_joints_deg,
                motion_tool,
                measured_dh,
                solver,
                ik_cache,
            )
            if result is None:
                corrected_motions.append(motion)
                if motion.target.target_type == RobotProgramTargetType.JOINT:
                    previous_reference_joints_deg = motion.target.joint_angles.to_list()
                continue
            corrected_motion, q_measured = result
            corrected_motions.append(corrected_motion)
            previous_reference_joints_deg = q_measured

        return RobotProgram(
            brand=program.brand,
            source_path=program.source_path,
            source_text=program.source_text,
            motions=corrected_motions,
            warnings=list(program.warnings),
        )

    def _build_compensated_motion_variant(
        self,
        motion: RobotProgramMotion,
        target: RobotProgramTarget,
        output_mode: ProgramCompensationOutputMode,
        previous_reference_joints_deg: list[float],
        motion_tool: RobotTool,
        measured_dh: list[list[float]],
        solver: MGI,
        ik_cache: dict[tuple, list[float]],
        replace_mode: RobotProgramMotionMode | None = None,
    ) -> tuple[RobotProgramMotion, list[float]] | None:
        if target.target_type != RobotProgramTargetType.CARTESIAN:
            return None

        target_pose_base = self._target_pose_base(motion, target, motion_tool)
        tool_key = (motion_tool.x, motion_tool.y, motion_tool.z, motion_tool.a, motion_tool.b, motion_tool.c)
        cache_key = (
            round(target_pose_base.x, 4), round(target_pose_base.y, 4), round(target_pose_base.z, 4),
            round(target_pose_base.a, 3), round(target_pose_base.b, 3), round(target_pose_base.c, 3),
            tool_key, output_mode.value,
        )
        cached = ik_cache.get(cache_key)
        if cached is not None:
            q_target_measured_deg = cached
        else:
            q_target_measured_deg = self._select_joints_for_measured_geometry_pose(
                target_pose_base, previous_reference_joints_deg, motion_tool, measured_dh, solver
            )
            if q_target_measured_deg is None:
                return None
            ik_cache[cache_key] = q_target_measured_deg

        if output_mode == ProgramCompensationOutputMode.ARTICULAR:
            motion_out = RobotProgramMotion(
                mode=replace_mode or RobotProgramMotionMode.PTP,
                target=RobotProgramTarget(
                    target_type=RobotProgramTargetType.JOINT,
                    joint_angles=JointAngles6.from_values(q_target_measured_deg),
                ),
                line_number=motion.line_number,
                source=motion.source,
                base_pose=motion.base_pose.copy(),
                tool_pose=motion.tool_pose.copy(),
                cp_speed_mps=motion.cp_speed_mps,
            )
            return motion_out, q_target_measured_deg

        corrected_pose_base = self._fk_nominal_pose_base(q_target_measured_deg, motion_tool)
        corrected_pose_program_base = self._pose_from_robot_base_to_program_base(corrected_pose_base, motion.base_pose)
        corrected_target = RobotProgramTarget(
            target_type=RobotProgramTargetType.CARTESIAN,
            cartesian_pose=corrected_pose_program_base,
        )
        if motion.mode == RobotProgramMotionMode.CIRCULAR and target is motion.via_target:
            motion_out = RobotProgramMotion(
                mode=motion.mode,
                target=motion.target,
                via_target=corrected_target,
                line_number=motion.line_number,
                source=motion.source,
                base_pose=motion.base_pose.copy(),
                tool_pose=motion.tool_pose.copy(),
                cp_speed_mps=motion.cp_speed_mps,
            )
        else:
            motion_out = replace(motion, target=corrected_target)
        return motion_out, q_target_measured_deg

    def _select_joints_for_measured_geometry_pose(
        self,
        target_pose_base: Pose6,
        reference_joints_deg: list[float],
        motion_tool: RobotTool,
        measured_dh: list[list[float]],
        solver: MGI,
    ) -> list[float] | None:
        # Lot A.2 : le solveur est fourni (déjà construit pour ce tool), on met à jour les valeurs de singularité
        solver.set_q1ValueIfSingularityQ1Deg(reference_joints_deg[0])
        solver.set_q4ValueIfSingularityQ5Deg(reference_joints_deg[3])
        solver.set_q6ValueIfSingularityQ5Deg(reference_joints_deg[5])
        result = solver.compute_mgi_target(target_pose_base.to_tuple(), returnDegrees=True)
        best_solution = result.get_best_solution_from_current(
            [math.radians(float(value)) for value in reference_joints_deg[:6]],
            self.robot_model.get_joint_weights(),
        )
        if best_solution is None:
            return None
        return self._normalize_joints(best_solution[1].joints)

    def _build_measured_geometry_mgi(self, measured_dh: list[list[float]], motion_tool: RobotTool) -> MGI:
        params = MgiParams(
            self.robot_model.get_config_identifier(),
            self._measured_geometric_params(measured_dh),
            invert_table=[axis == -1 for axis in self.robot_model.get_axis_reversed()],
            axis_limits=MgiAxisLimits(False, self.robot_model.get_axis_limits()),
            configuration_filter=MgiConfigurationFilter.allow_all(),
        )
        solver = MGI(params, motion_tool)
        current_joints_deg = self._normalize_joints(self.robot_model.get_joints())
        solver.set_q1ValueIfSingularityQ1Deg(current_joints_deg[0])
        solver.set_q4ValueIfSingularityQ5Deg(current_joints_deg[3])
        solver.set_q6ValueIfSingularityQ5Deg(current_joints_deg[5])
        return solver

    # =========================================================================
    # Lot A.1 : normalisation DH mesurée (brut + accesseur avec cache)
    # =========================================================================

    def _normalized_measured_dh_table(self) -> list[list[float]] | None:
        if self._cached_measured_dh is not None:
            return self._cached_measured_dh
        return self._compute_normalized_measured_dh_table()

    def _compute_normalized_measured_dh_table(self) -> list[list[float]] | None:
        raw_table = self.robot_model.get_measured_dh_params()
        if len(raw_table) < 6:
            return None
        normalized: list[list[float]] = []
        has_non_zero_value = False
        for raw_row in raw_table[:6]:
            row = [float(value) for value in raw_row[:4]]
            while len(row) < 4:
                row.append(0.0)
            if any(abs(value) > 1e-9 for value in row):
                has_non_zero_value = True
            normalized.append(row)
        if not has_non_zero_value:
            return None
        return normalized

    # =========================================================================
    # Lot F : FK mesuré vectorisé avec pré-extraction numpy
    # =========================================================================

    def _rebuild_measured_dh_arrays(self) -> None:
        """Pré-extrait les paramètres DH mesurés en arrays numpy pour éviter les conversions répétées."""
        if self._cached_measured_dh is None:
            self._cached_measured_dh_arrays = None
            self._cached_axis_reversed = None
            return
        dh = self._cached_measured_dh
        self._cached_measured_dh_arrays = (
            np.radians([row[0] for row in dh]),   # alpha en radians
            np.array([row[1] for row in dh]),       # d en mm
            np.radians([row[2] for row in dh]),     # theta_offset en radians
            np.array([row[3] for row in dh]),       # r en mm
        )
        self._cached_axis_reversed = np.array(self.robot_model.get_axis_reversed()[:6], dtype=float)

    def _fk_nominal_pose_base(self, joints_deg: list[float], motion_tool: RobotTool) -> Pose6:
        fk_result = self.robot_model.compute_fk_joints(joints_deg, tool=motion_tool)
        if fk_result is None:
            return Pose6.zeros()
        return fk_result.dh_pose.copy()

    def _fk_measured_pose_base(self, joints_deg: list[float], motion_tool: RobotTool) -> Pose6 | None:
        # Lot F : utiliser les arrays numpy pré-extraits si disponibles
        if self._cached_measured_dh_arrays is not None and self._cached_axis_reversed is not None:
            alpha_arr, d_arr, theta_offset_arr, r_arr = self._cached_measured_dh_arrays
            joints_arr = np.asarray(joints_deg[:6], dtype=float)
            theta_arr = theta_offset_arr + np.radians(joints_arr * self._cached_axis_reversed)
            transform = np.eye(4, dtype=float)
            for axis in range(6):
                transform = transform @ math_utils.dh_modified(
                    float(alpha_arr[axis]), float(d_arr[axis]), float(theta_arr[axis]), float(r_arr[axis])
                )
            transform = transform @ RobotModel.build_tool_transform(motion_tool)
            return matrix_to_pose(transform)

        # Fallback sans cache (hors passe de simulation)
        measured_dh = self._normalized_measured_dh_table()
        if measured_dh is None:
            return None
        transform = np.eye(4, dtype=float)
        axis_reversed = self.robot_model.get_axis_reversed()
        for axis in range(6):
            alpha_deg, d_mm, theta_offset_deg, r_mm = measured_dh[axis]
            theta_rad = math.radians(theta_offset_deg + joints_deg[axis] * float(axis_reversed[axis]))
            transform = transform @ math_utils.dh_modified(
                math.radians(alpha_deg), d_mm, theta_rad, r_mm,
            )
        transform = transform @ RobotModel.build_tool_transform(motion_tool)
        return matrix_to_pose(transform)

    def _target_pose_base(self, motion: RobotProgramMotion, target: RobotProgramTarget, motion_tool: RobotTool) -> Pose6:
        if target.target_type == RobotProgramTargetType.JOINT:
            return self._fk_nominal_pose_base(target.joint_angles.to_list(), motion_tool)
        # Chemin monde : cible en repère pièce → monde → base robot courant
        T_world_pieceFrame = self._piece_frame_world_for(self._current_ext_axis_values)
        if T_world_pieceFrame is None:
            # Sans modèle pièce/workspace : chemin legacy (base_pose statique)
            return self._pose_from_program_base_to_robot_base(target.cartesian_pose, motion.base_pose)
        T_world_robotBase = self._world_robot_base_for(self._current_ext_axis_values)
        T_robotBase_world = invert_homogeneous_transform(T_world_robotBase)
        T_target_in_piece = pose_to_matrix(target.cartesian_pose)
        T_target_in_base = T_robotBase_world @ T_world_pieceFrame @ T_target_in_piece
        return matrix_to_pose(T_target_in_base)

    @staticmethod
    def _pose_from_program_base_to_robot_base(pose_program_base: Pose6, base_pose: Pose6) -> Pose6:
        transform = pose_to_matrix(base_pose) @ pose_to_matrix(pose_program_base)
        return matrix_to_pose(transform)

    @staticmethod
    def _pose_from_robot_base_to_program_base(pose_robot_base: Pose6, base_pose: Pose6) -> Pose6:
        transform = np.linalg.inv(pose_to_matrix(base_pose)) @ pose_to_matrix(pose_robot_base)
        return matrix_to_pose(transform)

    @staticmethod
    def _tool_from_pose(tool_pose: Pose6) -> RobotTool:
        return RobotTool(tool_pose.x, tool_pose.y, tool_pose.z, tool_pose.a, tool_pose.b, tool_pose.c)

    @staticmethod
    def _tool_to_pose(tool: RobotTool) -> Pose6:
        return Pose6(tool.x, tool.y, tool.z, tool.a, tool.b, tool.c)

    @staticmethod
    def _normalize_joints(joints_deg: list[float]) -> list[float]:
        normalized = [float(value) for value in joints_deg[:6]]
        while len(normalized) < 6:
            normalized.append(0.0)
        return normalized

    @staticmethod
    def _measured_geometric_params(measured_dh: list[list[float]]) -> MgiGeometricParams:
        return MgiGeometricParams(
            measured_dh[0][3],
            measured_dh[1][1],
            measured_dh[2][1],
            measured_dh[3][1],
            measured_dh[3][3],
            measured_dh[5][3],
        )

    @staticmethod
    def _quintic(u: float) -> float:
        clamped = max(0.0, min(1.0, float(u)))
        return 10.0 * clamped**3 - 15.0 * clamped**4 + 6.0 * clamped**5

    @staticmethod
    def _shortest_joint_path(from_joints_deg: list[float], to_joints_deg: list[float]) -> list[float] | None:
        if len(from_joints_deg) < 6 or len(to_joints_deg) < 6:
            return None
        deltas_deg: list[float] = []
        for axis in range(6):
            delta_deg = (float(to_joints_deg[axis]) - float(from_joints_deg[axis]) + 180.0) % 360.0 - 180.0
            if delta_deg == -180.0 and (float(to_joints_deg[axis]) - float(from_joints_deg[axis])) > 0.0:
                delta_deg = 180.0
            deltas_deg.append(delta_deg)
        return deltas_deg

    @staticmethod
    def _shortest_angle_delta_deg(from_deg: float, to_deg: float) -> float:
        delta_deg = (float(to_deg) - float(from_deg) + 180.0) % 360.0 - 180.0
        if delta_deg == -180.0 and (float(to_deg) - float(from_deg)) > 0.0:
            return 180.0
        return delta_deg

    @staticmethod
    def _interpolate_linear_pose(from_pose: Pose6, to_pose: Pose6, u: float, orientation_deltas_deg: list[float]) -> Pose6:
        clamped = max(0.0, min(1.0, float(u)))
        return Pose6(
            from_pose.x + (to_pose.x - from_pose.x) * clamped,
            from_pose.y + (to_pose.y - from_pose.y) * clamped,
            from_pose.z + (to_pose.z - from_pose.z) * clamped,
            from_pose.a + orientation_deltas_deg[0] * clamped,
            from_pose.b + orientation_deltas_deg[1] * clamped,
            from_pose.c + orientation_deltas_deg[2] * clamped,
        )

    def _circle_pose_points(self, start_pose: Pose6, via_pose: Pose6, end_pose: Pose6, intervals: int) -> list[Pose6]:
        start_xyz = np.array([start_pose.x, start_pose.y, start_pose.z], dtype=float)
        via_xyz = np.array([via_pose.x, via_pose.y, via_pose.z], dtype=float)
        end_xyz = np.array([end_pose.x, end_pose.y, end_pose.z], dtype=float)
        vector_1 = via_xyz - start_xyz
        vector_2 = end_xyz - start_xyz
        normal = np.cross(vector_1, vector_2)
        normal_norm = float(np.linalg.norm(normal))
        if normal_norm <= 1e-9:
            return [start_pose.copy(), end_pose.copy()]
        normal = normal / normal_norm
        ex = self._normalize_vector(vector_1)
        ey = self._normalize_vector(np.cross(normal, ex))

        x1 = float(np.dot(via_xyz - start_xyz, ex))
        y1 = float(np.dot(via_xyz - start_xyz, ey))
        x2 = float(np.dot(end_xyz - start_xyz, ex))
        y2 = float(np.dot(end_xyz - start_xyz, ey))
        determinant = 2.0 * (x1 * y2 - y1 * x2)
        if abs(determinant) <= 1e-9:
            return [start_pose.copy(), end_pose.copy()]

        center_x = ((x1 * x1 + y1 * y1) * y2 - (x2 * x2 + y2 * y2) * y1) / determinant
        center_y = (x1 * (x2 * x2 + y2 * y2) - x2 * (x1 * x1 + y1 * y1)) / determinant
        center_xyz = start_xyz + center_x * ex + center_y * ey
        radius = float(np.linalg.norm(start_xyz - center_xyz))

        def angle(point_xyz: np.ndarray) -> float:
            relative = point_xyz - center_xyz
            return math.atan2(float(np.dot(relative, ey)), float(np.dot(relative, ex)))

        angle_start = angle(start_xyz)
        angle_via = angle(via_xyz)
        angle_end = angle(end_xyz)
        ccw_span = (angle_end - angle_start) % (2.0 * math.pi)
        via_ccw = (angle_via - angle_start) % (2.0 * math.pi)
        span = ccw_span if via_ccw <= ccw_span else ccw_span - 2.0 * math.pi

        orientation_deltas_deg = [
            self._shortest_angle_delta_deg(start_pose.to_list()[3 + axis], end_pose.to_list()[3 + axis])
            for axis in range(3)
        ]
        poses: list[Pose6] = []
        safe_intervals = max(1, int(intervals))
        for index in range(safe_intervals + 1):
            u = index / safe_intervals
            theta = angle_start + span * u
            xyz = center_xyz + radius * math.cos(theta) * ex + radius * math.sin(theta) * ey
            interpolated = self._interpolate_linear_pose(start_pose, end_pose, u, orientation_deltas_deg)
            poses.append(Pose6(float(xyz[0]), float(xyz[1]), float(xyz[2]), interpolated.a, interpolated.b, interpolated.c))
        return poses

    @staticmethod
    def _normalize_vector(vector: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-12:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return vector / norm

    @staticmethod
    def _cumulative_arc_lengths_mm(points_xyz: list[list[float]]) -> list[float]:
        lengths: list[float] = [0.0]
        for a, b in zip(points_xyz, points_xyz[1:]):
            lengths.append(lengths[-1] + math.sqrt(sum((float(b[i]) - float(a[i])) ** 2 for i in range(3))))
        return lengths

    @staticmethod
    def _xyz_distance_mm(a: list[float], b: list[float]) -> float:
        return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))

    @staticmethod
    def _interp_xyz_at_arc_length(
        points_xyz: list[list[float]],
        arc_lengths: list[float],
        target_length: float,
    ) -> list[float]:
        if len(points_xyz) == 1:
            return list(points_xyz[0])
        clamped = max(0.0, min(float(arc_lengths[-1]), float(target_length)))
        right = bisect_left(arc_lengths, clamped)
        right = min(right, len(arc_lengths) - 1)
        left = max(0, right - 1)
        left_len = arc_lengths[left]
        right_len = arc_lengths[right]
        if right_len - left_len <= 1e-9:
            return list(points_xyz[right])
        alpha = (clamped - left_len) / (right_len - left_len)
        return [float(points_xyz[left][i]) + alpha * (float(points_xyz[right][i]) - float(points_xyz[left][i])) for i in range(3)]

    @staticmethod
    def _motion_linear_speed_mps(motion: RobotProgramMotion) -> float:
        if motion.cp_speed_mps is not None and motion.cp_speed_mps > 1e-9:
            return float(motion.cp_speed_mps)
        return ProgramSimulator.DEFAULT_LINEAR_SPEED_MPS

    @staticmethod
    def _distance_xyz_mm(from_pose: Pose6, to_pose: Pose6) -> float:
        return math.sqrt(
            (to_pose.x - from_pose.x) ** 2 +
            (to_pose.y - from_pose.y) ** 2 +
            (to_pose.z - from_pose.z) ** 2
        )

    @staticmethod
    def _pose_xyz(pose: Pose6) -> list[float]:
        return [float(pose.x), float(pose.y), float(pose.z)]

    def _build_path_from_samples(self, samples: list[ProgramSimulationSample], use_measured: bool) -> tuple[list[float], list[list[float]]] | None:
        if not samples:
            return None
        points_xyz: list[list[float]] = []
        for sample in samples:
            pose = sample.measured_pose_base if use_measured else sample.nominal_pose_base
            if pose is None:
                continue
            points_xyz.append(self._pose_xyz(pose))
        if not points_xyz:
            return None
        return self._progresses_from_xyz(points_xyz), points_xyz

    @staticmethod
    def _progresses_from_xyz(points_xyz: list[list[float]]) -> list[float]:
        if not points_xyz:
            return []
        if len(points_xyz) == 1:
            return [0.0]
        cumulative_lengths_mm = [0.0]
        for previous_xyz, current_xyz in zip(points_xyz, points_xyz[1:]):
            cumulative_lengths_mm.append(
                cumulative_lengths_mm[-1] + math.sqrt(sum((float(current_xyz[axis]) - float(previous_xyz[axis])) ** 2 for axis in range(3)))
            )
        total_length_mm = cumulative_lengths_mm[-1]
        if total_length_mm <= 1e-9:
            return [index / max(1, len(points_xyz) - 1) for index in range(len(points_xyz))]
        return [value / total_length_mm for value in cumulative_lengths_mm]

    def _interpolate_path_xyz(self, path: tuple[list[float], list[list[float]]] | None, progress: float) -> list[float] | None:
        if path is None:
            return None
        progresses, points_xyz = path
        if not points_xyz:
            return None
        if len(points_xyz) == 1:
            return list(points_xyz[0])
        clamped_progress = max(0.0, min(1.0, float(progress)))
        if clamped_progress <= progresses[0]:
            return list(points_xyz[0])
        if clamped_progress >= progresses[-1]:
            return list(points_xyz[-1])
        right_index = bisect_left(progresses, clamped_progress)
        right_index = min(right_index, len(progresses) - 1)
        left_index = max(0, right_index - 1)
        progress_left = progresses[left_index]
        progress_right = progresses[right_index]
        if progress_right - progress_left <= 1e-9:
            return list(points_xyz[right_index])
        alpha = (clamped_progress - progress_left) / (progress_right - progress_left)
        return [
            float(points_xyz[left_index][axis]) + alpha * (float(points_xyz[right_index][axis]) - float(points_xyz[left_index][axis]))
            for axis in range(3)
        ]

    @staticmethod
    def _path_length_mm(points_xyz: list[list[float]]) -> float:
        if len(points_xyz) < 2:
            return 0.0
        total_length_mm = 0.0
        for previous_xyz, current_xyz in zip(points_xyz, points_xyz[1:]):
            total_length_mm += math.sqrt(sum((float(current_xyz[axis]) - float(previous_xyz[axis])) ** 2 for axis in range(3)))
        return total_length_mm
