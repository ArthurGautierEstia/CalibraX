from __future__ import annotations

from bisect import bisect_left

from dataclasses import dataclass, replace
import math

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
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.types import JointAngles6, Pose6
from utils.mgi import MGI, MgiAxisLimits, MgiConfigKey, MgiConfigurationFilter, MgiGeometricParams, MgiParams, RobotTool
from utils.reference_frame_utils import pose_to_matrix, matrix_to_pose


@dataclass(frozen=True)
class _PtpProbeSample:
    time_s: float
    joints_deg: JointAngles6
    pose_base: Pose6


class ProgramSimulator:
    DEFAULT_DT_S = 0.02
    DEFAULT_LINEAR_SPEED_MPS = 0.2
    DEFAULT_PTP_SPEED_PERCENT = 50.0
    CARTESIAN_SAMPLE_STEP_MM = 1.0

    def __init__(self, robot_model: RobotModel, tool_model: ToolModel) -> None:
        self.robot_model = robot_model
        self.tool_model = tool_model

    def simulate_program(self, program: RobotProgram, include_compensation: bool = True) -> ProgramSimulationResult:
        if program.brand != RobotProgramBrand.KUKA:
            return ProgramSimulationResult(warnings=["Format de programme non supporte."])
        if not self.robot_model.get_has_configuration():
            return ProgramSimulationResult(warnings=["Charger une configuration robot avant de simuler un programme."])

        nominal_samples = self._simulate_motion_list(program.motions)
        warnings = list(program.warnings)
        cartesian_program: RobotProgram | None = None
        articular_program: RobotProgram | None = None
        cartesian_samples: list[ProgramSimulationSample] = []
        articular_samples: list[ProgramSimulationSample] = []

        measured_dh = self._normalized_measured_dh_table()
        if measured_dh is None:
            warnings.append("Aucun modele mesure disponible: trajectoire reelle et compensation indisponibles.")
        elif include_compensation:
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

    def build_error_curves(
        self,
        nominal_samples: list[ProgramSimulationSample],
        compensated_samples: list[ProgramSimulationSample],
    ) -> tuple[list[float], list[float], list[float]]:
        nominal_ok = [sample for sample in nominal_samples if sample.measured_pose_base is not None]
        if not nominal_ok:
            return [], [], []

        nominal_xyz = [self._pose_xyz(sample.nominal_pose_base) for sample in nominal_ok]
        nominal_progresses = self._progresses_from_xyz(nominal_xyz)
        total_length_mm = self._path_length_mm(nominal_xyz)

        measured_xyz_list = [self._pose_xyz(sample.measured_pose_base) for sample in nominal_ok]

        # Paramètrisation arc length des samples compensés en espace nominal DH (même espace que nominal),
        # mais position réelle pour l'erreur en espace measured DH.
        comp_nominal_xyz: list[list[float]] = []
        comp_actual_xyz: list[list[float]] = []
        for sample in compensated_samples:
            if sample.nominal_pose_base is None:
                continue
            comp_nominal_xyz.append(self._pose_xyz(sample.nominal_pose_base))
            actual_pose = sample.measured_pose_base if sample.measured_pose_base is not None else sample.nominal_pose_base
            comp_actual_xyz.append(self._pose_xyz(actual_pose))

        compensated_progresses = self._progresses_from_xyz(comp_nominal_xyz) if comp_nominal_xyz else []

        abscissa_mm: list[float] = []
        measured_error_y_mm: list[float] = []
        compensated_error_y_mm: list[float] = []

        for nominal_xyz_point, progress, measured_xyz in zip(nominal_xyz, nominal_progresses, measured_xyz_list):
            abscissa_mm.append(progress * total_length_mm)
            measured_error_y_mm.append(float(measured_xyz[1]) - float(nominal_xyz_point[1]))
            if comp_actual_xyz:
                compensated_xyz = self._interpolate_path_xyz(
                    (compensated_progresses, comp_actual_xyz), progress
                )
                compensated_error_y_mm.append(
                    0.0 if compensated_xyz is None else float(compensated_xyz[1]) - float(nominal_xyz_point[1])
                )
        return abscissa_mm, measured_error_y_mm, compensated_error_y_mm

    def _simulate_motion_list(self, motions: list[RobotProgramMotion]) -> list[ProgramSimulationSample]:
        current_joints_deg = self._normalize_joints(self.robot_model.get_joints())
        initial_tool = self.tool_model.get_tool() if not motions else self._tool_from_pose(motions[0].tool_pose)
        current_pose_base = self._fk_nominal_pose_base(current_joints_deg, initial_tool)
        current_time_s = 0.0
        samples: list[ProgramSimulationSample] = []

        for motion in motions:
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
        return []

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
        probe_samples = self._build_ptp_probe_samples(
            current_joints_deg,
            deltas_deg,
            current_time_s,
            duration_s,
            coarse_probe_intervals,
            motion_tool,
        )
        estimated_tcp_length_mm = self._probe_path_length_mm(probe_samples)
        refined_probe_intervals = max(coarse_probe_intervals, self._cartesian_intervals_for_distance(estimated_tcp_length_mm))
        if refined_probe_intervals != coarse_probe_intervals:
            probe_samples = self._build_ptp_probe_samples(
                current_joints_deg,
                deltas_deg,
                current_time_s,
                duration_s,
                refined_probe_intervals,
                motion_tool,
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

    @classmethod
    def _cartesian_intervals_for_distance(cls, distance_mm: float) -> int:
        step_mm = max(1e-6, float(cls.CARTESIAN_SAMPLE_STEP_MM))
        if distance_mm <= step_mm:
            return 1
        return max(1, int(math.ceil(float(distance_mm) / step_mm)))

    def _build_sample(
        self,
        time_s: float,
        motion: RobotProgramMotion,
        joints_deg: list[float],
        motion_tool: RobotTool,
    ) -> ProgramSimulationSample:
        nominal_pose_base = self._fk_nominal_pose_base(joints_deg, motion_tool)
        measured_pose_base = self._fk_measured_pose_base(joints_deg, motion_tool)
        return ProgramSimulationSample(
            time_s=float(time_s),
            motion_mode=motion.mode,
            source_line=int(motion.line_number),
            joints_deg=JointAngles6.from_values(joints_deg),
            nominal_pose_base=nominal_pose_base,
            measured_pose_base=measured_pose_base,
        )

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

    def _build_compensated_program(
        self,
        program: RobotProgram,
        output_mode: ProgramCompensationOutputMode,
        measured_dh: list[list[float]],
    ) -> RobotProgram | None:
        corrected_motions: list[RobotProgramMotion] = []
        current_joints_deg = self._normalize_joints(self.robot_model.get_joints())
        previous_reference_joints_deg = list(current_joints_deg)

        for motion in program.motions:
            motion_tool = self._tool_from_pose(motion.tool_pose)
            if motion.mode == RobotProgramMotionMode.PTP and motion.target.target_type == RobotProgramTargetType.JOINT:
                corrected_motions.append(motion)
                current_joints_deg = motion.target.joint_angles.to_list()
                previous_reference_joints_deg = list(current_joints_deg)
                continue

            if motion.mode == RobotProgramMotionMode.CIRCULAR and motion.via_target is not None:
                corrected_via_motion = self._build_compensated_motion_variant(
                    motion,
                    motion.via_target,
                    output_mode,
                    previous_reference_joints_deg,
                    motion_tool,
                    measured_dh,
                    replace_mode=RobotProgramMotionMode.PTP if output_mode == ProgramCompensationOutputMode.ARTICULAR else motion.mode,
                )
                if corrected_via_motion is not None:
                    if output_mode == ProgramCompensationOutputMode.ARTICULAR:
                        corrected_motions.append(corrected_via_motion)
                        previous_reference_joints_deg = corrected_via_motion.target.joint_angles.to_list()
                    elif corrected_via_motion.via_target is not None:
                        motion = replace(motion, via_target=corrected_via_motion.via_target)

            corrected_motion = self._build_compensated_motion_variant(
                motion,
                motion.target,
                output_mode,
                previous_reference_joints_deg,
                motion_tool,
                measured_dh,
            )
            if corrected_motion is None:
                corrected_motions.append(motion)
                if motion.target.target_type == RobotProgramTargetType.JOINT:
                    previous_reference_joints_deg = motion.target.joint_angles.to_list()
                continue
            corrected_motions.append(corrected_motion)
            if corrected_motion.target.target_type == RobotProgramTargetType.JOINT:
                previous_reference_joints_deg = corrected_motion.target.joint_angles.to_list()

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
        replace_mode: RobotProgramMotionMode | None = None,
    ) -> RobotProgramMotion | None:
        if target.target_type != RobotProgramTargetType.CARTESIAN:
            return None

        target_pose_base = self._target_pose_base(motion, target, motion_tool)
        q_seed_deg = self._select_joints_for_measured_geometry_pose(target_pose_base, previous_reference_joints_deg, motion_tool, measured_dh)
        if q_seed_deg is None:
            return None

        corrected_joints_deg = self._apply_measured_theta_offsets_deg(q_seed_deg, measured_dh)
        if corrected_joints_deg is None:
            return None

        if output_mode == ProgramCompensationOutputMode.ARTICULAR:
            return RobotProgramMotion(
                mode=replace_mode or RobotProgramMotionMode.PTP,
                target=RobotProgramTarget(
                    target_type=RobotProgramTargetType.JOINT,
                    joint_angles=JointAngles6.from_values(corrected_joints_deg),
                ),
                line_number=motion.line_number,
                source=motion.source,
                base_pose=motion.base_pose.copy(),
                tool_pose=motion.tool_pose.copy(),
                cp_speed_mps=motion.cp_speed_mps,
            )

        corrected_pose_base = self._fk_nominal_pose_base(corrected_joints_deg, motion_tool)
        corrected_pose_program_base = self._pose_from_robot_base_to_program_base(corrected_pose_base, motion.base_pose)
        corrected_target = RobotProgramTarget(
            target_type=RobotProgramTargetType.CARTESIAN,
            cartesian_pose=corrected_pose_program_base,
        )
        if motion.mode == RobotProgramMotionMode.CIRCULAR and target is motion.via_target:
            return RobotProgramMotion(
                mode=motion.mode,
                target=motion.target,
                via_target=corrected_target,
                line_number=motion.line_number,
                source=motion.source,
                base_pose=motion.base_pose.copy(),
                tool_pose=motion.tool_pose.copy(),
                cp_speed_mps=motion.cp_speed_mps,
            )
        return replace(motion, target=corrected_target)

    def _select_joints_for_measured_geometry_pose(
        self,
        target_pose_base: Pose6,
        reference_joints_deg: list[float],
        motion_tool: RobotTool,
        measured_dh: list[list[float]],
    ) -> list[float] | None:
        solver = self._build_measured_geometry_mgi(measured_dh, motion_tool)
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

    def _apply_measured_theta_offsets_deg(self, theoretical_joints_deg: list[float], measured_dh: list[list[float]]) -> list[float] | None:
        corrected_joints_deg = [float(value) for value in theoretical_joints_deg[:6]]
        axis_reversed = self.robot_model.get_axis_reversed()
        axis_limits = self.robot_model.get_axis_limits()
        for axis in range(6):
            reverse = float(axis_reversed[axis])
            if abs(reverse) <= 1e-12:
                return None
            theoretical_offset_deg = float(self.robot_model.get_dh_param(axis, 2))
            measured_offset_deg = float(measured_dh[axis][2])
            delta_joint_deg = (theoretical_offset_deg - measured_offset_deg) / reverse
            fitted = self._fit_joint_near_reference(
                theoretical_joints_deg[axis],
                theoretical_joints_deg[axis] + delta_joint_deg,
                axis_limits[axis],
            )
            if fitted is None:
                return None
            corrected_joints_deg[axis] = fitted
        return corrected_joints_deg

    def _normalized_measured_dh_table(self) -> list[list[float]] | None:
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

    def _fk_nominal_pose_base(self, joints_deg: list[float], motion_tool: RobotTool) -> Pose6:
        fk_result = self.robot_model.compute_fk_joints(joints_deg, tool=motion_tool)
        if fk_result is None:
            return Pose6.zeros()
        return fk_result.dh_pose.copy()

    def _fk_measured_pose_base(self, joints_deg: list[float], motion_tool: RobotTool) -> Pose6 | None:
        measured_dh = self._normalized_measured_dh_table()
        if measured_dh is None:
            return None

        transform = np.eye(4, dtype=float)
        axis_reversed = self.robot_model.get_axis_reversed()
        for axis in range(6):
            alpha_deg, d_mm, theta_offset_deg, r_mm = measured_dh[axis]
            theta_rad = math.radians(theta_offset_deg + joints_deg[axis] * float(axis_reversed[axis]))
            transform = transform @ math_utils.dh_modified(
                math.radians(alpha_deg),
                d_mm,
                theta_rad,
                r_mm,
            )
        transform = transform @ RobotModel.build_tool_transform(motion_tool)
        return matrix_to_pose(transform)

    def _target_pose_base(self, motion: RobotProgramMotion, target: RobotProgramTarget, motion_tool: RobotTool) -> Pose6:
        if target.target_type == RobotProgramTargetType.JOINT:
            return self._fk_nominal_pose_base(target.joint_angles.to_list(), motion_tool)
        return self._pose_from_program_base_to_robot_base(target.cartesian_pose, motion.base_pose)

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
    def _fit_joint_near_reference(reference_deg: float, candidate_deg: float, axis_limits_deg: tuple[float, float]) -> float | None:
        joint_min_deg = float(axis_limits_deg[0])
        joint_max_deg = float(axis_limits_deg[1])
        k_min = int(math.ceil((joint_min_deg - candidate_deg) / 360.0))
        k_max = int(math.floor((joint_max_deg - candidate_deg) / 360.0))
        if k_min > k_max:
            return None

        best_value: float | None = None
        best_distance: float | None = None
        for cycle in range(k_min, k_max + 1):
            value = float(candidate_deg) + 360.0 * cycle
            distance = abs(value - float(reference_deg))
            if best_distance is None or distance < best_distance:
                best_value = value
                best_distance = distance
        return best_value

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
        # Binary search for the interval containing clamped_progress
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
