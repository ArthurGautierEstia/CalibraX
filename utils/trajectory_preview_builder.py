from __future__ import annotations

import math

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.trajectory_keypoint import KeypointMotionMode, TrajectoryKeypoint
from models.trajectory_options import TrajectoryBezierDegree
from models.trajectory_preview import TrajectoryPreviewResult, TrajectoryPreviewSample, TrajectoryPreviewSegment
from models.trajectory_result import (
    TrajectoryBuilderBehavior,
    TrajectoryComputationStatus,
    TrajectorySample,
    TrajectorySegment,
)
from models.types import JointAngles6, Pose6, XYZ3
from models.workspace_model import WorkspaceModel
from utils.trajectory_builder import TrajectoryBuilder
import utils.math_utils as math_utils


class TrajectoryPreviewBuilder(TrajectoryBuilder):
    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        behavior: TrajectoryBuilderBehavior = TrajectoryBuilderBehavior.CONTINUE_ON_ERROR,
        sample_dt_s: float = TrajectoryBuilder.DEFAULT_SAMPLE_DT_S,
        smooth_time_enabled: bool = True,
        bezier_degree: TrajectoryBezierDegree | str = TrajectoryBezierDegree.BEZIER5,
        jerk_check_enabled: bool = True,
    ) -> None:
        super().__init__(
            robot_model=robot_model,
            tool_model=tool_model,
            workspace_model=workspace_model,
            behavior=behavior,
            sample_dt_s=sample_dt_s,
            smooth_time_enabled=smooth_time_enabled,
            bezier_degree=bezier_degree,
            jerk_check_enabled=jerk_check_enabled,
        )

    @staticmethod
    def _new_empty_preview_segment(last_time_s: float, mode: KeypointMotionMode) -> TrajectoryPreviewSegment:
        segment = TrajectoryPreviewSegment()
        segment.mode = mode
        segment.last_time_s = float(last_time_s)
        return segment

    @staticmethod
    def _extract_previous_sample(result: TrajectoryPreviewSegment) -> TrajectoryPreviewSample | None:
        if not result.samples:
            return None
        return result.samples[-1]

    @staticmethod
    def _preview_sample_to_builder_sample(sample: TrajectoryPreviewSample | None) -> TrajectorySample | None:
        if sample is None or sample.joints_deg is None:
            return None
        builder_sample = TrajectorySample()
        builder_sample.time = sample.time_s
        builder_sample.pose = sample.pose_base.to_list()
        builder_sample.joints = sample.joints_deg.to_list()
        builder_sample.reachable = bool(sample.reachable)
        return builder_sample

    @staticmethod
    def _accumulate_status(
        trajectory: TrajectoryPreviewResult,
        segment_result: TrajectoryPreviewSegment,
        segment_index: int,
    ) -> None:
        if segment_result.status == TrajectoryComputationStatus.SUCCESS:
            return
        if trajectory.status != TrajectoryComputationStatus.SUCCESS:
            return
        trajectory.status = segment_result.status
        trajectory.first_error_segment_index = int(segment_index)

    def compute_preview(
        self,
        current_joints: JointAngles6,
        segments: list[TrajectorySegment],
    ) -> TrajectoryPreviewResult:
        result = TrajectoryPreviewResult()
        if not segments:
            return result

        self._working_mgi_solver = None
        self._robot_allowed_configs = set(self.robot_model.get_allowed_configurations())
        self._joint_weights = [float(v) for v in self.robot_model.get_joint_weights()[:6]]
        try:
            start_time_s = 0.0
            first_result = self.compute_first_preview_segment(current_joints, segments[0].from_keypoint, start_time_s)
            result.segments.append(first_result)
            TrajectoryPreviewBuilder._accumulate_status(result, first_result, 0)
            if self._should_stop_on_error(first_result.status):
                return result

            previous_sample = TrajectoryPreviewBuilder._extract_previous_sample(first_result)
            start_time_s = first_result.last_time_s
            segment_idx = 0
            while segment_idx < len(segments):
                segment = segments[segment_idx]
                if TrajectoryBuilder._is_lin_or_cubic_mode(segment.to_keypoint.mode):
                    descriptors = self._collect_bezier_chain_descriptors(segments, segment_idx)
                    if len(descriptors) >= 2:
                        chain_results = self._generate_preview_bezier_super_segments(descriptors, start_time_s)
                        if chain_results:
                            for local_offset, chain_result in enumerate(chain_results):
                                result.segments.append(chain_result)
                                TrajectoryPreviewBuilder._accumulate_status(
                                    result,
                                    chain_result,
                                    1 + segment_idx + local_offset,
                                )
                                if self._should_stop_on_error(chain_result.status):
                                    return result
                            previous_sample = TrajectoryPreviewBuilder._extract_previous_sample(chain_results[-1])
                            start_time_s = chain_results[-1].last_time_s
                            segment_idx += len(chain_results)
                            continue

                segment_result = self.compute_preview_segment(segment, previous_sample, start_time_s)
                result.segments.append(segment_result)
                TrajectoryPreviewBuilder._accumulate_status(result, segment_result, 1 + segment_idx)
                if self._should_stop_on_error(segment_result.status):
                    break
                previous_sample = TrajectoryPreviewBuilder._extract_previous_sample(segment_result)
                start_time_s = segment_result.last_time_s
                segment_idx += 1
            result.analysis_pending = result.sample_count() > 0
            return result
        finally:
            self._working_mgi_solver = None
            self._robot_allowed_configs = None
            self._joint_weights = None

    def compute_first_preview_segment(
        self,
        current_joints: JointAngles6,
        to_keypoint: TrajectoryKeypoint,
        start_time_s: float = 0.0,
    ) -> TrajectoryPreviewSegment:
        from models.trajectory_keypoint import ConfigurationPolicy, KeypointTargetType

        joints_6 = current_joints.to_list()
        identified_config = self.robot_model.get_current_axis_config()
        if identified_config is None:
            identified_config = self._resolve_reference_config(None)

        synthetic_from = TrajectoryKeypoint(
            target_type=KeypointTargetType.JOINT,
            joint_target=joints_6,
            mode=to_keypoint.mode,
            cubic_vectors=[XYZ3.zeros(), XYZ3.zeros()],
            configuration_policy=ConfigurationPolicy.FORCED,
            forced_config=identified_config,
            ptp_speed_percent=to_keypoint.ptp_speed_percent,
            linear_speed_mps=to_keypoint.linear_speed_mps,
        )
        segment = TrajectorySegment(synthetic_from, to_keypoint)
        return self.compute_preview_segment(segment, None, start_time_s)

    def compute_preview_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectoryPreviewSample | None = None,
        start_time_s: float = 0.0,
    ) -> TrajectoryPreviewSegment:
        if segment.to_keypoint.mode == KeypointMotionMode.LINEAR:
            return self._generate_preview_bezier_segment(segment, start_time_s, True)
        if segment.to_keypoint.mode == KeypointMotionMode.CUBIC:
            return self._generate_preview_bezier_segment(segment, start_time_s, False)
        return self.compute_preview_ptp_segment(segment, previous_sample, start_time_s)

    def compute_preview_ptp_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectoryPreviewSample | None = None,
        start_time_s: float = 0.0,
    ) -> TrajectoryPreviewSegment:
        result = TrajectoryPreviewSegment()
        result.mode = segment.to_keypoint.mode

        previous_builder_sample = TrajectoryPreviewBuilder._preview_sample_to_builder_sample(previous_sample)
        from_allowed_configs = self._resolve_allowed_configs_for_keypoint(segment.from_keypoint, previous_builder_sample)
        if not from_allowed_configs:
            result.status = TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION
            result.last_time_s = float(start_time_s)
            return result

        config_identifier = self.robot_model.get_config_identifier()
        from_joints, to_joints, endpoint_error = self._resolve_PTP_segment_endpoints(
            segment,
            previous_builder_sample,
            from_allowed_configs,
            config_identifier,
        )
        if from_joints is None or to_joints is None:
            result.status = TrajectoryBuilder._status_from_sample_error(endpoint_error)
            result.last_time_s = float(start_time_s)
            return result

        joint_deltas_deg = self._compute_PTP_shortest_path(from_joints, to_joints)
        if joint_deltas_deg is None:
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            result.last_time_s = float(start_time_s)
            return result

        required_duration_s, _effective_speed_limits = self._resolve_PTP_duration(segment, joint_deltas_deg)
        if required_duration_s is None:
            result.status = TrajectoryComputationStatus.SPEED_LIMIT_EXCEEDED
            result.last_time_s = float(start_time_s)
            return result

        intervals = 1
        if required_duration_s > self._EPS:
            intervals = int(math.ceil(required_duration_s / self.sample_dt_s))
        intervals = min(max(1, intervals), TrajectoryBuilder.MAX_SAMPLES_PER_SEGMENT)

        for i in range(1, intervals + 1):
            time_s = start_time_s + i * self.sample_dt_s
            linear_t = i / intervals
            smooth_t = math_utils.quintic_transition(linear_t)
            joints = JointAngles6.from_values(
                [from_joints[axis] + joint_deltas_deg[axis] * smooth_t for axis in range(6)]
            )
            fk_result = self.robot_model.compute_fk_joints(joints.to_list(), tool=self.tool_model.get_tool())
            if fk_result is None:
                result.samples.append(
                    TrajectoryPreviewSample(time_s=time_s, pose_base=Pose6.zeros(), joints_deg=joints, reachable=False)
                )
                if result.status == TrajectoryComputationStatus.SUCCESS:
                    result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
                continue
            result.samples.append(
                TrajectoryPreviewSample(
                    time_s=time_s,
                    pose_base=fk_result.dh_pose,
                    joints_deg=joints,
                    reachable=True,
                )
            )

        result.duration_s = len(result.samples) * self.sample_dt_s
        result.last_time_s = start_time_s + result.duration_s
        return result

    def _generate_preview_bezier_segment(
        self,
        segment: TrajectorySegment,
        start_time_s: float,
        force_linear_handles: bool,
    ) -> TrajectoryPreviewSegment:
        from_pose = self._resolve_keypoint_pose(segment.from_keypoint)
        to_pose = self._resolve_keypoint_pose(segment.to_keypoint)
        if from_pose is None or to_pose is None:
            result = TrajectoryPreviewBuilder._new_empty_preview_segment(start_time_s, segment.to_keypoint.mode)
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            return result

        p0_xyz = XYZ3(float(from_pose[0]), float(from_pose[1]), float(from_pose[2]))
        p3_xyz = XYZ3(float(to_pose[0]), float(to_pose[1]), float(to_pose[2]))
        p0 = p0_xyz.to_list()
        p3 = p3_xyz.to_list()
        segment_length_mm = math_utils.norm3(p3_xyz.x - p0_xyz.x, p3_xyz.y - p0_xyz.y, p3_xyz.z - p0_xyz.z)
        if force_linear_handles:
            t_out_xyz, t_in_xyz = segment.to_keypoint.resolve_linear_tangent_vectors(p0_xyz, p3_xyz)
        else:
            t_out_xyz, t_in_xyz = segment.to_keypoint.resolve_cubic_tangent_vectors(segment_length_mm)
        t_out = t_out_xyz.to_list()
        t_in = t_in_xyz.to_list()

        coeffs = self._build_bezier_coefficients(p0, p3, t_out, t_in)
        arc_length_mm = TrajectoryBuilder._estimate_arc_length(coeffs)
        speed_mmps = TrajectoryBuilder.linear_speed_mps_to_mmps(segment.to_keypoint.linear_speed_mps)
        intervals = TrajectoryBuilder._resolve_num_intervals(
            arc_length_mm,
            speed_mmps,
            self.sample_dt_s,
            self._time_profile_peak_speed_scale(),
        )

        result = TrajectoryPreviewSegment()
        result.mode = segment.to_keypoint.mode
        result.out_direction = XYZ3.from_values(t_out)
        result.in_direction = XYZ3.from_values(t_in)
        dA = TrajectoryBuilder._shortest_angle_delta_deg(from_pose[3], to_pose[3])
        dB = TrajectoryBuilder._shortest_angle_delta_deg(from_pose[4], to_pose[4])
        dC = TrajectoryBuilder._shortest_angle_delta_deg(from_pose[5], to_pose[5])
        for i in range(1, intervals + 1):
            time_s = start_time_s + i * self.sample_dt_s
            linear_t = i / intervals
            smooth_t = math_utils.quintic_transition(linear_t) if self.smooth_time_enabled else linear_t
            smooth_t5 = math_utils.quintic_transition(linear_t)
            xyz = coeffs.point(smooth_t)
            pose = Pose6(
                xyz[0],
                xyz[1],
                xyz[2],
                TrajectoryBuilder._wrap_angle_deg(from_pose[3] + dA * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(from_pose[4] + dB * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(from_pose[5] + dC * smooth_t5),
            )
            result.samples.append(TrajectoryPreviewSample(time_s=time_s, pose_base=pose, joints_deg=None))

        result.duration_s = len(result.samples) * self.sample_dt_s
        result.last_time_s = start_time_s + result.duration_s
        return result

    def _generate_preview_bezier_super_segments(
        self,
        descriptors,
        start_time_s: float,
    ) -> list[TrajectoryPreviewSegment]:
        if len(descriptors) < 2:
            return []
        total_length_mm = sum(max(0.0, float(descriptor.arc_length_mm)) for descriptor in descriptors)
        if total_length_mm <= self._EPS:
            return []
        group_speed_mmps = TrajectoryBuilder._harmonic_weighted_speed_mmps(descriptors)
        (
            intervals,
            cumulative_lengths,
            segment_by_step,
            distance_by_step,
            _first_step_by_segment,
            _last_step_by_segment,
            _counts_by_segment,
        ) = self._resolve_super_chain_intervals_and_assignment(descriptors, total_length_mm, group_speed_mmps)

        results: list[TrajectoryPreviewSegment] = []
        for descriptor in descriptors:
            segment_result = TrajectoryPreviewSegment()
            segment_result.mode = descriptor.segment.to_keypoint.mode
            segment_result.out_direction = XYZ3.from_values(descriptor.t_out)
            segment_result.in_direction = XYZ3.from_values(descriptor.t_in)
            segment_result.last_time_s = float(start_time_s)
            results.append(segment_result)

        orientation_deltas = [
            [
                TrajectoryBuilder._shortest_angle_delta_deg(descriptor.from_pose[3], descriptor.to_pose[3]),
                TrajectoryBuilder._shortest_angle_delta_deg(descriptor.from_pose[4], descriptor.to_pose[4]),
                TrajectoryBuilder._shortest_angle_delta_deg(descriptor.from_pose[5], descriptor.to_pose[5]),
            ]
            for descriptor in descriptors
        ]

        for step in range(1, intervals + 1):
            segment_idx = int(segment_by_step[step - 1])
            descriptor = descriptors[segment_idx]
            segment_result = results[segment_idx]
            time_s = start_time_s + step * self.sample_dt_s
            local_distance_mm = distance_by_step[step - 1] - cumulative_lengths[segment_idx]
            local_distance_mm = max(0.0, min(float(descriptor.arc_length_mm), float(local_distance_mm)))
            local_t = TrajectoryBuilder._distance_to_bezier_parameter(
                descriptor.arc_lut_t,
                descriptor.arc_lut_s,
                local_distance_mm,
            )
            xyz = descriptor.coeffs.point(local_t)
            smooth_t5 = math_utils.quintic_transition(local_t)
            dA, dB, dC = orientation_deltas[segment_idx]
            pose = Pose6(
                xyz[0],
                xyz[1],
                xyz[2],
                TrajectoryBuilder._wrap_angle_deg(descriptor.from_pose[3] + dA * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(descriptor.from_pose[4] + dB * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(descriptor.from_pose[5] + dC * smooth_t5),
            )
            segment_result.samples.append(TrajectoryPreviewSample(time_s=time_s, pose_base=pose, joints_deg=None))

        rolling_time = float(start_time_s)
        for segment_result in results:
            segment_result.duration_s = len(segment_result.samples) * self.sample_dt_s
            if segment_result.samples:
                segment_result.last_time_s = float(segment_result.samples[-1].time_s)
                rolling_time = segment_result.last_time_s
            else:
                segment_result.last_time_s = rolling_time
        return results
