from __future__ import annotations

import math

from models.trajectory_keypoint import ConfigurationPolicy, KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.types import JointAngles6, Pose6, XYZ3
from trajectory_engine.models.pipeline import (
    BuildStatus,
    TrajectoryComputationStatus,
    TrajectoryPreviewResult,
    TrajectoryPreviewSample,
    TrajectoryPreviewSegment,
    TrajectorySample,
    TrajectorySegment,
)
from trajectory_engine.core.builder_common import TrajectoryBuilderCommon
from trajectory_engine.dynamics import build_distance_profile, normalized_s_curve
from trajectory_engine.runtime import RuntimeEvaluator
from utils.mgi import MgiConfigKey


class TrajectoryPreviewBuilder(TrajectoryBuilderCommon):
    def compute_preview(self, current_joints: JointAngles6, segments: list[TrajectorySegment]) -> TrajectoryPreviewResult:
        result = TrajectoryPreviewResult(build_status=BuildStatus.RUNNING)
        self._working_mgi_solver = None
        self._robot_allowed_configs = set(self.robot_model.get_allowed_configurations())
        self._joint_weights = [float(v) for v in self.robot_model.get_joint_weights()[:6]]
        try:
            if not segments:
                result.build_status = BuildStatus.COMPLETED
                return result
            start_time_s = 0.0
            first = self.compute_first_preview_segment(current_joints, segments[0].from_keypoint, start_time_s)
            result.segments.append(first)
            self._accumulate_status(result, first, 0)
            previous_preview = first.samples[-1] if first.samples else None
            start_time_s = first.last_time_s
            previous_cart_exit_speed = 0.0
            for index, segment in enumerate(segments):
                if self._is_cancelled():
                    result.build_status = BuildStatus.CANCELLED
                    return result
                if self._is_cartesian_mode(segment.to_keypoint.mode):
                    exit_speed = self._segment_exit_speed(segments, index)
                    segment_result = self._compute_preview_cartesian_segment(
                        segment,
                        index,
                        start_time_s,
                        previous_cart_exit_speed,
                        exit_speed,
                    )
                    previous_cart_exit_speed = exit_speed
                else:
                    segment_result = self.compute_preview_ptp_segment(segment, previous_preview, start_time_s)
                    previous_cart_exit_speed = 0.0
                result.segments.append(segment_result)
                self._accumulate_status(result, segment_result, index + 1)
                previous_preview = segment_result.samples[-1] if segment_result.samples else previous_preview
                start_time_s = segment_result.last_time_s
                if segment_result.status != TrajectoryComputationStatus.SUCCESS:
                    break
            result.analysis_pending = result.sample_count() > 0
            result.build_status = BuildStatus.COMPLETED
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
        config_key = MgiConfigKey.identify_configuration_deg(current_joints.to_list(), self.robot_model.get_config_identifier())
        synthetic_from = TrajectoryKeypoint(
            target_type=KeypointTargetType.JOINT,
            joint_target=current_joints.to_list(),
            mode=to_keypoint.mode,
            cubic_vectors=[XYZ3.zeros(), XYZ3.zeros()],
            configuration_policy=ConfigurationPolicy.FORCED,
            forced_config=config_key,
            ptp_speed_percent=to_keypoint.ptp_speed_percent,
            linear_speed_mps=to_keypoint.linear_speed_mps,
        )
        return self.compute_preview_segment(TrajectorySegment(synthetic_from, to_keypoint), None, start_time_s)

    def compute_preview_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectoryPreviewSample | None = None,
        start_time_s: float = 0.0,
    ) -> TrajectoryPreviewSegment:
        if self._is_cartesian_mode(segment.to_keypoint.mode):
            return self._compute_preview_cartesian_segment(segment, 0, start_time_s, 0.0, 0.0)
        return self.compute_preview_ptp_segment(segment, previous_sample, start_time_s)

    def compute_preview_ptp_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectoryPreviewSample | None = None,
        start_time_s: float = 0.0,
    ) -> TrajectoryPreviewSegment:
        result = TrajectoryPreviewSegment()
        result.mode = KeypointMotionMode.PTP
        previous_joints = previous_sample.joints_deg if previous_sample is not None and previous_sample.joints_deg is not None else None
        from_joints = self._resolve_keypoint_joints(segment.from_keypoint, previous_joints)
        to_joints = self._resolve_keypoint_joints(segment.to_keypoint, from_joints)
        if from_joints is None or to_joints is None:
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            result.last_time_s = start_time_s
            return result
        delta = self._shortest_joint_delta(from_joints, to_joints)
        duration_s = self._ptp_duration(segment, delta)
        if duration_s <= self._EPS:
            result.duration_s = 0.0
            result.last_time_s = start_time_s
            return result
        intervals = self._intervals_for_duration(duration_s)
        for step in range(1, intervals + 1):
            if self._is_cancelled():
                break
            local_time_s = min(duration_s, step * self.sample_dt_s)
            ratio = 1.0 if duration_s <= self._EPS else normalized_s_curve(local_time_s / duration_s)
            joints = self._interpolate_joints(from_joints, delta, ratio)
            fk_result = self.robot_model.compute_fk_joints(joints.to_list(), tool=self.tool_model.get_tool())
            if fk_result is None:
                result.samples.append(TrajectoryPreviewSample(time_s=start_time_s + local_time_s, pose_base=Pose6.zeros(), joints_deg=joints, reachable=False))
                result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            else:
                result.samples.append(TrajectoryPreviewSample(time_s=start_time_s + local_time_s, pose_base=fk_result.dh_pose, joints_deg=joints, reachable=True))
        result.duration_s = duration_s
        result.last_time_s = start_time_s + duration_s
        return result

    def _compute_preview_cartesian_segment(
        self,
        segment: TrajectorySegment,
        segment_index: int,
        start_time_s: float,
        entry_speed_mm_s: float,
        exit_speed_mm_s: float,
    ) -> TrajectoryPreviewSegment:
        result = TrajectoryPreviewSegment()
        result.mode = segment.to_keypoint.mode
        runtime_segment = self._build_runtime_segment(segment, segment_index, entry_speed_mm_s, exit_speed_mm_s)
        if runtime_segment is None:
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            result.last_time_s = start_time_s
            return result
        result.out_direction = runtime_segment.out_direction.copy()
        result.in_direction = runtime_segment.in_direction.copy()
        limits = self._dynamic_limits(segment)
        profile = build_distance_profile(
            segment_index=segment_index,
            length_mm=runtime_segment.speed_profile.length_mm,
            target_speed_mm_s=runtime_segment.speed_profile.target_speed_mm_s,
            entry_speed_mm_s=runtime_segment.speed_profile.entry_speed_mm_s,
            exit_speed_mm_s=runtime_segment.speed_profile.exit_speed_mm_s,
            accel_limit_mm_s2=limits.cartesian_accel_mm_s2,
            jerk_limit_mm_s3=limits.cartesian_jerk_mm_s3,
            start_time_s=start_time_s,
            start_position_mm=0.0,
        )
        evaluator = RuntimeEvaluator(runtime_segment, profile)
        intervals = self._intervals_for_duration(profile.duration_s - start_time_s)
        for step in range(1, intervals + 1):
            if self._is_cancelled():
                break
            time_s = min(profile.duration_s, start_time_s + step * self.sample_dt_s)
            result.samples.append(TrajectoryPreviewSample(time_s=time_s, pose_base=evaluator.evaluate_pose(time_s), joints_deg=None, reachable=True))
        result.duration_s = max(0.0, profile.duration_s - start_time_s)
        result.last_time_s = start_time_s + result.duration_s
        return result

    @staticmethod
    def _accumulate_status(
        trajectory: TrajectoryPreviewResult,
        segment: TrajectoryPreviewSegment,
        segment_index: int,
    ) -> None:
        if segment.status == TrajectoryComputationStatus.SUCCESS:
            return
        if trajectory.status != TrajectoryComputationStatus.SUCCESS:
            return
        trajectory.status = segment.status
        trajectory.first_error_segment_index = int(segment_index)

    def _intervals_for_duration(self, duration_s: float) -> int:
        if duration_s <= self._EPS:
            return 1
        return min(max(1, int(math.ceil(float(duration_s) / self.sample_dt_s))), self.MAX_SAMPLES_PER_SEGMENT)

    @staticmethod
    def _shortest_joint_delta(from_joints: JointAngles6, to_joints: JointAngles6) -> JointAngles6:
        from_values = from_joints.to_list()
        to_values = to_joints.to_list()
        deltas: list[float] = []
        for axis in range(6):
            delta = (to_values[axis] - from_values[axis] + 180.0) % 360.0 - 180.0
            if delta == -180.0 and (to_values[axis] - from_values[axis]) > 0.0:
                delta = 180.0
            deltas.append(delta)
        return JointAngles6.from_values(deltas)

    @staticmethod
    def _interpolate_joints(start: JointAngles6, delta: JointAngles6, ratio: float) -> JointAngles6:
        start_values = start.to_list()
        delta_values = delta.to_list()
        return JointAngles6.from_values([start_values[axis] + delta_values[axis] * ratio for axis in range(6)])
