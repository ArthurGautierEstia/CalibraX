from __future__ import annotations

import math

from models.trajectory_keypoint import ConfigurationPolicy, KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.types import JointAngles6, Pose6, TrajectorySampleKinematics, XYZ3
from trajectory_engine.models import (
    BuildStatus,
    JointDynamicStats,
    SegmentResult,
    TrajectoryComputationStatus,
    TrajectoryDynamicViolation,
    TrajectoryDynamicViolationKind,
    TrajectoryDynamicViolationSeverity,
    TrajectoryResult,
    TrajectorySample,
    TrajectorySampleErrorCode,
    TrajectorySampleMgiSolution,
    TrajectorySegment,
    TrajectoryBuilderBehavior,
)
from trajectory_engine.v2.builders.common import BuilderV2Common
from trajectory_engine.v2.dynamics import build_distance_profile, normalized_s_curve
from trajectory_engine.v2.runtime import RuntimeEvaluator
from trajectory_engine.v2.sampling import (
    reset_articular_dynamics,
    reset_cartesian_dynamics,
    update_articular_dynamics,
    update_cartesian_dynamics,
)
from utils.mgi import MgiConfigKey, MgiResult, MgiResultStatus


class TrajectoryBuilderV2(BuilderV2Common):
    def compute_trajectory(self, current_joints: list[float], segments: list[TrajectorySegment]) -> TrajectoryResult:
        result = TrajectoryResult(build_status=BuildStatus.RUNNING)
        self._working_mgi_solver = None
        self._robot_allowed_configs = set(self.robot_model.get_allowed_configurations())
        self._joint_weights = [float(v) for v in self.robot_model.get_joint_weights()[:6]]
        try:
            if self._is_cancelled():
                result.build_status = BuildStatus.CANCELLED
                return result
            if not segments:
                result.build_status = BuildStatus.COMPLETED
                return result

            previous_sample: TrajectorySample | None = None
            start_time_s = 0.0
            first_segment = self.compute_first_segment(current_joints, segments[0].from_keypoint, start_time_s)
            result.segments.append(first_segment)
            self._accumulate_status(result, first_segment, 0)
            if self._should_stop_on_error(first_segment):
                result.build_status = BuildStatus.COMPLETED
                return result
            previous_sample = first_segment.samples[-1] if first_segment.samples else None
            start_time_s = first_segment.last_time

            previous_cart_exit_speed = 0.0
            previous_curve = None
            for index, segment in enumerate(segments):
                if self._is_cancelled():
                    result.build_status = BuildStatus.CANCELLED
                    return result
                if self._is_cartesian_mode(segment.to_keypoint.mode):
                    exit_speed = self._segment_exit_speed(segments, index)
                    segment_result, previous_curve = self._compute_cartesian_segment(
                        segment,
                        index,
                        previous_sample,
                        start_time_s,
                        previous_curve,
                        previous_cart_exit_speed,
                        exit_speed,
                    )
                    previous_cart_exit_speed = exit_speed
                else:
                    segment_result = self.compute_PTP_segment(segment, previous_sample, start_time_s)
                    previous_cart_exit_speed = 0.0
                    previous_curve = None

                result.segments.append(segment_result)
                self._accumulate_status(result, segment_result, index + 1)
                if self._should_stop_on_error(segment_result):
                    break
                previous_sample = segment_result.samples[-1] if segment_result.samples else previous_sample
                start_time_s = segment_result.last_time

            result.build_status = BuildStatus.COMPLETED
            return result
        finally:
            self._working_mgi_solver = None
            self._robot_allowed_configs = None
            self._joint_weights = None

    def compute_first_segment(
        self,
        current_joints: list[float],
        to_keypoint: TrajectoryKeypoint,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        joints = self._copy_joints_6(current_joints)
        config_key = MgiConfigKey.identify_configuration_deg(joints, self.robot_model.get_config_identifier())
        synthetic_from = TrajectoryKeypoint(
            target_type=KeypointTargetType.JOINT,
            joint_target=joints,
            mode=to_keypoint.mode,
            cubic_vectors=[XYZ3.zeros(), XYZ3.zeros()],
            configuration_policy=ConfigurationPolicy.FORCED,
            forced_config=config_key,
            ptp_speed_percent=to_keypoint.ptp_speed_percent,
            linear_speed_mps=to_keypoint.linear_speed_mps,
        )
        return self.compute_segment(TrajectorySegment(synthetic_from, to_keypoint), None, start_time_s)

    def compute_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        if self._is_cartesian_mode(segment.to_keypoint.mode):
            segment_result, _curve = self._compute_cartesian_segment(
                segment,
                0,
                previous_sample,
                start_time_s,
                None,
                0.0,
                0.0,
            )
            return segment_result
        return self.compute_PTP_segment(segment, previous_sample, start_time_s)

    def compute_PTP_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        result = SegmentResult()
        result.mode = KeypointMotionMode.PTP
        previous_joints = JointAngles6.from_values(previous_sample.joints) if previous_sample is not None and previous_sample.reachable else None
        from_joints = self._resolve_keypoint_joints(segment.from_keypoint, previous_joints)
        if from_joints is None:
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            result.last_time = start_time_s
            return result
        to_joints = self._resolve_keypoint_joints(segment.to_keypoint, from_joints)
        if to_joints is None:
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            result.last_time = start_time_s
            return result

        delta = self._shortest_joint_delta(from_joints, to_joints)
        duration_s = self._ptp_duration(segment, delta)
        intervals = self._intervals_for_duration(duration_s)
        if intervals <= 0:
            intervals = 1
        speed_limits, accel_limits, jerk_limits = self._axis_dynamic_limits()
        previous = previous_sample
        for step in range(1, intervals + 1):
            if self._is_cancelled():
                break
            local_time_s = min(duration_s, step * self.sample_dt_s)
            u = 1.0 if duration_s <= self._EPS else local_time_s / duration_s
            smooth_u = normalized_s_curve(u)
            joints = self._interpolate_joints(from_joints, delta, smooth_u)
            sample = self._build_ptp_sample(start_time_s + local_time_s, joints, previous)
            self._apply_dynamic_limits(sample, speed_limits, accel_limits, jerk_limits)
            result.samples.append(sample)
            self._update_joint_stats(result, sample)
            self._register_sample_error(result, sample, len(result.samples) - 1)
            previous = sample
            if self._should_stop_on_error(result):
                break

        result.duration = len(result.samples) * self.sample_dt_s if duration_s <= self._EPS else duration_s
        result.last_time = start_time_s + result.duration
        return result

    def _compute_cartesian_segment(
        self,
        segment: TrajectorySegment,
        segment_index: int,
        previous_sample: TrajectorySample | None,
        start_time_s: float,
        previous_curve: object | None,
        entry_speed_mm_s: float,
        exit_speed_mm_s: float,
    ) -> tuple[SegmentResult, object | None]:
        result = SegmentResult()
        result.mode = segment.to_keypoint.mode
        runtime_segment = self._build_runtime_segment(
            segment,
            segment_index,
            previous_curve,
            entry_speed_mm_s,
            exit_speed_mm_s,
        )
        if runtime_segment is None or runtime_segment.arc_lut is None:
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            result.last_time = start_time_s
            return result, previous_curve

        result.out_direction = runtime_segment.out_direction.to_list()
        result.in_direction = runtime_segment.in_direction.to_list()
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
        previous = previous_sample
        speed_limits, accel_limits, jerk_limits = self._axis_dynamic_limits()
        for step in range(1, intervals + 1):
            if self._is_cancelled():
                break
            time_s = min(profile.duration_s, start_time_s + step * self.sample_dt_s)
            pose = evaluator.evaluate_pose(time_s)
            sample = self._build_cartesian_sample(time_s, pose, previous)
            self._apply_dynamic_limits(sample, speed_limits, accel_limits, jerk_limits)
            result.samples.append(sample)
            self._update_joint_stats(result, sample)
            self._register_sample_error(result, sample, len(result.samples) - 1)
            previous = sample
            if self._should_stop_on_error(result):
                break

        result.duration = max(0.0, profile.duration_s - start_time_s)
        result.last_time = start_time_s + result.duration
        return result, runtime_segment.curve

    def _build_cartesian_sample(
        self,
        time_s: float,
        pose: Pose6,
        previous_sample: TrajectorySample | None,
    ) -> TrajectorySample:
        sample = TrajectorySample()
        sample.time = float(time_s)
        sample.pose = pose.to_list()
        previous_joints = JointAngles6.from_values(previous_sample.joints) if previous_sample is not None and previous_sample.reachable else None
        allowed_configs = set(self._get_robot_allowed_configs())
        mgi_result = self._compute_mgi_for_pose(pose, previous_joints)
        sample.mgi_solutions = self._compact_mgi_solutions(mgi_result, allowed_configs)
        selected = self._select_best_solution(mgi_result, previous_joints, allowed_configs)
        if selected is None:
            sample.reachable = False
            sample.error_code = TrajectorySampleErrorCode.POINT_UNREACHABLE
            sample.joints = [0.0] * 6
            sample.configuration = None
        else:
            config_key, solution = selected
            sample.joints = self._copy_joints_6(solution.joints)
            sample.configuration = config_key
            fk_result = self.robot_model.compute_fk_joints(sample.joints, tool=self.tool_model.get_tool())
            if fk_result is None:
                sample.reachable = False
                sample.error_code = TrajectorySampleErrorCode.POINT_UNREACHABLE
            else:
                sample.reachable = True
                sample.error_code = TrajectorySampleErrorCode.NONE
                sample.kinematics = TrajectorySampleKinematics.from_fk_result(fk_result)
        self._update_sample_dynamics(sample, previous_sample)
        return sample

    def _build_ptp_sample(
        self,
        time_s: float,
        joints_deg: JointAngles6,
        previous_sample: TrajectorySample | None,
    ) -> TrajectorySample:
        sample = TrajectorySample()
        sample.time = float(time_s)
        sample.joints = joints_deg.to_list()
        fk_result = self.robot_model.compute_fk_joints(sample.joints, tool=self.tool_model.get_tool())
        if fk_result is None:
            sample.reachable = False
            sample.error_code = TrajectorySampleErrorCode.POINT_UNREACHABLE
            sample.pose = [0.0] * 6
            sample.configuration = None
        else:
            sample.reachable = True
            sample.error_code = TrajectorySampleErrorCode.NONE
            sample.pose = fk_result.dh_pose.to_list()
            sample.kinematics = TrajectorySampleKinematics.from_fk_result(fk_result)
            sample.configuration = MgiConfigKey.identify_configuration_deg(sample.joints, self.robot_model.get_config_identifier())
            sample.mgi_solutions = {
                sample.configuration: TrajectorySampleMgiSolution(status=MgiResultStatus.VALID.name, joints=sample.joints)
            }
        self._update_sample_dynamics(sample, previous_sample)
        return sample

    def _update_sample_dynamics(self, sample: TrajectorySample, previous_sample: TrajectorySample | None) -> None:
        if previous_sample is None:
            reset_cartesian_dynamics(sample)
            reset_articular_dynamics(sample)
            return
        dt = sample.time - previous_sample.time
        if dt <= self._EPS:
            dt = self.sample_dt_s
        update_cartesian_dynamics(sample, previous_sample, dt)
        update_articular_dynamics(sample, previous_sample, dt)

    @staticmethod
    def _compact_mgi_solutions(
        mgi_result: MgiResult,
        allowed_configs: set[MgiConfigKey],
    ) -> dict[MgiConfigKey, TrajectorySampleMgiSolution]:
        compact: dict[MgiConfigKey, TrajectorySampleMgiSolution] = {}
        for config_key, solution in mgi_result.solutions.items():
            expanded = mgi_result.get_solutions_expanded(config_key, only_valid=True)
            selected = expanded[0] if expanded else solution
            status_name = MgiResultStatus.VALID.name if expanded else solution.status.name
            if status_name == MgiResultStatus.VALID.name and config_key not in allowed_configs:
                status_name = MgiResultStatus.FORBIDDEN_CONFIGURATION.name
            compact[config_key] = TrajectorySampleMgiSolution(status=status_name, joints=selected.joints)
        return compact

    def _axis_dynamic_limits(self) -> tuple[list[float], list[float], list[float]]:
        speed_limits = [max(0.0, float(v)) for v in self.robot_model.get_axis_speed_limits()[:6]]
        accel_limits = [max(0.0, float(v)) for v in self.robot_model.get_axis_accel_limits()[:6]]
        jerk_limits = [max(0.0, float(v)) for v in self.robot_model.get_axis_jerk_limits()[:6]]
        while len(speed_limits) < 6:
            speed_limits.append(0.0)
        while len(accel_limits) < 6:
            accel_limits.append(0.0)
        while len(jerk_limits) < 6:
            jerk_limits.append(0.0)
        return speed_limits, accel_limits, jerk_limits

    def _apply_dynamic_limits(
        self,
        sample: TrajectorySample,
        speed_limits: list[float],
        accel_limits: list[float],
        jerk_limits: list[float],
    ) -> None:
        if sample.error_code != TrajectorySampleErrorCode.NONE:
            return
        eps = 1e-6
        for axis in range(6):
            speed = abs(float(sample.articular_velocity[axis]))
            if sample.articular_velocity_valid and speed > speed_limits[axis] + eps:
                sample.dynamic_violations.append(
                    TrajectoryDynamicViolation(
                        TrajectoryDynamicViolationKind.SPEED,
                        axis,
                        speed,
                        speed_limits[axis],
                        TrajectoryDynamicViolationSeverity.ERROR,
                    )
                )
            accel = abs(float(sample.articular_acceleration[axis]))
            if sample.articular_acceleration_valid and accel_limits[axis] > self._EPS and accel > accel_limits[axis] + eps:
                sample.dynamic_violations.append(
                    TrajectoryDynamicViolation(
                        TrajectoryDynamicViolationKind.ACCELERATION,
                        axis,
                        accel,
                        accel_limits[axis],
                        TrajectoryDynamicViolationSeverity.WARNING,
                    )
                )
            jerk = abs(float(sample.articular_jerk[axis]))
            if (
                self.jerk_check_enabled
                and sample.articular_jerk_valid
                and jerk_limits[axis] > self._EPS
                and jerk > jerk_limits[axis] + eps
            ):
                sample.dynamic_violations.append(
                    TrajectoryDynamicViolation(
                        TrajectoryDynamicViolationKind.JERK,
                        axis,
                        jerk,
                        jerk_limits[axis],
                        TrajectoryDynamicViolationSeverity.ERROR,
                    )
                )
        self._set_primary_dynamic_error(sample)

    @staticmethod
    def _set_primary_dynamic_error(sample: TrajectorySample) -> None:
        if sample.error_code != TrajectorySampleErrorCode.NONE:
            return
        speed_errors = [
            violation
            for violation in sample.dynamic_violations
            if violation.severity == TrajectoryDynamicViolationSeverity.ERROR
            and violation.kind == TrajectoryDynamicViolationKind.SPEED
        ]
        if speed_errors:
            first = min(speed_errors, key=lambda violation: violation.axis)
            sample.error_code = TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED
            sample.error_axis = first.axis
            return
        jerk_errors = [
            violation
            for violation in sample.dynamic_violations
            if violation.severity == TrajectoryDynamicViolationSeverity.ERROR
            and violation.kind == TrajectoryDynamicViolationKind.JERK
        ]
        if jerk_errors:
            first = min(jerk_errors, key=lambda violation: violation.axis)
            sample.error_code = TrajectorySampleErrorCode.JERK_LIMIT_EXCEEDED
            sample.error_axis = first.axis

    @staticmethod
    def _status_from_sample_error(error_code: TrajectorySampleErrorCode) -> TrajectoryComputationStatus:
        if error_code == TrajectorySampleErrorCode.POINT_UNREACHABLE:
            return TrajectoryComputationStatus.POINT_UNREACHABLE
        if error_code == TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED:
            return TrajectoryComputationStatus.SPEED_LIMIT_EXCEEDED
        if error_code == TrajectorySampleErrorCode.JERK_LIMIT_EXCEEDED:
            return TrajectoryComputationStatus.JERK_LIMIT_EXCEEDED
        if error_code == TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION:
            return TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION
        return TrajectoryComputationStatus.SUCCESS

    def _register_sample_error(self, result: SegmentResult, sample: TrajectorySample, sample_index: int) -> None:
        if sample.error_code == TrajectorySampleErrorCode.NONE:
            return
        if result.first_error_sample_index is None:
            result.first_error_sample_index = int(sample_index)
            result.first_error_axis = sample.error_axis
        if result.status == TrajectoryComputationStatus.SUCCESS:
            result.status = self._status_from_sample_error(sample.error_code)

    @staticmethod
    def _accumulate_status(trajectory: TrajectoryResult, segment: SegmentResult, segment_index: int) -> None:
        if segment.status == TrajectoryComputationStatus.SUCCESS:
            return
        if trajectory.status != TrajectoryComputationStatus.SUCCESS:
            return
        trajectory.status = segment.status
        trajectory.first_error_segment_index = int(segment_index)

    def _should_stop_on_error(self, segment: SegmentResult) -> bool:
        if segment.status == TrajectoryComputationStatus.SUCCESS:
            return False
        return self.behavior == TrajectoryBuilderBehavior.STOP_ON_ERROR

    @staticmethod
    def _update_joint_stats(segment_result: SegmentResult, sample: TrajectorySample) -> None:
        for axis in range(6):
            stats: JointDynamicStats = segment_result.joints_stats[axis]
            velocity = float(sample.articular_velocity[axis])
            acceleration = float(sample.articular_acceleration[axis])
            if velocity > stats.max_positive_velocity:
                stats.max_positive_velocity = velocity
            if velocity < stats.max_negative_velocity:
                stats.max_negative_velocity = velocity
            if acceleration > stats.max_acceleration:
                stats.max_acceleration = acceleration
            if acceleration < stats.max_deceleration:
                stats.max_deceleration = acceleration

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
