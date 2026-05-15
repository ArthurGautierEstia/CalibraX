from __future__ import annotations

from dataclasses import dataclass
import math

from models.trajectory_keypoint import ConfigurationPolicy, KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.types import JointAngles6, Pose6, TrajectorySampleKinematics, XYZ3
from trajectory_engine.models.pipeline import (
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
from trajectory_engine.core.builder_common import TrajectoryBuilderCommon
from trajectory_engine.dynamics import (
    build_distance_profile,
    normalized_s_curve,
    normalized_s_curve_derivative,
    normalized_s_curve_second_derivative,
    normalized_s_curve_third_derivative,
)
from trajectory_engine.runtime import RuntimeEvaluator
from trajectory_engine.sampling import (
    reset_articular_dynamics,
    reset_cartesian_dynamics,
    update_articular_dynamics,
    update_cartesian_dynamics,
)
from utils.mgi import MgiConfigKey, MgiResult, MgiResultStatus


@dataclass(frozen=True)
class _SampleSchedulePoint:
    time_s: float
    tick_index: int


@dataclass(frozen=True)
class _QuantizedDuration:
    tick_count: int
    duration_s: float


class _SampleClock:
    _EPS = 1e-9

    def __init__(self, sample_dt_s: float, start_time_s: float = 0.0) -> None:
        self.sample_dt_s = max(float(sample_dt_s), self._EPS)
        self.origin_time_s = float(start_time_s)
        self.next_tick_index = 1

    def quantize_duration(self, duration_s: float) -> _QuantizedDuration:
        duration = max(0.0, float(duration_s))
        if duration <= self._EPS:
            return _QuantizedDuration(tick_count=0, duration_s=0.0)
        tick_count = int(math.ceil((duration - self._EPS) / self.sample_dt_s))
        tick_count = max(1, tick_count)
        return _QuantizedDuration(tick_count=tick_count, duration_s=tick_count * self.sample_dt_s)

    def segment_points(self, start_time_s: float, tick_count: int) -> list[_SampleSchedulePoint]:
        if tick_count <= 0:
            return []

        start_tick_index = self._tick_index_for_time(start_time_s)
        first_tick_index = max(self.next_tick_index, start_tick_index + 1)
        end_tick_index = start_tick_index + int(tick_count)
        points = [
            _SampleSchedulePoint(
                time_s=self.time_for_tick(tick_index),
                tick_index=tick_index,
            )
            for tick_index in range(first_tick_index, end_tick_index + 1)
        ]
        self.next_tick_index = end_tick_index + 1
        return points

    def time_for_tick(self, tick_index: int) -> float:
        return self.origin_time_s + int(tick_index) * self.sample_dt_s

    def _tick_index_for_time(self, time_s: float) -> int:
        return int(round((float(time_s) - self.origin_time_s) / self.sample_dt_s))


class TrajectoryBuilder(TrajectoryBuilderCommon):
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
            sample_clock = _SampleClock(self.sample_dt_s, start_time_s)
            first_segment = self.compute_first_segment(
                current_joints,
                segments[0].from_keypoint,
                start_time_s,
                sample_clock,
            )
            result.segments.append(first_segment)
            self._accumulate_status(result, first_segment, 0)
            if self._should_stop_on_error(first_segment):
                result.build_status = BuildStatus.COMPLETED
                return result
            previous_sample = first_segment.samples[-1] if first_segment.samples else None
            start_time_s = first_segment.last_time

            previous_cart_exit_speed = 0.0
            for index, segment in enumerate(segments):
                if self._is_cancelled():
                    result.build_status = BuildStatus.CANCELLED
                    return result
                if self._is_cartesian_mode(segment.to_keypoint.mode):
                    exit_speed = self._segment_exit_speed(segments, index)
                    segment_result = self._compute_cartesian_segment(
                        segment,
                        index,
                        previous_sample,
                        start_time_s,
                        previous_cart_exit_speed,
                        exit_speed,
                        sample_clock,
                    )
                    previous_cart_exit_speed = exit_speed
                else:
                    segment_result = self.compute_PTP_segment(segment, previous_sample, start_time_s, sample_clock)
                    previous_cart_exit_speed = 0.0

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
        sample_clock: _SampleClock | None = None,
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
        return self.compute_segment(TrajectorySegment(synthetic_from, to_keypoint), None, start_time_s, sample_clock)

    def compute_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectorySample | None = None,
        start_time_s: float = 0.0,
        sample_clock: _SampleClock | None = None,
    ) -> SegmentResult:
        clock = _SampleClock(self.sample_dt_s, start_time_s) if sample_clock is None else sample_clock
        if self._is_cartesian_mode(segment.to_keypoint.mode):
            return self._compute_cartesian_segment(
                segment,
                0,
                previous_sample,
                start_time_s,
                0.0,
                0.0,
                clock,
            )
        return self.compute_PTP_segment(segment, previous_sample, start_time_s, clock)

    def compute_PTP_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectorySample | None = None,
        start_time_s: float = 0.0,
        sample_clock: _SampleClock | None = None,
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
        theoretical_duration_s = self._ptp_duration(segment, delta, 0.100)
        clock = _SampleClock(self.sample_dt_s, start_time_s) if sample_clock is None else sample_clock
        quantized_duration = clock.quantize_duration(theoretical_duration_s)
        duration_s = quantized_duration.duration_s
        end_time_s = start_time_s + duration_s
        schedule = clock.segment_points(start_time_s, quantized_duration.tick_count)
        speed_limits, accel_limits, jerk_limits = self._axis_dynamic_limits()
        previous = previous_sample
        for point in schedule:
            if self._is_cancelled():
                break
            local_time_s = max(0.0, min(duration_s, point.time_s - start_time_s))
            u = 1.0 if duration_s <= self._EPS else local_time_s / duration_s
            smooth_u = normalized_s_curve(u)
            joints = self._interpolate_joints(from_joints, delta, smooth_u)
            sample = self._build_ptp_sample(
                point.time_s,
                joints,
                previous,
                update_articular_dynamics_from_previous=False,
            )
            self._apply_ptp_analytic_articular_dynamics(sample, delta, duration_s, local_time_s)
            self._apply_dynamic_limits(sample, speed_limits, accel_limits, jerk_limits)
            result.samples.append(sample)
            self._update_joint_stats(result, sample)
            self._register_sample_error(result, sample, len(result.samples) - 1)
            previous = sample
            if self._should_stop_on_error(result):
                break

        result.duration = duration_s
        result.last_time = end_time_s
        return result

    def _compute_cartesian_segment(
        self,
        segment: TrajectorySegment,
        segment_index: int,
        previous_sample: TrajectorySample | None,
        start_time_s: float,
        entry_speed_mm_s: float,
        exit_speed_mm_s: float,
        sample_clock: _SampleClock | None = None,
    ) -> SegmentResult:
        result = SegmentResult()
        result.mode = segment.to_keypoint.mode
        runtime_segment = self._build_runtime_segment(
            segment,
            segment_index,
            entry_speed_mm_s,
            exit_speed_mm_s,
        )
        if runtime_segment is None or runtime_segment.arc_lut is None:
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            result.last_time = start_time_s
            return result

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
        clock = _SampleClock(self.sample_dt_s, start_time_s) if sample_clock is None else sample_clock
        theoretical_duration_s = max(0.0, profile.duration_s - start_time_s)
        quantized_duration = clock.quantize_duration(theoretical_duration_s)
        duration_s = quantized_duration.duration_s
        schedule = clock.segment_points(start_time_s, quantized_duration.tick_count)
        previous = previous_sample
        speed_limits, accel_limits, jerk_limits = self._axis_dynamic_limits()
        for point in schedule:
            if self._is_cancelled():
                break
            local_time_s = max(0.0, min(duration_s, point.time_s - start_time_s))
            if duration_s <= self._EPS:
                profile_time_s = profile.duration_s
            else:
                time_ratio = max(0.0, min(1.0, local_time_s / duration_s))
                profile_time_s = start_time_s + theoretical_duration_s * time_ratio
            pose = evaluator.evaluate_pose(profile_time_s)
            sample = self._build_cartesian_sample(point.time_s, pose, previous)
            self._apply_dynamic_limits(sample, speed_limits, accel_limits, jerk_limits)
            result.samples.append(sample)
            self._update_joint_stats(result, sample)
            self._register_sample_error(result, sample, len(result.samples) - 1)
            previous = sample
            if self._should_stop_on_error(result):
                break

        result.duration = duration_s
        result.last_time = start_time_s + result.duration
        return result

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
        update_articular_dynamics_from_previous: bool = True,
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
        self._update_sample_dynamics(
            sample,
            previous_sample,
            update_cartesian=True,
            update_articular=update_articular_dynamics_from_previous,
        )
        return sample

    def _update_sample_dynamics(
        self,
        sample: TrajectorySample,
        previous_sample: TrajectorySample | None,
        update_cartesian: bool = True,
        update_articular: bool = True,
    ) -> None:
        if previous_sample is None:
            if update_cartesian:
                reset_cartesian_dynamics(sample)
            if update_articular:
                reset_articular_dynamics(sample)
            return
        dt = sample.time - previous_sample.time
        if dt <= self._EPS:
            dt = self.sample_dt_s
        if update_cartesian:
            update_cartesian_dynamics(sample, previous_sample, dt)
        if update_articular:
            update_articular_dynamics(sample, previous_sample, dt)

    @staticmethod
    def _apply_ptp_analytic_articular_dynamics(
        sample: TrajectorySample,
        delta_joints_deg: JointAngles6,
        duration_s: float,
        local_time_s: float,
    ) -> None:
        reset_articular_dynamics(sample)
        if not sample.reachable:
            return
        duration = max(float(duration_s), 1e-9)
        u = max(0.0, min(1.0, float(local_time_s) / duration))
        velocity_factor = normalized_s_curve_derivative(u) / duration
        acceleration_factor = normalized_s_curve_second_derivative(u) / (duration * duration)
        jerk_factor = normalized_s_curve_third_derivative(u) / (duration * duration * duration)
        delta_values = delta_joints_deg.to_list()

        sample.articular_velocity = [delta_values[axis] * velocity_factor for axis in range(6)]
        sample.articular_acceleration = [delta_values[axis] * acceleration_factor for axis in range(6)]
        sample.articular_jerk = [delta_values[axis] * jerk_factor for axis in range(6)]
        sample.articular_velocity_valid = True
        sample.articular_acceleration_valid = True
        sample.articular_jerk_valid = True

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
