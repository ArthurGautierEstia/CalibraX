import math

from models.robot_model import RobotModel
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.trajectory_result import (
    SegmentResult,
    TrajectoryBuilderBehavior,
    TrajectoryComputationStatus,
    TrajectoryResult,
    TrajectorySample,
    TrajectorySampleErrorCode,
    TrajectorySampleMgiSolution,
    TrajectorySegment,
)
from utils.bezier3 import Bezier3Coefficients3D
import utils.math_utils as math_utils
from utils.mgi import MGI, MgiConfigKey, MgiResult, MgiResultItem, MgiResultStatus, ConfigurationIdentifier
from utils.trajectory_constants import LINEAR_TANGENT_RATIO


class TrajectoryBuilder:
    DEFAULT_SAMPLE_DT_S = 0.004  # 4 ms
    DEFAULT_ARC_LENGTH_SAMPLES = 200
    MAX_SAMPLES_PER_SEGMENT = 50_000
    LINEAR_TANGENT_RATIO = LINEAR_TANGENT_RATIO
    _EPS = 1e-9

    def __init__(
        self,
        robot_model: RobotModel,
        behavior: TrajectoryBuilderBehavior = TrajectoryBuilderBehavior.CONTINUE_ON_ERROR,
        sample_dt_s: float = DEFAULT_SAMPLE_DT_S,
    ) -> None:
        self.robot_model = robot_model
        self.behavior = behavior
        self.sample_dt_s = sample_dt_s if sample_dt_s > 0.0 else TrajectoryBuilder.DEFAULT_SAMPLE_DT_S
        self._working_mgi_solver: MGI | None = None
        self._robot_allowed_configs: set[MgiConfigKey] | None = None
        self._joint_weights: list[float] | None = None

    @staticmethod
    def linear_speed_mps_to_mmps(speed_mps: float) -> float:
        return float(speed_mps) * 1000.0

    @staticmethod
    def _new_empty_segment(
        last_time_s: float,
        mode: KeypointMotionMode,
    ) -> SegmentResult:
        segment = SegmentResult()
        segment.last_time = float(last_time_s)
        segment.mode = mode
        return segment

    @staticmethod
    def _extract_previous_sample(result: SegmentResult) -> TrajectorySample | None:
        if not result.samples:
            return None
        return result.samples[-1]

    @staticmethod
    def _accumulate_status(
        trajectory: TrajectoryResult,
        segment_result: SegmentResult,
        segment_index: int,
    ) -> None:
        if segment_result.status == TrajectoryComputationStatus.SUCCESS:
            return
        if trajectory.status != TrajectoryComputationStatus.SUCCESS:
            return
        trajectory.status = segment_result.status
        trajectory.first_error_segment_index = segment_index

    @staticmethod
    def _status_from_sample_error(error_code: TrajectorySampleErrorCode) -> TrajectoryComputationStatus:
        if error_code == TrajectorySampleErrorCode.POINT_UNREACHABLE:
            return TrajectoryComputationStatus.POINT_UNREACHABLE
        if error_code == TrajectorySampleErrorCode.OVER_DYNAMIC_LIMIT:
            return TrajectoryComputationStatus.OVER_DYNAMIC_LIMIT
        if error_code == TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION:
            return TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION
        return TrajectoryComputationStatus.SUCCESS

    @staticmethod
    def _normalize_joints_6(values: list[float]) -> list[float]:
        joints = [float(v) for v in values[:6]]
        while len(joints) < 6:
            joints.append(0.0)
        return joints

    @staticmethod
    def _shortest_angle_delta_deg(from_deg: float, to_deg: float) -> float:
        delta = (to_deg - from_deg + 180.0) % 360.0 - 180.0
        if delta == -180.0 and (to_deg - from_deg) > 0.0:
            return 180.0
        return delta

    @staticmethod
    def _wrap_angle_deg(angle_deg: float) -> float:
        wrapped = (angle_deg + 180.0) % 360.0 - 180.0
        if wrapped == -180.0 and angle_deg > 0.0:
            return 180.0
        return wrapped

    def _should_stop_on_error(self, status: TrajectoryComputationStatus) -> bool:
        if status == TrajectoryComputationStatus.SUCCESS:
            return False
        return self.behavior == TrajectoryBuilderBehavior.STOP_ON_ERROR

    def _resolve_keypoint_pose(self, keypoint: TrajectoryKeypoint) -> list[float] | None:
        if keypoint.target_type == KeypointTargetType.CARTESIAN:
            return [float(v) for v in keypoint.cartesian_target[:6]]
        return self._resolve_pose_from_joints(keypoint.joint_target)

    def _resolve_pose_from_joints(self, joints_deg: list[float]) -> list[float] | None:
        fk_result = self.robot_model.compute_fk_joints(joints_deg)
        if fk_result is None:
            return None
        _, _, dh_pose, _, _ = fk_result
        if len(dh_pose) < 6:
            return None
        return [float(v) for v in dh_pose[:6]]

    def _resolve_keypoint_joints(self, keypoint: TrajectoryKeypoint, reference_sample: TrajectorySample | None, allowed_configs: set[MgiConfigKey]) -> list[float] | None:
        if keypoint.target_type == KeypointTargetType.JOINT:
            return TrajectoryBuilder._normalize_joints_6(keypoint.joint_target)
        pose = self._resolve_keypoint_pose(keypoint)
        if pose is None:
            return None
        mgi_result = self._compute_mgi_for_pose(pose, reference_sample)
        selected_solution = self._select_best_solution(mgi_result, reference_sample, allowed_configs)
        if selected_solution is None:
            return None
        _, solution = selected_solution
        return TrajectoryBuilder._normalize_joints_6(solution.joints)

    def _resolve_PTP_segment_endpoints(self,
                                       segment: TrajectorySegment,
                                       previous_sample: TrajectorySample | None,
                                       from_allowed_configs: set[MgiConfigKey],
                                       to_allowed_configs: set[MgiConfigKey],
                                       config_identifier: ConfigurationIdentifier):
        from_joints = self._resolve_keypoint_joints(segment.from_keypoint, previous_sample, from_allowed_configs)
        if from_joints is None:
            return None, None

        to_reference_sample = TrajectorySample()
        to_reference_sample.reachable = True
        to_reference_sample.joints = TrajectoryBuilder._normalize_joints_6(from_joints)
        to_joints = self._resolve_keypoint_joints(segment.to_keypoint, to_reference_sample, to_allowed_configs)
        if to_joints is None:
            return None, None

        from_config = MgiConfigKey.identify_configuration_deg(from_joints, config_identifier)
        to_config = MgiConfigKey.identify_configuration_deg(to_joints, config_identifier)
        if from_config not in from_allowed_configs or to_config not in to_allowed_configs:
            return None, None
        return from_joints, to_joints

    def _compute_PTP_shortest_path(self, from_joints: list[float], to_joints: list[float]) -> list[float]:
        axis_limits = self.robot_model.get_axis_limits()
        joint_deltas_deg = [0.0] * 6
        for axis in range(6):
            q0 = float(from_joints[axis])
            qf = float(to_joints[axis])
            q_min, q_max = axis_limits[axis]
            q_min = float(q_min)
            q_max = float(q_max)

            if q0 < q_min - self._EPS or q0 > q_max + self._EPS:
                return None

            k_min = int(math.ceil((q_min - qf) / 360.0))
            k_max = int(math.floor((q_max - qf) / 360.0))
            if k_min > k_max:
                return None

            best_delta: float | None = None
            for k in range(k_min, k_max + 1):
                candidate_delta = (qf + 360.0 * k) - q0
                if best_delta is None or abs(candidate_delta) < abs(best_delta):
                    best_delta = candidate_delta

            if best_delta is None:
                return None

            joint_deltas_deg[axis] = float(best_delta)
        return joint_deltas_deg

    def _resolve_PTP_duration(self, segment: TrajectorySegment, joint_deltas_deg: list[float]):
        speed_ratio = max(0.0, min(1.0, float(segment.to_keypoint.ptp_speed_percent) / 100.0))
        axis_speed_limits = [float(v) for v in self.robot_model.get_axis_speed_limits()[:6]]
        while len(axis_speed_limits) < 6:
            axis_speed_limits.append(0.0)
        effective_speed_limits = [max(0.0, v * speed_ratio) for v in axis_speed_limits]

        # For s(t)=6t^5-15t^4+10t^3, max(ds/dtau)=1.875.
        quintic_peak_scale = 1.875
        required_duration_s = 0.0
        for axis in range(6):
            move = abs(joint_deltas_deg[axis])
            if move <= self._EPS:
                continue
            vmax = effective_speed_limits[axis]
            if vmax <= self._EPS:
                return None, None

            required_duration_s = max(required_duration_s, quintic_peak_scale * move / vmax)
        
        return required_duration_s, effective_speed_limits

    @staticmethod
    def _linear_tangents_from_points(p0, p3, tangent_ratio: float = LINEAR_TANGENT_RATIO):
        dx = (p3[0] - p0[0]) * tangent_ratio
        dy = (p3[1] - p0[1]) * tangent_ratio
        dz = (p3[2] - p0[2]) * tangent_ratio
        return [dx, dy, dz], [-dx, -dy, -dz]

    @staticmethod
    def _estimate_arc_length(coeffs: Bezier3Coefficients3D, samples_count: int = DEFAULT_ARC_LENGTH_SAMPLES) -> float:
        """
        Approximation de l'abscisse curviligne
        """
        if samples_count < 2:
            samples_count = 2

        prev = coeffs.point(0.0)
        total = 0.0
        for i in range(1, samples_count + 1):
            t = i / samples_count
            p = coeffs.point(t)
            total += math_utils.norm3(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2])
            prev = p
        return total

    @staticmethod
    def _resolve_num_intervals(arc_length_mm: float, speed_mmps: float, sample_dt_s: float) -> int:
        """
        Identifier le nombre d'intervalle pour parcourir la courbe à une vitesse donnée, pour un échantillonage temporel donnée.
        """
        if arc_length_mm <= TrajectoryBuilder._EPS or speed_mmps <= TrajectoryBuilder._EPS:
            return 1

        # T = D/V
        theoretical_duration_s = arc_length_mm / speed_mmps
        # intervals = T/dT
        intervals = int(math.ceil(theoretical_duration_s / sample_dt_s))
        return min(max(1, intervals), TrajectoryBuilder.MAX_SAMPLES_PER_SEGMENT)

    def _get_working_mgi_solver(self) -> MGI:
        if self._working_mgi_solver is None:
            self._working_mgi_solver = MGI(self.robot_model.mgi_params, self.robot_model.get_tool())
        return self._working_mgi_solver

    def _get_robot_allowed_configs(self) -> set[MgiConfigKey]:
        if self._robot_allowed_configs is None:
            self._robot_allowed_configs = set(self.robot_model.get_allowed_configurations())
        return self._robot_allowed_configs

    def _get_joint_weights(self) -> list[float]:
        if self._joint_weights is None:
            self._joint_weights = [float(v) for v in self.robot_model.get_joint_weights()[:6]]
        while len(self._joint_weights) < 6:
            self._joint_weights.append(1.0)
        return self._joint_weights

    @staticmethod
    def _normalize_positive_limits(values: list[float], default: float = 0.0) -> list[float]:
        out = [max(0.0, float(v)) for v in values[:6]]
        while len(out) < 6:
            out.append(max(0.0, float(default)))
        return out

    def _get_axis_dynamic_limits(self) -> tuple[list[float], list[float], list[float]]:
        # Dynamic checks must always rely on robot-model limits (no per-segment override).
        speed_limits = TrajectoryBuilder._normalize_positive_limits(self.robot_model.get_axis_speed_limits())
        accel_limits = TrajectoryBuilder._normalize_positive_limits(self.robot_model.get_axis_estimated_accel_limits())
        jerk_limits = TrajectoryBuilder._normalize_positive_limits(self.robot_model.get_axis_jerk_limits())
        return speed_limits, accel_limits, jerk_limits

    def _resolve_effective_allowed_configs(self, segment: TrajectorySegment) -> set[MgiConfigKey]:
        from_allowed = set(segment.from_keypoint.allowed_configs)
        to_allowed = set(segment.to_keypoint.allowed_configs)
        robot_allowed = self._get_robot_allowed_configs()
        return robot_allowed & from_allowed & to_allowed

    def _resolve_ptp_allowed_configs(
        self,
        segment: TrajectorySegment,
    ) -> tuple[set[MgiConfigKey], set[MgiConfigKey], set[MgiConfigKey]]:
        robot_allowed = self._get_robot_allowed_configs()
        from_allowed = robot_allowed & set(segment.from_keypoint.allowed_configs)
        to_allowed = robot_allowed & set(segment.to_keypoint.allowed_configs)
        return from_allowed, to_allowed, (from_allowed | to_allowed)

    def _get_reference_joints_for_ik(self, previous_sample: TrajectorySample | None) -> list[float]:
        if previous_sample is not None and previous_sample.reachable:
            return TrajectoryBuilder._normalize_joints_6(previous_sample.joints)
        return TrajectoryBuilder._normalize_joints_6(self.robot_model.get_joints())

    def _compute_mgi_for_pose(self, pose: list[float], previous_sample: TrajectorySample | None) -> MgiResult:
        reference_joints = self._get_reference_joints_for_ik(previous_sample)
        solver = self._get_working_mgi_solver()
        solver.set_q1ValueIfSingularityQ1Deg(reference_joints[0])
        solver.set_q4ValueIfSingularityQ5Deg(reference_joints[3])
        solver.set_q6ValueIfSingularityQ5Deg(reference_joints[5])
        return solver.compute_mgi_target(pose, returnDegrees=True)

    @staticmethod
    def _compact_mgi_solutions(
        mgi_result: MgiResult,
        allowed_configs: set[MgiConfigKey],
    ) -> dict[MgiConfigKey, TrajectorySampleMgiSolution]:
        compact: dict[MgiConfigKey, TrajectorySampleMgiSolution] = {}
        for config_key, solution in mgi_result.solutions.items():
            status_name = solution.status.name
            if solution.status == MgiResultStatus.VALID and config_key not in allowed_configs:
                status_name = MgiResultStatus.FORBIDDEN_CONFIGURATION.name
            compact[config_key] = TrajectorySampleMgiSolution(status=status_name, joints=solution.joints)
        return compact

    def _select_best_solution(
        self,
        mgi_result: MgiResult,
        previous_sample: TrajectorySample | None,
        allowed_configs: set[MgiConfigKey],
    ) -> tuple[MgiConfigKey, MgiResultItem] | None:
        reference_joints_rad = [math.radians(v) for v in self._get_reference_joints_for_ik(previous_sample)]
        joint_weights = self._get_joint_weights()

        temporarily_forbidden: list[MgiResultItem] = []
        for config_key, solution in mgi_result.solutions.items():
            if config_key in allowed_configs:
                continue
            if solution.status != MgiResultStatus.VALID:
                continue
            solution.status = MgiResultStatus.FORBIDDEN_CONFIGURATION
            temporarily_forbidden.append(solution)

        try:
            return mgi_result.get_best_solution_from_current(reference_joints_rad, joint_weights)
        finally:
            for solution in temporarily_forbidden:
                solution.status = MgiResultStatus.VALID

    @staticmethod
    def _update_joint_stats(segment_result: SegmentResult, sample: TrajectorySample) -> None:
        for axis in range(6):
            stats = segment_result.joints_stats[axis]
            vel = float(sample.articular_velocity[axis])
            acc = float(sample.articular_acceleration[axis])
            if vel > stats.max_positive_velocity:
                stats.max_positive_velocity = vel
            if vel < stats.max_negative_velocity:
                stats.max_negative_velocity = vel
            if acc > stats.max_acceleration:
                stats.max_acceleration = acc
            if acc < stats.max_deceleration:
                stats.max_deceleration = acc

    def _register_sample_error(self, result: SegmentResult, sample: TrajectorySample, sample_index: int) -> None:
        if sample.error_code == TrajectorySampleErrorCode.NONE:
            return
        if result.first_error_sample_index is None:
            result.first_error_sample_index = sample_index
            result.first_error_axis = sample.error_axis
        if result.status == TrajectoryComputationStatus.SUCCESS:
            result.status = TrajectoryBuilder._status_from_sample_error(sample.error_code)

    def _update_articular_dynamics(
        self,
        sample: TrajectorySample,
        previous_sample: TrajectorySample | None,
        dt: float,
    ) -> None:
        sample.articular_velocity = [0.0] * 6
        sample.articular_acceleration = [0.0] * 6
        if previous_sample is None:
            return
        if not sample.reachable or not previous_sample.reachable:
            return

        for axis in range(6):
            vel = (sample.joints[axis] - previous_sample.joints[axis]) / dt
            sample.articular_velocity[axis] = vel
            sample.articular_acceleration[axis] = (vel - previous_sample.articular_velocity[axis]) / dt

    def _apply_dynamic_limits_if_needed(
        self,
        sample: TrajectorySample,
        previous_sample: TrajectorySample | None,
        speed_limits_deg_s: list[float],
        accel_limits_deg_s2: list[float],
        jerk_limits_deg_s3: list[float],
    ) -> None:
        if sample.error_code != TrajectorySampleErrorCode.NONE:
            return
        if previous_sample is None:
            return
        if not sample.reachable or not previous_sample.reachable:
            return

        dt = sample.time - previous_sample.time
        if dt <= self._EPS:
            dt = self.sample_dt_s

        eps = 1e-6
        for axis in range(6):
            vel = abs(float(sample.articular_velocity[axis]))
            if vel > float(speed_limits_deg_s[axis]) + eps:
                sample.error_code = TrajectorySampleErrorCode.OVER_DYNAMIC_LIMIT
                sample.error_axis = axis
                return

            acc = abs(float(sample.articular_acceleration[axis]))
            if acc > float(accel_limits_deg_s2[axis]) + eps:
                sample.error_code = TrajectorySampleErrorCode.OVER_DYNAMIC_LIMIT
                sample.error_axis = axis
                return

            jerk_limit = float(jerk_limits_deg_s3[axis])
            if jerk_limit <= self._EPS:
                continue
            jerk = abs(
                (float(sample.articular_acceleration[axis]) - float(previous_sample.articular_acceleration[axis])) / dt
            )
            if jerk > jerk_limit + eps:
                sample.error_code = TrajectorySampleErrorCode.OVER_DYNAMIC_LIMIT
                sample.error_axis = axis
                return

    @staticmethod
    def _resolve_error_for_missing_selected_solution(
        mgi_result: MgiResult,
        allowed_configs: set[MgiConfigKey],
    ) -> TrajectorySampleErrorCode:
        has_any_valid = False
        has_allowed_valid = False
        for config_key, solution in mgi_result.solutions.items():
            if solution.status != MgiResultStatus.VALID:
                continue
            has_any_valid = True
            if config_key in allowed_configs:
                has_allowed_valid = True
                break
        if has_any_valid and not has_allowed_valid:
            return TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION
        return TrajectorySampleErrorCode.POINT_UNREACHABLE

    def select_config(
        self,
        sample: TrajectorySample,
        config_key: MgiConfigKey,
        previous_sample: TrajectorySample | None = None,
    ) -> bool:
        selected = sample.mgi_solutions.get(config_key)
        if selected is None:
            return False
        if selected.status != MgiResultStatus.VALID.name:
            return False

        sample.configuration = config_key
        sample.joints = TrajectoryBuilder._normalize_joints_6(selected.joints)
        sample.reachable = True
        sample.error_code = TrajectorySampleErrorCode.NONE
        sample.error_axis = None

        dt = self.sample_dt_s
        if previous_sample is not None:
            dt = sample.time - previous_sample.time
            if dt <= self._EPS:
                dt = self.sample_dt_s
        self._update_articular_dynamics(sample, previous_sample, dt)
        # speed_limits, accel_limits, jerk_limits = self._get_axis_dynamic_limits()
        # self._apply_dynamic_limits_if_needed(sample, previous_sample, speed_limits, accel_limits, jerk_limits)
        return True

    def compute_trajectory(
        self,
        current_joints: list[float],
        segments: list[TrajectorySegment],
    ) -> TrajectoryResult:
        trajectory = TrajectoryResult()
        if not segments:
            return trajectory

        self._working_mgi_solver = MGI(self.robot_model.mgi_params, self.robot_model.get_tool())
        self._robot_allowed_configs = set(self.robot_model.get_allowed_configurations())
        self._joint_weights = [float(v) for v in self.robot_model.get_joint_weights()[:6]]
        try:
            previous_sample: TrajectorySample | None = None
            start_time_s = 0.0

            first_result = self.compute_first_segment(current_joints, segments[0].from_keypoint, start_time_s)
            trajectory.segments.append(first_result)
            TrajectoryBuilder._accumulate_status(trajectory, first_result, segment_index=0)
            if self._should_stop_on_error(first_result.status):
                return trajectory

            previous_sample = TrajectoryBuilder._extract_previous_sample(first_result)
            start_time_s = first_result.last_time

            for idx, segment in enumerate(segments, start=1):
                segment_result = self.compute_segment(segment, previous_sample, start_time_s)
                trajectory.segments.append(segment_result)
                TrajectoryBuilder._accumulate_status(trajectory, segment_result, segment_index=idx)
                if self._should_stop_on_error(segment_result.status):
                    break

                previous_sample = TrajectoryBuilder._extract_previous_sample(segment_result)
                start_time_s = segment_result.last_time
            return trajectory
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
        joints_6 = TrajectoryBuilder._normalize_joints_6(current_joints)
        config_identifier = self.robot_model.get_config_identifier()
        identified_config = MgiConfigKey.identify_configuration_deg(joints_6, config_identifier)

        synthetic_from = TrajectoryKeypoint(
            target_type=KeypointTargetType.JOINT,
            joint_target=joints_6,
            mode=to_keypoint.mode,
            cubic_vectors=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            allowed_configs=[identified_config],
            ptp_speed_percent=to_keypoint.ptp_speed_percent,
            linear_speed_mps=to_keypoint.linear_speed_mps,
        )
        segment = TrajectorySegment(synthetic_from, to_keypoint)
        return self.compute_segment(segment, None, start_time_s)

    def compute_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        if segment.to_keypoint.mode == KeypointMotionMode.LINEAR:
            return self.compute_LIN_segment(segment, previous_sample, start_time_s)
        if segment.to_keypoint.mode == KeypointMotionMode.CUBIC:
            return self.compute_cubique_segment(segment, previous_sample, start_time_s)
        return self.compute_PTP_segment(segment, previous_sample, start_time_s)

    def compute_PTP_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        result = SegmentResult()
        result.status = TrajectoryComputationStatus.SUCCESS
        result.mode = segment.to_keypoint.mode

        # 1) Resolve allowed configurations for PTP endpoints independently.
        #    This keeps PTP simple and allows start/end in different configs.
        from_allowed_configs, to_allowed_configs, sample_allowed_configs = self._resolve_ptp_allowed_configs(segment)
        if not from_allowed_configs or not to_allowed_configs:
            result.status = TrajectoryComputationStatus.NO_COMMON_ALLOWED_CONFIGURATION
            result.last_time = float(start_time_s)
            return result

        # 2) Resolve segment endpoints in joint space.
        config_identifier = self.robot_model.get_config_identifier()

        from_joints, to_joints = self._resolve_PTP_segment_endpoints(
            segment,
            previous_sample,
            from_allowed_configs,
            to_allowed_configs,
            config_identifier,
        )
        if from_joints is None or to_joints is None:
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            result.last_time = float(start_time_s)
            return result

        # 3) Compute shortest feasible delta on each axis while staying in joint limits.
        joint_deltas_deg = self._compute_PTP_shortest_path(from_joints, to_joints)
        if joint_deltas_deg is None:
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            result.last_time = float(start_time_s)
            return result

        # 4) Build synchronized timing from speed limits and requested PTP speed percent.
        required_duration_s, effective_speed_limits = self._resolve_PTP_duration(segment, joint_deltas_deg)
        if required_duration_s is None or effective_speed_limits is None:
            result.status = TrajectoryComputationStatus.OVER_DYNAMIC_LIMIT
            result.last_time = float(start_time_s)
            return result

        intervals = 1
        if required_duration_s > self._EPS:
            intervals = int(math.ceil(required_duration_s / self.sample_dt_s))
        intervals = min(max(1, intervals), TrajectoryBuilder.MAX_SAMPLES_PER_SEGMENT)

        result.duration = intervals * self.sample_dt_s
        result.last_time = start_time_s + result.duration

        # 5) Evaluate joint laws and fill full sample payload (pose, dynamics, errors, stats).
        self._generate_PTP_segment(
            previous_sample,
            start_time_s,
            intervals,
            from_joints,
            joint_deltas_deg,
            sample_allowed_configs,
            config_identifier,
            result,
        )
        return result

    def compute_LIN_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        # Straight line encoded as cubic bezier.
        return self._generate_bezier_segment(
            segment,
            previous_sample,
            start_time_s,
            force_linear_handles=True,
        )

    def compute_cubique_segment(
        self,
        segment: TrajectorySegment,
        previous_sample: TrajectorySample | None = None,
        start_time_s: float = 0.0,
    ) -> SegmentResult:
        return self._generate_bezier_segment(
            segment,
            previous_sample,
            start_time_s,
            force_linear_handles=False,
        )

    def _generate_bezier_segment(
        self,
        segment: TrajectorySegment,
        previous_segment_last_sample: TrajectorySample | None = None,
        start_time_s: float = 0.0,
        force_linear_handles: bool = False,
    ) -> SegmentResult:
        from_pose = self._resolve_keypoint_pose(segment.from_keypoint)
        to_pose = self._resolve_keypoint_pose(segment.to_keypoint)
        if from_pose is None or to_pose is None:
            result = TrajectoryBuilder._new_empty_segment(start_time_s, segment.to_keypoint.mode)
            result.status = TrajectoryComputationStatus.POINT_UNREACHABLE
            return result

        effective_allowed_configs = self._resolve_effective_allowed_configs(segment)
        if not effective_allowed_configs:
            result = TrajectoryBuilder._new_empty_segment(start_time_s, segment.to_keypoint.mode)
            result.status = TrajectoryComputationStatus.NO_COMMON_ALLOWED_CONFIGURATION
            return result

        p0 = [from_pose[0], from_pose[1], from_pose[2]]
        p3 = [to_pose[0], to_pose[1], to_pose[2]]
        segment_length_mm = math_utils.norm3(p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2])

        if force_linear_handles:
            t_out, t_in = self._linear_tangents_from_points(
                p0,
                p3,
                TrajectoryBuilder.LINEAR_TANGENT_RATIO,
            )
        else:
            # Cubic handles are carried by the destination keypoint (segment-in semantics):
            # [0] = start-side direction/amplitude, [1] = end-side direction/amplitude.
            t_out, t_in = segment.to_keypoint.resolve_cubic_tangent_vectors(segment_length_mm)

        coeffs = Bezier3Coefficients3D(p0, p3, t_out, t_in)
        arc_length_mm = TrajectoryBuilder._estimate_arc_length(coeffs)
        speed_mmps = TrajectoryBuilder.linear_speed_mps_to_mmps(segment.to_keypoint.linear_speed_mps)
        intervals = TrajectoryBuilder._resolve_num_intervals(arc_length_mm, speed_mmps, self.sample_dt_s)

        result = SegmentResult()
        result.status = TrajectoryComputationStatus.SUCCESS
        result.mode = segment.to_keypoint.mode
        result.duration = intervals * self.sample_dt_s
        result.last_time = start_time_s + result.duration
        
        result.in_direction = t_in
        result.out_direction = t_out
        speed_limits_deg_s, accel_limits_deg_s2, jerk_limits_deg_s3 = self._get_axis_dynamic_limits()

        dA = TrajectoryBuilder._shortest_angle_delta_deg(from_pose[3], to_pose[3])
        dB = TrajectoryBuilder._shortest_angle_delta_deg(from_pose[4], to_pose[4])
        dC = TrajectoryBuilder._shortest_angle_delta_deg(from_pose[5], to_pose[5])

        previous_sample = previous_segment_last_sample
        for i in range(1, intervals + 1):
            time_s = start_time_s + i * self.sample_dt_s
            linear_t = i / intervals
            smooth_t3, smooth_t5 = math_utils.pair_cubic_quintic_transition(linear_t)
            xyz = coeffs.point(smooth_t3)
            orientation_abc = [
                TrajectoryBuilder._wrap_angle_deg(from_pose[3] + dA * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(from_pose[4] + dB * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(from_pose[5] + dC * smooth_t5),
            ]

            sample = self._build_sample_from_cartesian(
                time_s,
                xyz,
                orientation_abc,
                previous_sample,
                effective_allowed_configs,
                speed_limits_deg_s,
                accel_limits_deg_s2,
                jerk_limits_deg_s3,
            )
            result.samples.append(sample)
            TrajectoryBuilder._update_joint_stats(result, sample)
            self._register_sample_error(result, sample, sample_index=i - 1)
            previous_sample = sample

            if self._should_stop_on_error(result.status):
                break

        generated_intervals = len(result.samples)
        result.duration = generated_intervals * self.sample_dt_s
        result.last_time = start_time_s + result.duration
        return result

    def _build_sample_from_cartesian(
        self,
        time_s: float,
        xyz: list[float],
        orientation_abc: list[float],
        previous_sample: TrajectorySample | None,
        allowed_configs: set[MgiConfigKey],
        speed_limits_deg_s: list[float],
        accel_limits_deg_s2: list[float],
        jerk_limits_deg_s3: list[float],
    ) -> TrajectorySample:
        sample = TrajectorySample()
        sample.time = float(time_s)
        sample.pose[0] = float(xyz[0])
        sample.pose[1] = float(xyz[1])
        sample.pose[2] = float(xyz[2])
        sample.pose[3] = float(orientation_abc[0])
        sample.pose[4] = float(orientation_abc[1])
        sample.pose[5] = float(orientation_abc[2])

        mgi_result = self._compute_mgi_for_pose(sample.pose, previous_sample)
        sample.mgi_solutions = TrajectoryBuilder._compact_mgi_solutions(mgi_result, allowed_configs)
        selected_solution = self._select_best_solution(mgi_result, previous_sample, allowed_configs)
        if selected_solution is None:
            sample.reachable = False
            sample.error_code = TrajectoryBuilder._resolve_error_for_missing_selected_solution(
                mgi_result,
                allowed_configs,
            )
            sample.configuration = None
            sample.joints = [0.0] * 6
        else:
            config_key, solution = selected_solution
            sample.reachable = True
            sample.error_code = TrajectorySampleErrorCode.NONE
            sample.configuration = config_key
            sample.joints = TrajectoryBuilder._normalize_joints_6(solution.joints)

        if previous_sample is None:
            sample.cartesian_velocity = [0.0] * 6
            sample.cartesian_acceleration = [0.0] * 6
            sample.articular_velocity = [0.0] * 6
            sample.articular_acceleration = [0.0] * 6
            sample.velocity = 0.0
            sample.acceleration = 0.0
            return sample

        dt = sample.time - previous_sample.time
        if dt <= self._EPS:
            dt = self.sample_dt_s

        dA = TrajectoryBuilder._shortest_angle_delta_deg(previous_sample.pose[3], sample.pose[3])
        dB = TrajectoryBuilder._shortest_angle_delta_deg(previous_sample.pose[4], sample.pose[4])
        dC = TrajectoryBuilder._shortest_angle_delta_deg(previous_sample.pose[5], sample.pose[5])

        sample.cartesian_velocity[0] = (sample.pose[0] - previous_sample.pose[0]) / dt
        sample.cartesian_velocity[1] = (sample.pose[1] - previous_sample.pose[1]) / dt
        sample.cartesian_velocity[2] = (sample.pose[2] - previous_sample.pose[2]) / dt
        sample.cartesian_velocity[3] = dA / dt
        sample.cartesian_velocity[4] = dB / dt
        sample.cartesian_velocity[5] = dC / dt

        for axis in range(6):
            sample.cartesian_acceleration[axis] = (
                sample.cartesian_velocity[axis] - previous_sample.cartesian_velocity[axis]
            ) / dt

        sample.velocity = math_utils.norm3(
            sample.cartesian_velocity[0],
            sample.cartesian_velocity[1],
            sample.cartesian_velocity[2],
        )
        sample.acceleration = math_utils.norm3(
            sample.cartesian_acceleration[0],
            sample.cartesian_acceleration[1],
            sample.cartesian_acceleration[2],
        )

        self._update_articular_dynamics(sample, previous_sample, dt)
        # self._apply_dynamic_limits_if_needed(
        #     sample,
        #     previous_sample,
        #     speed_limits_deg_s,
        #     accel_limits_deg_s2,
        #     jerk_limits_deg_s3,
        # )
        return sample

    def _build_sample_from_ptp_joints(
        self,
        time_s: float,
        joints_deg: list[float],
        previous_sample: TrajectorySample | None,
        allowed_configs: set[MgiConfigKey],
        config_identifier: ConfigurationIdentifier,
        speed_limits_deg_s: list[float],
        accel_limits_deg_s2: list[float],
        jerk_limits_deg_s3: list[float],
    ) -> TrajectorySample:
        # A) Create base sample payload from joint state at current time.
        sample = TrajectorySample()
        sample.time = float(time_s)
        sample.joints = TrajectoryBuilder._normalize_joints_6(joints_deg)

        # B) Compute pose from FK and expose configuration reachability metadata.
        fk_result = self.robot_model.compute_fk_joints(sample.joints)
        if fk_result is None:
            sample.reachable = False
            sample.error_code = TrajectorySampleErrorCode.POINT_UNREACHABLE
            sample.configuration = None
            sample.pose = [0.0] * 6
            sample.mgi_solutions = {}
        else:
            _, _, dh_pose, _, _ = fk_result
            sample.pose = [float(v) for v in dh_pose[:6]]
            while len(sample.pose) < 6:
                sample.pose.append(0.0)

            config_key = MgiConfigKey.identify_configuration_deg(sample.joints, config_identifier)
            config_status = (
                MgiResultStatus.VALID.name
                if config_key in allowed_configs
                else MgiResultStatus.FORBIDDEN_CONFIGURATION.name
            )
            sample.mgi_solutions = {
                config_key: TrajectorySampleMgiSolution(status=config_status, joints=sample.joints)
            }

            if config_status != MgiResultStatus.VALID.name:
                sample.reachable = False
                sample.error_code = TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION
                sample.configuration = None
            else:
                sample.reachable = True
                sample.error_code = TrajectorySampleErrorCode.NONE
                sample.configuration = config_key

        # C) Compute cartesian/articular dynamics from previous sample.
        if previous_sample is None:
            sample.cartesian_velocity = [0.0] * 6
            sample.cartesian_acceleration = [0.0] * 6
            sample.articular_velocity = [0.0] * 6
            sample.articular_acceleration = [0.0] * 6
            sample.velocity = 0.0
            sample.acceleration = 0.0
            return sample

        dt = sample.time - previous_sample.time
        if dt <= self._EPS:
            dt = self.sample_dt_s

        dA = TrajectoryBuilder._shortest_angle_delta_deg(previous_sample.pose[3], sample.pose[3])
        dB = TrajectoryBuilder._shortest_angle_delta_deg(previous_sample.pose[4], sample.pose[4])
        dC = TrajectoryBuilder._shortest_angle_delta_deg(previous_sample.pose[5], sample.pose[5])

        sample.cartesian_velocity[0] = (sample.pose[0] - previous_sample.pose[0]) / dt
        sample.cartesian_velocity[1] = (sample.pose[1] - previous_sample.pose[1]) / dt
        sample.cartesian_velocity[2] = (sample.pose[2] - previous_sample.pose[2]) / dt
        sample.cartesian_velocity[3] = dA / dt
        sample.cartesian_velocity[4] = dB / dt
        sample.cartesian_velocity[5] = dC / dt

        for axis in range(6):
            sample.cartesian_acceleration[axis] = (
                sample.cartesian_velocity[axis] - previous_sample.cartesian_velocity[axis]
            ) / dt

        sample.velocity = math_utils.norm3(
            sample.cartesian_velocity[0],
            sample.cartesian_velocity[1],
            sample.cartesian_velocity[2],
        )
        sample.acceleration = math_utils.norm3(
            sample.cartesian_acceleration[0],
            sample.cartesian_acceleration[1],
            sample.cartesian_acceleration[2],
        )
        self._update_articular_dynamics(sample, previous_sample, dt)
        # self._apply_dynamic_limits_if_needed(
        #     sample,
        #     previous_sample,
        #     speed_limits_deg_s,
        #     accel_limits_deg_s2,
        #     jerk_limits_deg_s3,
        # )

        return sample

    def _generate_PTP_segment(
        self,
        previous_sample: TrajectorySample | None,
        start_time_s: float,
        intervals: int,
        from_joints: list[float],
        joint_deltas_deg: list[float],
        effective_allowed_configs: set[MgiConfigKey],
        config_identifier: ConfigurationIdentifier,
        result: SegmentResult,
    ):
        speed_limits, accel_limits, jerk_limits = self._get_axis_dynamic_limits()

        # This loop only orchestrates sampling:
        # - evaluate the joint law,
        # - build one sample,
        # - append and update segment-level aggregates.
        previous_generated_sample = previous_sample

        for i in range(1, intervals + 1):
            time_s = start_time_s + i * self.sample_dt_s

            linear_t = i / intervals
            smooth_t = math_utils.quintic_transition(linear_t)
            joints_deg = [from_joints[axis] + joint_deltas_deg[axis] * smooth_t for axis in range(6)]

            sample = self._build_sample_from_ptp_joints(
                time_s=time_s,
                joints_deg=joints_deg,
                previous_sample=previous_generated_sample,
                allowed_configs=effective_allowed_configs,
                config_identifier=config_identifier,
                speed_limits_deg_s=speed_limits,
                accel_limits_deg_s2=accel_limits,
                jerk_limits_deg_s3=jerk_limits,
            )

            # Segment aggregation is intentionally separated from sample creation.
            result.samples.append(sample)
            TrajectoryBuilder._update_joint_stats(result, sample)
            self._register_sample_error(result, sample, sample_index=i - 1)
            previous_generated_sample = sample

            if self._should_stop_on_error(result.status):
                break

        generated_intervals = len(result.samples)
        result.duration = generated_intervals * self.sample_dt_s
        result.last_time = start_time_s + result.duration
