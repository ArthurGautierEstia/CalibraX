import math
import bisect
from dataclasses import dataclass
import numpy as np

from models.robot_model import RobotModel
from models.workspace_model import WorkspaceModel
from models.tool_model import ToolModel
from models.trajectory_keypoint import (
    ConfigurationPolicy,
    KeypointMotionMode,
    KeypointTargetType,
    TrajectoryKeypoint,
)
from models.trajectory_options import TrajectoryBezierDegree
from models.trajectory_result import (
    TrajectoryCollisionDiagnostic,
    TrajectoryCollisionDomain,
    TrajectoryDynamicViolation,
    TrajectoryDynamicViolationKind,
    TrajectoryDynamicViolationSeverity,
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
from utils.bezier5 import Bezier5Coefficients3D
from utils.collision_utils import (
    CollisionPair,
    CollisionWorldCache,
    build_world_frame_transforms,
    resolve_flange_world_transform,
)
import utils.math_utils as math_utils
from utils.reference_frame_utils import convert_pose_to_base_frame
from utils.mgi import MGI, MgiConfigKey, MgiResult, MgiResultItem, MgiResultStatus, ConfigurationIdentifier


@dataclass
class _BezierSegmentDescriptor:
    segment: TrajectorySegment
    from_pose: list[float]
    to_pose: list[float]
    coeffs: Bezier3Coefficients3D | Bezier5Coefficients3D
    t_out: list[float]
    t_in: list[float]
    arc_length_mm: float
    speed_mmps: float
    arc_lut_t: list[float]
    arc_lut_s: list[float]


class TrajectoryBuilder:
    DEFAULT_SAMPLE_DT_S = 0.004  # 4 ms
    DEFAULT_ARC_LENGTH_SAMPLES = 200
    MAX_SAMPLES_PER_SEGMENT = 50_000
    QUINTIC_PEAK_SPEED_SCALE = 1.875
    _EPS = 1e-9
    # Heuristic thresholds to detect "teleportation-like" joint jumps.
    # These values are intentionally high to avoid flagging ordinary overspeed cases.
    CONFIG_JUMP_ACCEL_THRESHOLD_DEG_S2 = 20_000.0
    CONFIG_JUMP_DELTA_FACTOR = 6.0
    CONFIG_JUMP_SPEED_FACTOR = 8.0
    CONFIG_JUMP_MIN_DELTA_DEG = 45.0
    CONFIG_JUMP_MIN_SPEED_DEG_S = 2_000.0
    SUPER_CHAIN_COLLINEAR_ANGLE_TOL_DEG = 0.2
    SUPER_CHAIN_MIN_VECTOR_NORM = 1e-6

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        behavior: TrajectoryBuilderBehavior = TrajectoryBuilderBehavior.CONTINUE_ON_ERROR,
        sample_dt_s: float = DEFAULT_SAMPLE_DT_S,
        smooth_time_enabled: bool = True,
        bezier_degree: TrajectoryBezierDegree | str = TrajectoryBezierDegree.BEZIER5,
        jerk_check_enabled: bool = True,
    ) -> None:
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.behavior = behavior
        self.sample_dt_s = sample_dt_s if sample_dt_s > 0.0 else TrajectoryBuilder.DEFAULT_SAMPLE_DT_S
        self.smooth_time_enabled = bool(smooth_time_enabled)
        self.bezier_degree = TrajectoryBezierDegree.from_value(bezier_degree)
        self.jerk_check_enabled = bool(jerk_check_enabled)
        self._working_mgi_solver: MGI | None = None
        self._robot_allowed_configs: set[MgiConfigKey] | None = None
        self._joint_weights: list[float] | None = None
        self._collision_world_cache: CollisionWorldCache | None = None
        self._collision_cache_revisions: tuple[int, int, int] | None = None

    def set_time_smoothing_enabled(self, enabled: bool) -> None:
        self.smooth_time_enabled = bool(enabled)

    def set_bezier_degree(self, degree: TrajectoryBezierDegree | str) -> None:
        self.bezier_degree = TrajectoryBezierDegree.from_value(degree)

    def set_jerk_check_enabled(self, enabled: bool) -> None:
        self.jerk_check_enabled = bool(enabled)

    def _get_collision_cache_revisions(self) -> tuple[int, int, int]:
        return (
            int(self.workspace_model.get_workspace_structure_revision()),
            int(self.robot_model.get_axis_colliders_revision()),
            int(self.tool_model.get_tool_colliders_revision()),
        )

    def _ensure_collision_world_cache(self) -> CollisionWorldCache:
        revisions = self._get_collision_cache_revisions()
        if self._collision_world_cache is None:
            self._collision_world_cache = CollisionWorldCache()

        if revisions != self._collision_cache_revisions:
            self._collision_world_cache.set_workspace_collision_zones(
                self.workspace_model.get_workspace_collision_zones()
            )
            self._collision_world_cache.set_robot_axis_templates(self.robot_model.get_axis_colliders())
            self._collision_world_cache.set_tool_templates(self.tool_model.get_tool_colliders())
            self._collision_cache_revisions = revisions

        return self._collision_world_cache

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
        if error_code == TrajectorySampleErrorCode.CONFIGURATION_JUMP:
            return TrajectoryComputationStatus.CONFIGURATION_JUMP
        if error_code == TrajectorySampleErrorCode.COLLISION_DETECTED:
            return TrajectoryComputationStatus.COLLISION_DETECTED
        if error_code == TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED:
            return TrajectoryComputationStatus.SPEED_LIMIT_EXCEEDED
        if error_code == TrajectorySampleErrorCode.JERK_LIMIT_EXCEEDED:
            return TrajectoryComputationStatus.JERK_LIMIT_EXCEEDED
        if error_code == TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION:
            return TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION
        return TrajectoryComputationStatus.SUCCESS

    @staticmethod
    def _collision_diagnostics_from_pairs(
        pairs: list[CollisionPair],
        domain: TrajectoryCollisionDomain,
    ) -> list[TrajectoryCollisionDiagnostic]:
        diagnostics: list[TrajectoryCollisionDiagnostic] = []
        for pair in pairs:
            diagnostics.append(
                TrajectoryCollisionDiagnostic(
                    domain=domain,
                    owner_a=pair.shape_a.owner,
                    name_a=pair.shape_a.name,
                    source_index_a=pair.shape_a.source_index,
                    owner_b=pair.shape_b.owner,
                    name_b=pair.shape_b.name,
                    source_index_b=pair.shape_b.source_index,
                )
            )
        return diagnostics

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
            return convert_pose_to_base_frame(
                keypoint.cartesian_target,
                keypoint.cartesian_frame,
                self.workspace_model.get_robot_base_transform_world(),
            )
        return self._resolve_pose_from_joints(keypoint.joint_target)

    def _resolve_pose_from_joints(self, joints_deg: list[float]) -> list[float] | None:
        fk_result = self.robot_model.compute_fk_joints(joints_deg, tool=self.tool_model.get_tool())
        if fk_result is None:
            return None
        _, _, dh_pose, _, _ = fk_result
        if len(dh_pose) < 6:
            return None
        return [float(v) for v in dh_pose[:6]]

    def _resolve_reference_config(self, reference_sample: TrajectorySample | None) -> MgiConfigKey:
        config_identifier = self.robot_model.get_config_identifier()
        reference_joints = self._get_reference_joints_for_ik(reference_sample)
        return MgiConfigKey.identify_configuration_deg(reference_joints, config_identifier)

    def _resolve_allowed_configs_for_keypoint(
        self,
        keypoint: TrajectoryKeypoint,
        reference_sample: TrajectorySample | None,
    ) -> set[MgiConfigKey]:
        robot_allowed = self._get_robot_allowed_configs()
        if not robot_allowed:
            return set()

        # Joint targets already define their branch through the provided joint values.
        if keypoint.target_type == KeypointTargetType.JOINT:
            config_identifier = self.robot_model.get_config_identifier()
            joint_config = MgiConfigKey.identify_configuration_deg(
                TrajectoryBuilder._normalize_joints_6(keypoint.joint_target),
                config_identifier,
            )
            return ({joint_config} & robot_allowed)

        if keypoint.configuration_policy == ConfigurationPolicy.AUTO:
            return set(robot_allowed)

        if keypoint.configuration_policy == ConfigurationPolicy.CURRENT_BRANCH:
            current_branch = self._resolve_reference_config(reference_sample)
            return ({current_branch} & robot_allowed)

        if keypoint.configuration_policy == ConfigurationPolicy.FORCED and keypoint.forced_config is not None:
            return ({keypoint.forced_config} & robot_allowed)

        return set()

    def _resolve_keypoint_joints(
        self,
        keypoint: TrajectoryKeypoint,
        reference_sample: TrajectorySample | None,
        allowed_configs: set[MgiConfigKey],
    ) -> tuple[list[float] | None, TrajectorySampleErrorCode]:
        if keypoint.target_type == KeypointTargetType.JOINT:
            joints = TrajectoryBuilder._normalize_joints_6(keypoint.joint_target)
            config_identifier = self.robot_model.get_config_identifier()
            joint_config = MgiConfigKey.identify_configuration_deg(joints, config_identifier)
            if joint_config not in allowed_configs:
                return None, TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION
            return joints, TrajectorySampleErrorCode.NONE

        pose = self._resolve_keypoint_pose(keypoint)
        if pose is None:
            return None, TrajectorySampleErrorCode.POINT_UNREACHABLE

        mgi_result = self._compute_mgi_for_pose(pose, reference_sample)
        selected_solution = self._select_best_solution(mgi_result, reference_sample, allowed_configs)
        if selected_solution is None:
            return None, TrajectoryBuilder._resolve_error_for_missing_selected_solution(mgi_result, allowed_configs)

        _, solution = selected_solution
        return TrajectoryBuilder._normalize_joints_6(solution.joints), TrajectorySampleErrorCode.NONE

    def _resolve_PTP_segment_endpoints(self,
                                       segment: TrajectorySegment,
                                       previous_sample: TrajectorySample | None,
                                       from_allowed_configs: set[MgiConfigKey],
                                       config_identifier: ConfigurationIdentifier):
        from_joints, from_error = self._resolve_keypoint_joints(
            segment.from_keypoint,
            previous_sample,
            from_allowed_configs,
        )
        if from_joints is None:
            return None, None, from_error

        to_reference_sample = TrajectorySample()
        to_reference_sample.reachable = True
        to_reference_sample.joints = TrajectoryBuilder._normalize_joints_6(from_joints)
        to_allowed_configs = self._resolve_allowed_configs_for_keypoint(
            segment.to_keypoint,
            to_reference_sample,
        )
        if not to_allowed_configs:
            return None, None, TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION
        to_joints, to_error = self._resolve_keypoint_joints(
            segment.to_keypoint,
            to_reference_sample,
            to_allowed_configs,
        )
        if to_joints is None:
            return None, None, to_error

        from_config = MgiConfigKey.identify_configuration_deg(from_joints, config_identifier)
        to_config = MgiConfigKey.identify_configuration_deg(to_joints, config_identifier)
        if from_config not in from_allowed_configs or to_config not in to_allowed_configs:
            return None, None, TrajectorySampleErrorCode.FORBIDDEN_CONFIGURATION
        return from_joints, to_joints, TrajectorySampleErrorCode.NONE

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

        required_duration_s = 0.0
        for axis in range(6):
            move = abs(joint_deltas_deg[axis])
            if move <= self._EPS:
                continue
            vmax = effective_speed_limits[axis]
            if vmax <= self._EPS:
                return None, None

            required_duration_s = max(
                required_duration_s,
                TrajectoryBuilder.QUINTIC_PEAK_SPEED_SCALE * move / vmax,
            )
        
        return required_duration_s, effective_speed_limits

    @staticmethod
    def _estimate_arc_length(
        coeffs: Bezier3Coefficients3D | Bezier5Coefficients3D,
        samples_count: int = DEFAULT_ARC_LENGTH_SAMPLES,
    ) -> float:
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
    def _resolve_num_intervals(
        arc_length_mm: float,
        speed_mmps: float,
        sample_dt_s: float,
        peak_speed_scale: float = 1.0,
    ) -> int:
        """
        Identifier le nombre d'intervalle pour parcourir la courbe à une vitesse donnée, pour un échantillonage temporel donnée.
        """
        if arc_length_mm <= TrajectoryBuilder._EPS or speed_mmps <= TrajectoryBuilder._EPS:
            return 1

        # T = D/V, scaled when the temporal profile has a peak velocity above its average.
        profile_scale = max(1.0, float(peak_speed_scale))
        theoretical_duration_s = profile_scale * arc_length_mm / speed_mmps
        # intervals = T/dT
        intervals = int(math.ceil(theoretical_duration_s / sample_dt_s))
        return min(max(1, intervals), TrajectoryBuilder.MAX_SAMPLES_PER_SEGMENT)

    def _time_profile_peak_speed_scale(self) -> float:
        if self.smooth_time_enabled:
            return TrajectoryBuilder.QUINTIC_PEAK_SPEED_SCALE
        return 1.0

    @staticmethod
    def _is_lin_or_cubic_mode(mode: KeypointMotionMode) -> bool:
        return mode in (KeypointMotionMode.LINEAR, KeypointMotionMode.CUBIC)

    @staticmethod
    def _build_arc_length_lut(
        coeffs: Bezier3Coefficients3D | Bezier5Coefficients3D,
        samples_count: int = DEFAULT_ARC_LENGTH_SAMPLES,
    ) -> tuple[list[float], list[float]]:
        if samples_count < 2:
            samples_count = 2
        t_values = [0.0]
        s_values = [0.0]
        prev = coeffs.point(0.0)
        cumulative = 0.0
        for i in range(1, samples_count + 1):
            t = i / samples_count
            p = coeffs.point(t)
            cumulative += math_utils.norm3(p[0] - prev[0], p[1] - prev[1], p[2] - prev[2])
            t_values.append(float(t))
            s_values.append(float(cumulative))
            prev = p
        return t_values, s_values

    def _build_bezier_coefficients(
        self,
        p0: list[float],
        p3: list[float],
        t_out: list[float],
        t_in: list[float],
    ) -> Bezier3Coefficients3D | Bezier5Coefficients3D:
        if self.bezier_degree == TrajectoryBezierDegree.BEZIER5:
            return Bezier5Coefficients3D(p0, p3, t_out, t_in)
        return Bezier3Coefficients3D(p0, p3, t_out, t_in)

    @staticmethod
    def _distance_to_bezier_parameter(
        arc_lut_t: list[float],
        arc_lut_s: list[float],
        target_distance_mm: float,
    ) -> float:
        if not arc_lut_t or not arc_lut_s or len(arc_lut_t) != len(arc_lut_s):
            return 0.0
        if target_distance_mm <= 0.0:
            return 0.0
        total_length = float(arc_lut_s[-1])
        if total_length <= TrajectoryBuilder._EPS:
            return 0.0
        if target_distance_mm >= total_length:
            return 1.0

        idx = bisect.bisect_left(arc_lut_s, float(target_distance_mm))
        if idx <= 0:
            return float(arc_lut_t[0])
        if idx >= len(arc_lut_s):
            return float(arc_lut_t[-1])

        s0 = float(arc_lut_s[idx - 1])
        s1 = float(arc_lut_s[idx])
        t0 = float(arc_lut_t[idx - 1])
        t1 = float(arc_lut_t[idx])
        ds = s1 - s0
        if abs(ds) <= TrajectoryBuilder._EPS:
            return t0
        alpha = (float(target_distance_mm) - s0) / ds
        return t0 + (t1 - t0) * alpha

    @staticmethod
    def _harmonic_weighted_speed_mmps(descriptors: list[_BezierSegmentDescriptor]) -> float:
        total_distance = 0.0
        weighted_time = 0.0
        for descriptor in descriptors:
            distance = max(0.0, float(descriptor.arc_length_mm))
            if distance <= TrajectoryBuilder._EPS:
                continue
            speed = float(descriptor.speed_mmps)
            if speed <= TrajectoryBuilder._EPS:
                return 0.0
            total_distance += distance
            weighted_time += distance / speed
        if total_distance <= TrajectoryBuilder._EPS or weighted_time <= TrajectoryBuilder._EPS:
            return 0.0
        return total_distance / weighted_time

    @staticmethod
    def _are_vectors_collinear_opposite(
        a: list[float],
        b: list[float],
        angle_tol_deg: float,
    ) -> bool:
        norm_a = math_utils.vector_norm3(a)
        norm_b = math_utils.vector_norm3(b)
        if norm_a <= TrajectoryBuilder.SUPER_CHAIN_MIN_VECTOR_NORM:
            return False
        if norm_b <= TrajectoryBuilder.SUPER_CHAIN_MIN_VECTOR_NORM:
            return False
        na = [float(a[0]) / norm_a, float(a[1]) / norm_a, float(a[2]) / norm_a]
        nb = [float(b[0]) / norm_b, float(b[1]) / norm_b, float(b[2]) / norm_b]
        dot = max(-1.0, min(1.0, na[0] * nb[0] + na[1] * nb[1] + na[2] * nb[2]))
        cos_tol = math.cos(math.radians(max(0.0, float(angle_tol_deg))))
        return dot <= -cos_tol

    def _build_bezier_segment_descriptor(self, segment: TrajectorySegment) -> _BezierSegmentDescriptor | None:
        if not TrajectoryBuilder._is_lin_or_cubic_mode(segment.to_keypoint.mode):
            return None

        from_pose = self._resolve_keypoint_pose(segment.from_keypoint)
        to_pose = self._resolve_keypoint_pose(segment.to_keypoint)
        if from_pose is None or to_pose is None:
            return None

        p0 = [float(from_pose[0]), float(from_pose[1]), float(from_pose[2])]
        p3 = [float(to_pose[0]), float(to_pose[1]), float(to_pose[2])]
        segment_length_mm = math_utils.norm3(p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2])

        if segment.to_keypoint.mode == KeypointMotionMode.LINEAR:
            t_out, t_in = segment.to_keypoint.resolve_linear_tangent_vectors(p0, p3)
        else:
            t_out, t_in = segment.to_keypoint.resolve_cubic_tangent_vectors(segment_length_mm)

        coeffs = self._build_bezier_coefficients(p0, p3, t_out, t_in)
        arc_lut_t, arc_lut_s = TrajectoryBuilder._build_arc_length_lut(coeffs)
        arc_length_mm = float(arc_lut_s[-1]) if arc_lut_s else 0.0
        speed_mmps = TrajectoryBuilder.linear_speed_mps_to_mmps(segment.to_keypoint.linear_speed_mps)

        return _BezierSegmentDescriptor(
            segment=segment,
            from_pose=from_pose,
            to_pose=to_pose,
            coeffs=coeffs,
            t_out=[float(v) for v in t_out[:3]],
            t_in=[float(v) for v in t_in[:3]],
            arc_length_mm=arc_length_mm,
            speed_mmps=float(speed_mmps),
            arc_lut_t=arc_lut_t,
            arc_lut_s=arc_lut_s,
        )

    def _can_chain_bezier_descriptors(
        self,
        previous_descriptor: _BezierSegmentDescriptor,
        next_descriptor: _BezierSegmentDescriptor,
    ) -> bool:
        if previous_descriptor.arc_length_mm <= self._EPS or next_descriptor.arc_length_mm <= self._EPS:
            return False

        continuity_gap = math_utils.norm3(
            next_descriptor.from_pose[0] - previous_descriptor.to_pose[0],
            next_descriptor.from_pose[1] - previous_descriptor.to_pose[1],
            next_descriptor.from_pose[2] - previous_descriptor.to_pose[2],
        )
        if continuity_gap > 1e-3:
            return False

        return TrajectoryBuilder._are_vectors_collinear_opposite(
            previous_descriptor.t_in,
            next_descriptor.t_out,
            TrajectoryBuilder.SUPER_CHAIN_COLLINEAR_ANGLE_TOL_DEG,
        )

    def _collect_bezier_chain_descriptors(
        self,
        segments: list[TrajectorySegment],
        start_index: int,
    ) -> list[_BezierSegmentDescriptor]:
        if start_index < 0 or start_index >= len(segments):
            return []

        first_segment = segments[start_index]
        if not TrajectoryBuilder._is_lin_or_cubic_mode(first_segment.to_keypoint.mode):
            return []

        first_descriptor = self._build_bezier_segment_descriptor(first_segment)
        if first_descriptor is None:
            return []
        descriptors = [first_descriptor]

        idx = start_index + 1
        while idx < len(segments):
            candidate_segment = segments[idx]
            if not TrajectoryBuilder._is_lin_or_cubic_mode(candidate_segment.to_keypoint.mode):
                break
            candidate_descriptor = self._build_bezier_segment_descriptor(candidate_segment)
            if candidate_descriptor is None:
                break
            if not self._can_chain_bezier_descriptors(descriptors[-1], candidate_descriptor):
                break
            descriptors.append(candidate_descriptor)
            idx += 1

        return descriptors

    def _get_working_mgi_solver(self) -> MGI:
        if self._working_mgi_solver is None:
            self._working_mgi_solver = MGI(self.robot_model.mgi_params, self.tool_model.get_tool())
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
        accel_limits = TrajectoryBuilder._normalize_positive_limits(self.robot_model.get_axis_accel_limits())
        jerk_limits = TrajectoryBuilder._normalize_positive_limits(self.robot_model.get_axis_jerk_limits())
        return speed_limits, accel_limits, jerk_limits

    def _resolve_effective_sample_allowed_configs(self) -> set[MgiConfigKey]:
        return set(self._get_robot_allowed_configs())

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
            expanded_valid_for_config = mgi_result.get_solutions_expanded(config_key, only_valid=True)
            selected_solution = expanded_valid_for_config[0] if expanded_valid_for_config else solution
            status_name = (
                MgiResultStatus.VALID.name
                if expanded_valid_for_config
                else solution.status.name
            )
            if status_name == MgiResultStatus.VALID.name and config_key not in allowed_configs:
                status_name = MgiResultStatus.FORBIDDEN_CONFIGURATION.name
            compact[config_key] = TrajectorySampleMgiSolution(status=status_name, joints=selected_solution.joints)
        return compact

    def _select_best_solution(
        self,
        mgi_result: MgiResult,
        previous_sample: TrajectorySample | None,
        allowed_configs: set[MgiConfigKey],
    ) -> tuple[MgiConfigKey, MgiResultItem] | None:
        reference_joints_rad = [math.radians(v) for v in self._get_reference_joints_for_ik(previous_sample)]
        joint_weights = self._get_joint_weights()
        return mgi_result.get_best_solution_from_current(
            reference_joints_rad,
            joint_weights,
            allowed_configs,
        )

    def _apply_collision_detection_if_needed(
        self,
        sample: TrajectorySample,
        corrected_matrices: list[np.ndarray] | None,
    ) -> None:
        sample.collisions = []
        if sample.error_code != TrajectorySampleErrorCode.NONE:
            return
        if not sample.reachable:
            return
        if not corrected_matrices:
            return

        collision_cache = self._ensure_collision_world_cache()
        frame_world_transforms = build_world_frame_transforms(
            corrected_matrices,
            self.workspace_model.get_robot_base_transform_world(),
        )
        flange_world_transform = resolve_flange_world_transform(frame_world_transforms)
        collision_cache.update_dynamic_world_shapes(frame_world_transforms, flange_world_transform)

        diagnostics = TrajectoryBuilder._collision_diagnostics_from_pairs(
            collision_cache.find_workspace_collisions(),
            TrajectoryCollisionDomain.WORKSPACE,
        )
        diagnostics.extend(
            TrajectoryBuilder._collision_diagnostics_from_pairs(
                collision_cache.find_robot_tool_collisions(
                    self.tool_model.get_evaluated_robot_axis_colliders()
                ),
                TrajectoryCollisionDomain.ROBOT_TOOL,
            )
        )
        if not diagnostics:
            return

        sample.collisions = diagnostics
        sample.error_code = TrajectorySampleErrorCode.COLLISION_DETECTED
        sample.error_axis = None

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

    @staticmethod
    def _reset_cartesian_dynamics(sample: TrajectorySample) -> None:
        sample.cartesian_velocity = [0.0] * 6
        sample.cartesian_acceleration = [0.0] * 6
        sample.cartesian_jerk = [0.0] * 6
        sample.cartesian_velocity_valid = False
        sample.cartesian_acceleration_valid = False
        sample.cartesian_jerk_valid = False
        sample.velocity = 0.0
        sample.acceleration = 0.0

    @staticmethod
    def _reset_articular_dynamics(sample: TrajectorySample) -> None:
        sample.articular_velocity = [0.0] * 6
        sample.articular_acceleration = [0.0] * 6
        sample.articular_jerk = [0.0] * 6
        sample.articular_velocity_valid = False
        sample.articular_acceleration_valid = False
        sample.articular_jerk_valid = False

    @staticmethod
    def _update_cartesian_dynamics(
        sample: TrajectorySample,
        previous_sample: TrajectorySample | None,
        dt: float,
    ) -> None:
        TrajectoryBuilder._reset_cartesian_dynamics(sample)
        if previous_sample is None:
            return
        if not sample.reachable or not previous_sample.reachable:
            return

        dA = TrajectoryBuilder._shortest_angle_delta_deg(previous_sample.pose[3], sample.pose[3])
        dB = TrajectoryBuilder._shortest_angle_delta_deg(previous_sample.pose[4], sample.pose[4])
        dC = TrajectoryBuilder._shortest_angle_delta_deg(previous_sample.pose[5], sample.pose[5])

        sample.cartesian_velocity[0] = (sample.pose[0] - previous_sample.pose[0]) / dt
        sample.cartesian_velocity[1] = (sample.pose[1] - previous_sample.pose[1]) / dt
        sample.cartesian_velocity[2] = (sample.pose[2] - previous_sample.pose[2]) / dt
        sample.cartesian_velocity[3] = dA / dt
        sample.cartesian_velocity[4] = dB / dt
        sample.cartesian_velocity[5] = dC / dt
        sample.cartesian_velocity_valid = True
        sample.velocity = math_utils.norm3(
            sample.cartesian_velocity[0],
            sample.cartesian_velocity[1],
            sample.cartesian_velocity[2],
        )

        if not getattr(previous_sample, "cartesian_velocity_valid", False):
            return

        for axis in range(6):
            sample.cartesian_acceleration[axis] = (
                sample.cartesian_velocity[axis] - previous_sample.cartesian_velocity[axis]
            ) / dt
        sample.cartesian_acceleration_valid = True
        sample.acceleration = math_utils.norm3(
            sample.cartesian_acceleration[0],
            sample.cartesian_acceleration[1],
            sample.cartesian_acceleration[2],
        )

        if not getattr(previous_sample, "cartesian_acceleration_valid", False):
            return

        for axis in range(6):
            sample.cartesian_jerk[axis] = (
                sample.cartesian_acceleration[axis] - previous_sample.cartesian_acceleration[axis]
            ) / dt
        sample.cartesian_jerk_valid = True

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
        TrajectoryBuilder._reset_articular_dynamics(sample)
        if previous_sample is None:
            return
        if not sample.reachable or not previous_sample.reachable:
            return

        for axis in range(6):
            vel = (sample.joints[axis] - previous_sample.joints[axis]) / dt
            sample.articular_velocity[axis] = vel
        sample.articular_velocity_valid = True

        if not getattr(previous_sample, "articular_velocity_valid", False):
            return

        for axis in range(6):
            vel = sample.articular_velocity[axis]
            acc = (vel - previous_sample.articular_velocity[axis]) / dt
            sample.articular_acceleration[axis] = acc
        sample.articular_acceleration_valid = True

        if not getattr(previous_sample, "articular_acceleration_valid", False):
            return

        for axis in range(6):
            acc = sample.articular_acceleration[axis]
            sample.articular_jerk[axis] = (acc - previous_sample.articular_acceleration[axis]) / dt
        sample.articular_jerk_valid = True

    def _set_primary_dynamic_error(self, sample: TrajectorySample) -> None:
        if sample.error_code != TrajectorySampleErrorCode.NONE:
            return

        speed_errors = [
            violation
            for violation in sample.dynamic_violations
            if (
                violation.severity == TrajectoryDynamicViolationSeverity.ERROR
                and violation.kind == TrajectoryDynamicViolationKind.SPEED
            )
        ]
        if speed_errors:
            first = min(speed_errors, key=lambda violation: violation.axis)
            sample.error_code = TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED
            sample.error_axis = first.axis
            return

        jerk_errors = [
            violation
            for violation in sample.dynamic_violations
            if (
                violation.severity == TrajectoryDynamicViolationSeverity.ERROR
                and violation.kind == TrajectoryDynamicViolationKind.JERK
            )
        ]
        if jerk_errors:
            first = min(jerk_errors, key=lambda violation: violation.axis)
            sample.error_code = TrajectorySampleErrorCode.JERK_LIMIT_EXCEEDED
            sample.error_axis = first.axis

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

        eps = 1e-6
        for axis in range(6):
            vel = abs(float(sample.articular_velocity[axis]))
            if sample.articular_velocity_valid and vel > float(speed_limits_deg_s[axis]) + eps:
                sample.dynamic_violations.append(
                    TrajectoryDynamicViolation(
                        TrajectoryDynamicViolationKind.SPEED,
                        axis,
                        vel,
                        float(speed_limits_deg_s[axis]),
                        TrajectoryDynamicViolationSeverity.ERROR,
                    )
                )

            acc = abs(float(sample.articular_acceleration[axis]))
            accel_limit = float(accel_limits_deg_s2[axis])
            if sample.articular_acceleration_valid and accel_limit > self._EPS and acc > accel_limit + eps:
                sample.dynamic_violations.append(
                    TrajectoryDynamicViolation(
                        TrajectoryDynamicViolationKind.ACCELERATION,
                        axis,
                        acc,
                        accel_limit,
                        TrajectoryDynamicViolationSeverity.WARNING,
                    )
                )

            if self.jerk_check_enabled:
                jerk_limit = float(jerk_limits_deg_s3[axis])
                if jerk_limit <= self._EPS:
                    continue
                jerk = abs(float(sample.articular_jerk[axis]))
                if sample.articular_jerk_valid and jerk > jerk_limit + eps:
                    sample.dynamic_violations.append(
                        TrajectoryDynamicViolation(
                            TrajectoryDynamicViolationKind.JERK,
                            axis,
                            jerk,
                            jerk_limit,
                            TrajectoryDynamicViolationSeverity.ERROR,
                        )
                    )

        self._set_primary_dynamic_error(sample)

    def _apply_config_jump_detection_if_needed(
        self,
        sample: TrajectorySample,
        previous_sample: TrajectorySample | None,
        speed_limits_deg_s: list[float],
        accel_limits_deg_s2: list[float],
    ) -> None:
        """
        Detect likely branch/configuration jumps after IK selection.
        The detector uses very high dynamic thresholds to distinguish
        true discontinuities from normal dynamic-limit exceedances.
        """
        if sample.error_code != TrajectorySampleErrorCode.NONE:
            return
        if previous_sample is None:
            return
        if not sample.reachable or not previous_sample.reachable:
            return

        dt = sample.time - previous_sample.time
        if dt <= self._EPS:
            dt = self.sample_dt_s

        for axis in range(6):
            q_curr = float(sample.joints[axis])
            q_prev = float(previous_sample.joints[axis])
            delta_q = abs(q_curr - q_prev)

            speed_limit = max(float(speed_limits_deg_s[axis]), self._EPS)
            accel_limit = max(float(accel_limits_deg_s2[axis]), self._EPS)

            # First gate: discontinuity in joint position for the sample period.
            allowed_delta = max(
                TrajectoryBuilder.CONFIG_JUMP_MIN_DELTA_DEG,
                TrajectoryBuilder.CONFIG_JUMP_DELTA_FACTOR * speed_limit * dt,
            )
            if delta_q <= allowed_delta:
                continue

            vel = abs(float(sample.articular_velocity[axis])) if sample.articular_velocity_valid else 0.0
            acc = abs(float(sample.articular_acceleration[axis])) if sample.articular_acceleration_valid else 0.0

            # Second gate: dynamics consistent with a near-instant jump.
            speed_threshold = max(
                TrajectoryBuilder.CONFIG_JUMP_MIN_SPEED_DEG_S,
                TrajectoryBuilder.CONFIG_JUMP_SPEED_FACTOR * speed_limit,
            )
            accel_threshold = max(
                TrajectoryBuilder.CONFIG_JUMP_ACCEL_THRESHOLD_DEG_S2,
                TrajectoryBuilder.CONFIG_JUMP_SPEED_FACTOR * accel_limit,
            )

            if vel >= speed_threshold or acc >= accel_threshold:
                sample.error_code = TrajectorySampleErrorCode.CONFIGURATION_JUMP
                sample.error_axis = axis
                return

    @staticmethod
    def _resolve_error_for_missing_selected_solution(
        mgi_result: MgiResult,
        allowed_configs: set[MgiConfigKey],
    ) -> TrajectorySampleErrorCode:
        has_any_valid = False
        has_allowed_valid = False

        expanded_valid = mgi_result.get_valid_solutions_expanded()
        if expanded_valid:
            for solution in expanded_valid:
                config_key = solution.config_key
                if config_key is None:
                    continue
                has_any_valid = True
                if config_key in allowed_configs:
                    has_allowed_valid = True
                    break
        else:
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
        sample.dynamic_violations = []
        sample.collisions = []

        fk_result = self.robot_model.compute_fk_joints(sample.joints, tool=self.tool_model.get_tool())
        if fk_result is not None:
            _, corrected_matrices, _, _, _ = fk_result
            self._apply_collision_detection_if_needed(sample, corrected_matrices)
        else:
            sample.reachable = False
            sample.error_code = TrajectorySampleErrorCode.POINT_UNREACHABLE
            sample.configuration = None

        dt = self.sample_dt_s
        if previous_sample is not None:
            dt = sample.time - previous_sample.time
            if dt <= self._EPS:
                dt = self.sample_dt_s
        TrajectoryBuilder._update_cartesian_dynamics(sample, previous_sample, dt)
        self._update_articular_dynamics(sample, previous_sample, dt)
        speed_limits, accel_limits, jerk_limits = self._get_axis_dynamic_limits()
        self._apply_config_jump_detection_if_needed(sample, previous_sample, speed_limits, accel_limits)
        self._apply_dynamic_limits_if_needed(sample, previous_sample, speed_limits, accel_limits, jerk_limits)
        return True

    def compute_trajectory(
        self,
        current_joints: list[float],
        segments: list[TrajectorySegment],
    ) -> TrajectoryResult:
        trajectory = TrajectoryResult()
        if not segments:
            return trajectory

        self._working_mgi_solver = MGI(self.robot_model.mgi_params, self.tool_model.get_tool())
        self._robot_allowed_configs = set(self.robot_model.get_allowed_configurations())
        self._joint_weights = [float(v) for v in self.robot_model.get_joint_weights()[:6]]
        self._ensure_collision_world_cache()
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

            segment_idx = 0
            while segment_idx < len(segments):
                segment = segments[segment_idx]
                if TrajectoryBuilder._is_lin_or_cubic_mode(segment.to_keypoint.mode):
                    chain_descriptors = self._collect_bezier_chain_descriptors(segments, segment_idx)
                    if len(chain_descriptors) >= 2:
                        chain_results = self._generate_bezier_super_segments(
                            chain_descriptors,
                            previous_sample,
                            start_time_s,
                        )
                        if chain_results:
                            for local_offset, chain_result in enumerate(chain_results):
                                trajectory.segments.append(chain_result)
                                TrajectoryBuilder._accumulate_status(
                                    trajectory,
                                    chain_result,
                                    segment_index=1 + segment_idx + local_offset,
                                )
                                if self._should_stop_on_error(chain_result.status):
                                    return trajectory

                            previous_sample = TrajectoryBuilder._extract_previous_sample(chain_results[-1])
                            start_time_s = chain_results[-1].last_time
                            segment_idx += len(chain_results)
                            continue

                segment_result = self.compute_segment(segment, previous_sample, start_time_s)
                trajectory.segments.append(segment_result)
                TrajectoryBuilder._accumulate_status(
                    trajectory,
                    segment_result,
                    segment_index=1 + segment_idx,
                )
                if self._should_stop_on_error(segment_result.status):
                    break

                previous_sample = TrajectoryBuilder._extract_previous_sample(segment_result)
                start_time_s = segment_result.last_time
                segment_idx += 1
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
        self._ensure_collision_world_cache()
        joints_6 = TrajectoryBuilder._normalize_joints_6(current_joints)
        config_identifier = self.robot_model.get_config_identifier()
        identified_config = MgiConfigKey.identify_configuration_deg(joints_6, config_identifier)

        synthetic_from = TrajectoryKeypoint(
            target_type=KeypointTargetType.JOINT,
            joint_target=joints_6,
            mode=to_keypoint.mode,
            cubic_vectors=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            configuration_policy=ConfigurationPolicy.FORCED,
            forced_config=identified_config,
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

        # 1) Resolve endpoint policy for start keypoint (strict, no fallback).
        from_allowed_configs = self._resolve_allowed_configs_for_keypoint(segment.from_keypoint, previous_sample)
        if not from_allowed_configs:
            result.status = TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION
            result.last_time = float(start_time_s)
            return result

        # 2) Resolve segment endpoints in joint space.
        config_identifier = self.robot_model.get_config_identifier()

        from_joints, to_joints, endpoint_error = self._resolve_PTP_segment_endpoints(
            segment,
            previous_sample,
            from_allowed_configs,
            config_identifier,
        )
        if from_joints is None or to_joints is None:
            result.status = TrajectoryBuilder._status_from_sample_error(endpoint_error)
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
            result.status = TrajectoryComputationStatus.SPEED_LIMIT_EXCEEDED
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
            self._resolve_effective_sample_allowed_configs(),
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

        from_allowed_configs = self._resolve_allowed_configs_for_keypoint(segment.from_keypoint, previous_segment_last_sample)
        if not from_allowed_configs:
            result = TrajectoryBuilder._new_empty_segment(start_time_s, segment.to_keypoint.mode)
            result.status = TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION
            return result

        if previous_segment_last_sample is not None and previous_segment_last_sample.reachable:
            from_config = previous_segment_last_sample.configuration
            if from_config is not None and from_config not in from_allowed_configs:
                result = TrajectoryBuilder._new_empty_segment(start_time_s, segment.to_keypoint.mode)
                result.status = TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION
                return result

        effective_allowed_configs = self._resolve_effective_sample_allowed_configs()
        if not effective_allowed_configs:
            result = TrajectoryBuilder._new_empty_segment(start_time_s, segment.to_keypoint.mode)
            result.status = TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION
            return result

        p0 = [from_pose[0], from_pose[1], from_pose[2]]
        p3 = [to_pose[0], to_pose[1], to_pose[2]]
        segment_length_mm = math_utils.norm3(p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2])

        if force_linear_handles:
            t_out, t_in = segment.to_keypoint.resolve_linear_tangent_vectors(p0, p3)
        else:
            # Cubic handles are carried by the destination keypoint (segment-in semantics):
            # [0] = start-side direction/amplitude, [1] = end-side direction/amplitude.
            t_out, t_in = segment.to_keypoint.resolve_cubic_tangent_vectors(segment_length_mm)

        coeffs = self._build_bezier_coefficients(p0, p3, t_out, t_in)
        arc_length_mm = TrajectoryBuilder._estimate_arc_length(coeffs)
        speed_mmps = TrajectoryBuilder.linear_speed_mps_to_mmps(segment.to_keypoint.linear_speed_mps)
        intervals = TrajectoryBuilder._resolve_num_intervals(
            arc_length_mm,
            speed_mmps,
            self.sample_dt_s,
            self._time_profile_peak_speed_scale(),
        )

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
        to_allowed_configs: set[MgiConfigKey] | None = None
        for i in range(1, intervals + 1):
            time_s = start_time_s + i * self.sample_dt_s
            linear_t = i / intervals
            smooth_t = math_utils.quintic_transition(linear_t) if self.smooth_time_enabled else linear_t
            smooth_t5 = math_utils.quintic_transition(linear_t)
            xyz = coeffs.point(smooth_t)
            orientation_abc = [
                TrajectoryBuilder._wrap_angle_deg(from_pose[3] + dA * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(from_pose[4] + dB * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(from_pose[5] + dC * smooth_t5),
            ]

            sample_allowed_configs = effective_allowed_configs
            if i == intervals:
                to_allowed_configs = self._resolve_allowed_configs_for_keypoint(segment.to_keypoint, previous_sample)
                sample_allowed_configs = to_allowed_configs

            sample = self._build_sample_from_cartesian(
                time_s,
                xyz,
                orientation_abc,
                previous_sample,
                sample_allowed_configs,
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

        if to_allowed_configs is not None and not to_allowed_configs and result.status == TrajectoryComputationStatus.SUCCESS:
            result.status = TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION

        generated_intervals = len(result.samples)
        result.duration = generated_intervals * self.sample_dt_s
        result.last_time = start_time_s + result.duration
        return result

    def _build_super_chain_step_assignment(
        self,
        descriptors: list[_BezierSegmentDescriptor],
        intervals: int,
        total_length_mm: float,
    ) -> tuple[list[float], list[int], list[float], list[int], list[int], list[int]]:
        cumulative_lengths = [0.0]
        for descriptor in descriptors:
            cumulative_lengths.append(cumulative_lengths[-1] + max(0.0, float(descriptor.arc_length_mm)))

        n_segments = len(descriptors)
        segment_by_step = [0] * intervals
        distance_by_step = [0.0] * intervals
        first_step_by_segment = [0] * n_segments
        last_step_by_segment = [0] * n_segments
        counts_by_segment = [0] * n_segments

        active_segment_idx = 0
        for step in range(1, intervals + 1):
            linear_u = step / intervals
            smooth_u = math_utils.quintic_transition(linear_u) if self.smooth_time_enabled else linear_u
            distance_mm = float(smooth_u) * total_length_mm

            while (
                active_segment_idx < (n_segments - 1)
                and distance_mm > (cumulative_lengths[active_segment_idx + 1] - self._EPS)
            ):
                active_segment_idx += 1

            segment_by_step[step - 1] = active_segment_idx
            distance_by_step[step - 1] = distance_mm
            counts_by_segment[active_segment_idx] += 1
            if first_step_by_segment[active_segment_idx] == 0:
                first_step_by_segment[active_segment_idx] = step
            last_step_by_segment[active_segment_idx] = step

        return (
            cumulative_lengths,
            segment_by_step,
            distance_by_step,
            first_step_by_segment,
            last_step_by_segment,
            counts_by_segment,
        )

    def _resolve_super_chain_intervals_and_assignment(
        self,
        descriptors: list[_BezierSegmentDescriptor],
        total_length_mm: float,
        speed_mmps: float,
    ) -> tuple[int, list[float], list[int], list[float], list[int], list[int], list[int]]:
        max_intervals = max(1, TrajectoryBuilder.MAX_SAMPLES_PER_SEGMENT * max(1, len(descriptors)))
        if total_length_mm <= self._EPS or speed_mmps <= self._EPS:
            intervals = 1
        else:
            theoretical_duration_s = self._time_profile_peak_speed_scale() * total_length_mm / speed_mmps
            intervals = int(math.ceil(theoretical_duration_s / self.sample_dt_s))
            intervals = min(max(1, intervals), max_intervals)

        while True:
            (
                cumulative_lengths,
                segment_by_step,
                distance_by_step,
                first_step_by_segment,
                last_step_by_segment,
                counts_by_segment,
            ) = self._build_super_chain_step_assignment(
                descriptors,
                intervals,
                total_length_mm,
            )
            if all(count > 0 for count in counts_by_segment):
                return (
                    intervals,
                    cumulative_lengths,
                    segment_by_step,
                    distance_by_step,
                    first_step_by_segment,
                    last_step_by_segment,
                    counts_by_segment,
                )
            if intervals >= max_intervals:
                return (
                    intervals,
                    cumulative_lengths,
                    segment_by_step,
                    distance_by_step,
                    first_step_by_segment,
                    last_step_by_segment,
                    counts_by_segment,
                )

            missing_count = sum(1 for count in counts_by_segment if count <= 0)
            intervals = min(max_intervals, intervals + missing_count)

    def _generate_bezier_super_segments(
        self,
        descriptors: list[_BezierSegmentDescriptor],
        previous_segment_last_sample: TrajectorySample | None,
        start_time_s: float,
    ) -> list[SegmentResult]:
        if len(descriptors) < 2:
            return []

        first_segment = descriptors[0].segment
        from_allowed_configs = self._resolve_allowed_configs_for_keypoint(
            first_segment.from_keypoint,
            previous_segment_last_sample,
        )
        if not from_allowed_configs:
            return []
        if previous_segment_last_sample is not None and previous_segment_last_sample.reachable:
            from_config = previous_segment_last_sample.configuration
            if from_config is not None and from_config not in from_allowed_configs:
                return []

        effective_allowed_configs = self._resolve_effective_sample_allowed_configs()
        if not effective_allowed_configs:
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
            last_step_by_segment,
            _counts_by_segment,
        ) = self._resolve_super_chain_intervals_and_assignment(
            descriptors,
            total_length_mm,
            group_speed_mmps,
        )

        results: list[SegmentResult] = []
        for descriptor in descriptors:
            segment_result = SegmentResult()
            segment_result.status = TrajectoryComputationStatus.SUCCESS
            segment_result.mode = descriptor.segment.to_keypoint.mode
            segment_result.out_direction = list(descriptor.t_out)
            segment_result.in_direction = list(descriptor.t_in)
            segment_result.last_time = float(start_time_s)
            results.append(segment_result)

        speed_limits_deg_s, accel_limits_deg_s2, jerk_limits_deg_s3 = self._get_axis_dynamic_limits()
        orientation_deltas = [
            [
                TrajectoryBuilder._shortest_angle_delta_deg(descriptor.from_pose[3], descriptor.to_pose[3]),
                TrajectoryBuilder._shortest_angle_delta_deg(descriptor.from_pose[4], descriptor.to_pose[4]),
                TrajectoryBuilder._shortest_angle_delta_deg(descriptor.from_pose[5], descriptor.to_pose[5]),
            ]
            for descriptor in descriptors
        ]
        to_allowed_configs_by_segment: list[set[MgiConfigKey] | None] = [None] * len(descriptors)

        previous_sample = previous_segment_last_sample
        stop_segment_idx: int | None = None

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
            orientation_abc = [
                TrajectoryBuilder._wrap_angle_deg(descriptor.from_pose[3] + dA * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(descriptor.from_pose[4] + dB * smooth_t5),
                TrajectoryBuilder._wrap_angle_deg(descriptor.from_pose[5] + dC * smooth_t5),
            ]

            sample_allowed_configs = effective_allowed_configs
            if step == last_step_by_segment[segment_idx]:
                to_allowed_configs = self._resolve_allowed_configs_for_keypoint(
                    descriptor.segment.to_keypoint,
                    previous_sample,
                )
                to_allowed_configs_by_segment[segment_idx] = to_allowed_configs
                sample_allowed_configs = to_allowed_configs

            sample = self._build_sample_from_cartesian(
                time_s,
                xyz,
                orientation_abc,
                previous_sample,
                sample_allowed_configs,
                speed_limits_deg_s,
                accel_limits_deg_s2,
                jerk_limits_deg_s3,
            )
            segment_result.samples.append(sample)
            TrajectoryBuilder._update_joint_stats(segment_result, sample)
            self._register_sample_error(segment_result, sample, sample_index=len(segment_result.samples) - 1)
            previous_sample = sample

            if self._should_stop_on_error(segment_result.status):
                stop_segment_idx = segment_idx
                break

        if stop_segment_idx is not None:
            results = results[: stop_segment_idx + 1]

        rolling_time = float(start_time_s)
        for segment_idx, segment_result in enumerate(results):
            to_allowed_configs = (
                to_allowed_configs_by_segment[segment_idx]
                if segment_idx < len(to_allowed_configs_by_segment)
                else None
            )
            if (
                to_allowed_configs is not None
                and not to_allowed_configs
                and segment_result.status == TrajectoryComputationStatus.SUCCESS
            ):
                segment_result.status = TrajectoryComputationStatus.FORBIDDEN_CONFIGURATION

            generated_intervals = len(segment_result.samples)
            segment_result.duration = generated_intervals * self.sample_dt_s
            if generated_intervals > 0:
                segment_result.last_time = float(segment_result.samples[-1].time)
                rolling_time = segment_result.last_time
            else:
                segment_result.last_time = rolling_time

        return results

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
        corrected_matrices: list[np.ndarray] | None = None
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
            fk_result = self.robot_model.compute_fk_joints(sample.joints, tool=self.tool_model.get_tool())
            if fk_result is None:
                sample.reachable = False
                sample.error_code = TrajectorySampleErrorCode.POINT_UNREACHABLE
                sample.configuration = None
                sample.joints = [0.0] * 6
            else:
                _, corrected_matrices, _, _, _ = fk_result

        self._apply_collision_detection_if_needed(sample, corrected_matrices)

        if previous_sample is None:
            TrajectoryBuilder._reset_cartesian_dynamics(sample)
            TrajectoryBuilder._reset_articular_dynamics(sample)
            return sample

        dt = sample.time - previous_sample.time
        if dt <= self._EPS:
            dt = self.sample_dt_s

        TrajectoryBuilder._update_cartesian_dynamics(sample, previous_sample, dt)
        self._update_articular_dynamics(sample, previous_sample, dt)
        self._apply_config_jump_detection_if_needed(
            sample,
            previous_sample,
            speed_limits_deg_s,
            accel_limits_deg_s2,
        )
        self._apply_dynamic_limits_if_needed(
            sample,
            previous_sample,
            speed_limits_deg_s,
            accel_limits_deg_s2,
            jerk_limits_deg_s3,
        )
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
        corrected_matrices: list[np.ndarray] | None = None

        # B) Compute pose from FK and expose configuration reachability metadata.
        fk_result = self.robot_model.compute_fk_joints(sample.joints, tool=self.tool_model.get_tool())
        if fk_result is None:
            sample.reachable = False
            sample.error_code = TrajectorySampleErrorCode.POINT_UNREACHABLE
            sample.configuration = None
            sample.pose = [0.0] * 6
            sample.mgi_solutions = {}
        else:
            _, corrected_matrices, dh_pose, _, _ = fk_result
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

        self._apply_collision_detection_if_needed(sample, corrected_matrices)

        # C) Compute cartesian/articular dynamics from previous sample.
        if previous_sample is None:
            TrajectoryBuilder._reset_cartesian_dynamics(sample)
            TrajectoryBuilder._reset_articular_dynamics(sample)
            return sample

        dt = sample.time - previous_sample.time
        if dt <= self._EPS:
            dt = self.sample_dt_s

        TrajectoryBuilder._update_cartesian_dynamics(sample, previous_sample, dt)
        self._update_articular_dynamics(sample, previous_sample, dt)
        self._apply_config_jump_detection_if_needed(
            sample,
            previous_sample,
            speed_limits_deg_s,
            accel_limits_deg_s2,
        )
        self._apply_dynamic_limits_if_needed(
            sample,
            previous_sample,
            speed_limits_deg_s,
            accel_limits_deg_s2,
            jerk_limits_deg_s3,
        )

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
