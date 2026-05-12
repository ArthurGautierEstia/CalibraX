from __future__ import annotations

import math

from models.reference_frame import ReferenceFrame
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.trajectory_keypoint import ConfigurationPolicy, KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.types import JointAngles6, Pose6, XYZ3
from models.workspace_model import WorkspaceModel
from trajectory_engine.models import BuildCancelToken, TrajectoryBuilderBehavior, TrajectorySegment
from trajectory_engine.v2.arc_length import build_arc_length_lut
from trajectory_engine.v2.dynamics import build_distance_profile, ptp_duration_s
from trajectory_engine.v2.geometry import Bezier7Curve3D
from trajectory_engine.v2.models import DynamicLimits, RuntimeSegment, SegmentSpeedProfile, TrajectoryPassMode
from utils.mgi import MGI, ConfigurationIdentifier, MgiConfigKey, MgiResult, MgiResultItem
from utils.reference_frame_utils import convert_pose_to_base_frame


class BuilderV2Common:
    DEFAULT_SAMPLE_DT_S = 0.004
    DEFAULT_ARC_LENGTH_SAMPLES = 300
    MAX_SAMPLES_PER_SEGMENT = 50_000
    _EPS = 1e-9

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        sample_dt_s: float = DEFAULT_SAMPLE_DT_S,
        cartesian_accel_limit_mm_s2: float = 1000.0,
        cartesian_jerk_limit_mm_s3: float = 10000.0,
        jerk_check_enabled: bool = True,
        behavior: TrajectoryBuilderBehavior = TrajectoryBuilderBehavior.CONTINUE_ON_ERROR,
    ) -> None:
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.sample_dt_s = sample_dt_s if sample_dt_s > 0.0 else self.DEFAULT_SAMPLE_DT_S
        self.cartesian_accel_limit_mm_s2 = max(1e-6, float(cartesian_accel_limit_mm_s2))
        self.cartesian_jerk_limit_mm_s3 = max(1e-6, float(cartesian_jerk_limit_mm_s3))
        self.jerk_check_enabled = bool(jerk_check_enabled)
        self.behavior = behavior
        self._cancel_token: BuildCancelToken | None = None
        self._working_mgi_solver: MGI | None = None
        self._robot_allowed_configs: set[MgiConfigKey] | None = None
        self._joint_weights: list[float] | None = None

    def set_cancel_token(self, cancel_token: BuildCancelToken | None) -> None:
        self._cancel_token = cancel_token

    def set_cartesian_dynamic_limits(self, accel_limit_mm_s2: float, jerk_limit_mm_s3: float) -> None:
        self.cartesian_accel_limit_mm_s2 = max(1e-6, float(accel_limit_mm_s2))
        self.cartesian_jerk_limit_mm_s3 = max(1e-6, float(jerk_limit_mm_s3))

    def set_jerk_check_enabled(self, enabled: bool) -> None:
        self.jerk_check_enabled = bool(enabled)

    def set_behavior(self, behavior: TrajectoryBuilderBehavior) -> None:
        self.behavior = behavior

    def _is_cancelled(self) -> bool:
        return self._cancel_token is not None and self._cancel_token.is_cancelled()

    @staticmethod
    def linear_speed_mps_to_mmps(speed_mps: float) -> float:
        return max(0.0, float(speed_mps) * 1000.0)

    @staticmethod
    def _copy_joints_6(values: list[float]) -> list[float]:
        out = [float(v) for v in values[:6]]
        while len(out) < 6:
            out.append(0.0)
        return out

    @staticmethod
    def _pose_xyz(pose: Pose6) -> XYZ3:
        return XYZ3(pose.x, pose.y, pose.z)

    @staticmethod
    def _pose_from_values(values: list[float]) -> Pose6:
        return Pose6.from_values(values)

    @staticmethod
    def _shortest_angle_delta_deg(from_deg: float, to_deg: float) -> float:
        delta = (float(to_deg) - float(from_deg) + 180.0) % 360.0 - 180.0
        if delta == -180.0 and (float(to_deg) - float(from_deg)) > 0.0:
            return 180.0
        return delta

    @staticmethod
    def _wrap_angle_deg(angle_deg: float) -> float:
        wrapped = (float(angle_deg) + 180.0) % 360.0 - 180.0
        if wrapped == -180.0 and angle_deg > 0.0:
            return 180.0
        return wrapped

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

    def _get_working_mgi_solver(self) -> MGI:
        if self._working_mgi_solver is None:
            self._working_mgi_solver = MGI(self.robot_model.mgi_params, self.tool_model.get_tool())
        return self._working_mgi_solver

    def _resolve_keypoint_pose(self, keypoint: TrajectoryKeypoint) -> Pose6 | None:
        if keypoint.target_type == KeypointTargetType.CARTESIAN:
            return convert_pose_to_base_frame(
                keypoint.cartesian_target,
                ReferenceFrame.from_value(keypoint.cartesian_frame),
                self.workspace_model.get_robot_base_transform_world(),
            )
        fk_result = self.robot_model.compute_fk_joints(keypoint.joint_target[:6], tool=self.tool_model.get_tool())
        if fk_result is None:
            return None
        return fk_result.dh_pose.copy()

    def _get_reference_joints_for_ik(self, previous_joints_deg: JointAngles6 | None) -> list[float]:
        if previous_joints_deg is not None:
            return previous_joints_deg.to_list()
        return self._copy_joints_6(self.robot_model.get_joints())

    def _compute_mgi_for_pose(self, pose: Pose6, previous_joints_deg: JointAngles6 | None) -> MgiResult:
        reference_joints = self._get_reference_joints_for_ik(previous_joints_deg)
        solver = self._get_working_mgi_solver()
        solver.set_q1ValueIfSingularityQ1Deg(reference_joints[0])
        solver.set_q4ValueIfSingularityQ5Deg(reference_joints[3])
        solver.set_q6ValueIfSingularityQ5Deg(reference_joints[5])
        return solver.compute_mgi_target(pose.to_list(), returnDegrees=True)

    def _resolve_reference_config(self, previous_joints_deg: JointAngles6 | None) -> MgiConfigKey:
        return MgiConfigKey.identify_configuration_deg(
            self._get_reference_joints_for_ik(previous_joints_deg),
            self.robot_model.get_config_identifier(),
        )

    def _resolve_allowed_configs_for_keypoint(
        self,
        keypoint: TrajectoryKeypoint,
        previous_joints_deg: JointAngles6 | None,
    ) -> set[MgiConfigKey]:
        robot_allowed = self._get_robot_allowed_configs()
        if keypoint.target_type == KeypointTargetType.JOINT:
            config_key = MgiConfigKey.identify_configuration_deg(
                self._copy_joints_6(keypoint.joint_target),
                self.robot_model.get_config_identifier(),
            )
            return {config_key} & robot_allowed
        if keypoint.configuration_policy == ConfigurationPolicy.AUTO:
            return set(robot_allowed)
        if keypoint.configuration_policy == ConfigurationPolicy.CURRENT_BRANCH:
            return {self._resolve_reference_config(previous_joints_deg)} & robot_allowed
        if keypoint.configuration_policy == ConfigurationPolicy.FORCED and keypoint.forced_config is not None:
            return {keypoint.forced_config} & robot_allowed
        return set()

    def _select_best_solution(
        self,
        mgi_result: MgiResult,
        previous_joints_deg: JointAngles6 | None,
        allowed_configs: set[MgiConfigKey],
    ) -> tuple[MgiConfigKey, MgiResultItem] | None:
        reference_joints_rad = [math.radians(v) for v in self._get_reference_joints_for_ik(previous_joints_deg)]
        return mgi_result.get_best_solution_from_current(reference_joints_rad, self._get_joint_weights(), allowed_configs)

    def _resolve_keypoint_joints(
        self,
        keypoint: TrajectoryKeypoint,
        previous_joints_deg: JointAngles6 | None,
    ) -> JointAngles6 | None:
        if keypoint.target_type == KeypointTargetType.JOINT:
            joints = JointAngles6.from_values(keypoint.joint_target)
            allowed = self._resolve_allowed_configs_for_keypoint(keypoint, previous_joints_deg)
            config_key = MgiConfigKey.identify_configuration_deg(joints.to_list(), self.robot_model.get_config_identifier())
            return joints if config_key in allowed else None
        pose = self._resolve_keypoint_pose(keypoint)
        if pose is None:
            return None
        allowed_configs = self._resolve_allowed_configs_for_keypoint(keypoint, previous_joints_deg)
        selected = self._select_best_solution(self._compute_mgi_for_pose(pose, previous_joints_deg), previous_joints_deg, allowed_configs)
        if selected is None:
            return None
        return JointAngles6.from_values(selected[1].joints)

    @staticmethod
    def _is_cartesian_mode(mode: KeypointMotionMode) -> bool:
        return mode in (KeypointMotionMode.LINEAR, KeypointMotionMode.CUBIC)

    def _build_curve(self, segment: TrajectorySegment, previous_curve: Bezier7Curve3D | None) -> tuple[Bezier7Curve3D, XYZ3, XYZ3] | None:
        start_pose = self._resolve_keypoint_pose(segment.from_keypoint)
        end_pose = self._resolve_keypoint_pose(segment.to_keypoint)
        if start_pose is None or end_pose is None:
            return None
        start = self._pose_xyz(start_pose)
        end = self._pose_xyz(end_pose)

        if segment.to_keypoint.mode == KeypointMotionMode.LINEAR:
            curve = Bezier7Curve3D.linear(start, end)
            direction = XYZ3((end.x - start.x) / 7.0, (end.y - start.y) / 7.0, (end.z - start.z) / 7.0)
            return curve, direction, XYZ3(-direction.x, -direction.y, -direction.z)

        if previous_curve is not None and TrajectoryPassMode.from_value(getattr(segment.from_keypoint, "pass_mode", TrajectoryPassMode.STOP)) == TrajectoryPassMode.FLY_BY:
            curve = Bezier7Curve3D.c3_continuation(previous_curve, end)
            out_direction = XYZ3(
                curve.control_points.p1.x - curve.control_points.p0.x,
                curve.control_points.p1.y - curve.control_points.p0.y,
                curve.control_points.p1.z - curve.control_points.p0.z,
            )
            in_direction = XYZ3(
                curve.control_points.p6.x - curve.control_points.p7.x,
                curve.control_points.p6.y - curve.control_points.p7.y,
                curve.control_points.p6.z - curve.control_points.p7.z,
            )
            return curve, out_direction, in_direction

        segment_length = ((end.x - start.x) ** 2 + (end.y - start.y) ** 2 + (end.z - start.z) ** 2) ** 0.5
        out_direction, in_direction = segment.to_keypoint.resolve_cubic_tangent_vectors(segment_length)
        return Bezier7Curve3D.from_handles(start, end, out_direction, in_direction), out_direction, in_direction

    def _segment_exit_speed(self, segments: list[TrajectorySegment], index: int) -> float:
        current = segments[index]
        if TrajectoryPassMode.from_value(getattr(current.to_keypoint, "pass_mode", TrajectoryPassMode.STOP)) != TrajectoryPassMode.FLY_BY:
            return 0.0
        if index + 1 >= len(segments):
            return 0.0
        next_segment = segments[index + 1]
        if not self._is_cartesian_mode(current.to_keypoint.mode) or not self._is_cartesian_mode(next_segment.to_keypoint.mode):
            return 0.0
        return min(
            self.linear_speed_mps_to_mmps(current.to_keypoint.linear_speed_mps),
            self.linear_speed_mps_to_mmps(next_segment.to_keypoint.linear_speed_mps),
        )

    def _build_runtime_segment(
        self,
        segment: TrajectorySegment,
        segment_index: int,
        previous_curve: Bezier7Curve3D | None,
        entry_speed_mm_s: float,
        exit_speed_mm_s: float,
    ) -> RuntimeSegment | None:
        start_pose = self._resolve_keypoint_pose(segment.from_keypoint)
        end_pose = self._resolve_keypoint_pose(segment.to_keypoint)
        curve_info = self._build_curve(segment, previous_curve)
        if start_pose is None or end_pose is None or curve_info is None:
            return None
        curve, out_direction, in_direction = curve_info
        arc_lut = build_arc_length_lut(curve, self.DEFAULT_ARC_LENGTH_SAMPLES, self._cancel_token)
        target_speed = self.linear_speed_mps_to_mmps(segment.to_keypoint.linear_speed_mps)
        return RuntimeSegment(
            mode=segment.to_keypoint.mode,
            curve=curve,
            arc_lut=arc_lut,
            start_pose=start_pose,
            end_pose=end_pose,
            speed_profile=SegmentSpeedProfile(
                segment_index=segment_index,
                length_mm=arc_lut.total_length_mm,
                target_speed_mm_s=target_speed,
                entry_speed_mm_s=entry_speed_mm_s,
                exit_speed_mm_s=exit_speed_mm_s,
            ),
            out_direction=out_direction,
            in_direction=in_direction,
        )

    def _ptp_duration(self, segment: TrajectorySegment, deltas: JointAngles6) -> float:
        speed_ratio = max(0.0, min(1.0, float(segment.to_keypoint.ptp_speed_percent) / 100.0))
        axis_speed_limits = self.robot_model.get_axis_speed_limits()
        delta_values = deltas.to_list()
        duration = 0.0
        for axis in range(6):
            limit = float(axis_speed_limits[axis]) * speed_ratio if axis < len(axis_speed_limits) else 0.0
            if abs(delta_values[axis]) > self._EPS and limit <= self._EPS:
                return 0.0
            duration = max(duration, ptp_duration_s(abs(delta_values[axis]), limit))
        return duration

    def _dynamic_limits(self, segment: TrajectorySegment) -> DynamicLimits:
        return DynamicLimits(
            cartesian_speed_mm_s=self.linear_speed_mps_to_mmps(segment.to_keypoint.linear_speed_mps),
            cartesian_accel_mm_s2=self.cartesian_accel_limit_mm_s2,
            cartesian_jerk_mm_s3=self.cartesian_jerk_limit_mm_s3,
        )
