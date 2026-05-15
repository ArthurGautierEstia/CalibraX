import json
from pathlib import Path
import unittest

from models.robot_configuration_file import RobotConfigurationFile
from models.robot_model import RobotModel
from models.tool_config_file import ToolConfigFile
from models.tool_model import ToolModel
from models.trajectory_keypoint import TrajectoryKeypoint
from models.types import JointAngles6, XYZ3
from models.workspace_file import WorkspaceFile
from models.workspace_model import WorkspaceModel
from trajectory_engine.arc_length import build_arc_length_lut, parameter_at_distance
from trajectory_engine.models.pipeline import (
    SegmentResult,
    TrajectoryBuilderBehavior,
    TrajectoryComputationStatus,
    TrajectoryResult,
    TrajectorySample,
    TrajectorySegment,
)
from trajectory_engine.core.full_builder import TrajectoryBuilder
from trajectory_engine.dynamics import (
    S_CURVE_PEAK_JERK_SCALE,
    S_CURVE_PEAK_SPEED_SCALE,
    SegmentDynamicProfileKind,
    build_distance_profile,
    ptp_jerk_duration_s,
    ptp_duration_s,
    resolve_segment_dynamic_profile,
)
from trajectory_engine.geometry import Bezier7Curve3D
from trajectory_engine.models.trajectory_primitives import SegmentDynamicPhaseKind


def _assert_xyz_close(test: unittest.TestCase, actual: XYZ3, expected: XYZ3, places: int = 6) -> None:
    test.assertAlmostEqual(actual.x, expected.x, places=places)
    test.assertAlmostEqual(actual.y, expected.y, places=places)
    test.assertAlmostEqual(actual.z, expected.z, places=places)


class _CancelToken:
    def __init__(self) -> None:
        self.calls = 0

    def is_cancelled(self) -> bool:
        self.calls += 1
        return self.calls > 3


class _PtpDurationRobotModel:
    def get_axis_speed_limits(self) -> list[float]:
        return [1_000_000.0] * 6

    def get_axis_jerk_limits(self) -> list[float]:
        return [420.0] * 6


class TrajectoryEngineTests(unittest.TestCase):
    @staticmethod
    def _project_root() -> Path:
        return Path(__file__).resolve().parents[1]

    def _compute_fixture_trajectory(self, trajectory_name: str) -> TrajectoryResult:
        root = self._project_root()
        robot_config_path = root / "default_data" / "configurations" / "rocky_robodk.json"
        robot_config = RobotConfigurationFile.load(str(robot_config_path))
        robot_model = RobotModel()
        robot_model.load_from_configuration_file(robot_config, str(robot_config_path))

        tool_model = ToolModel()
        if robot_config.default_tool_auto_load_on_startup and robot_config.default_tool_profile:
            tool_path = root / robot_config.default_tool_profile
            tool_profile = ToolConfigFile.load(str(tool_path))
            tool_model.apply_tool_profile(str(tool_path), tool_profile)
            robot_model.set_tool(tool_model.get_tool())

        workspace_model = WorkspaceModel()
        workspace_path = root / "user_data" / "workspaces" / "scene_easy.json"
        if workspace_path.exists():
            WorkspaceFile.load(str(workspace_path)).apply_to_workspace_model(workspace_model, str(workspace_path))

        trajectory_path = root / "user_data" / "trajectories" / trajectory_name
        with trajectory_path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        keypoints = [TrajectoryKeypoint.from_dict(item) for item in payload["keypoints"]]
        segments = [TrajectorySegment(keypoints[index], keypoints[index + 1]) for index in range(len(keypoints) - 1)]
        builder = TrajectoryBuilder(
            robot_model,
            tool_model,
            workspace_model,
            cartesian_accel_limit_mm_s2=float(payload["cartesian_accel_limit_mm_s2"]),
            cartesian_jerk_limit_mm_s3=float(payload["cartesian_jerk_limit_mm_s3"]),
        )
        return builder.compute_trajectory(robot_model.get_home_position(), segments)

    @staticmethod
    def _flatten_samples(result: TrajectoryResult) -> list[tuple[int, int, TrajectorySample]]:
        samples: list[tuple[int, int, TrajectorySample]] = []
        for segment_index, segment in enumerate(result.segments):
            for sample_index, sample in enumerate(segment.samples):
                samples.append((segment_index, sample_index, sample))
        return samples

    def _assert_uniform_sample_ticks(self, result: TrajectoryResult, sample_dt_s: float = 0.004) -> None:
        samples = self._flatten_samples(result)
        for _segment_index, _sample_index, sample in samples:
            tick = round(sample.time / sample_dt_s)
            self.assertAlmostEqual(sample.time, tick * sample_dt_s, places=9)
        for index in range(1, len(samples)):
            dt_s = samples[index][2].time - samples[index - 1][2].time
            self.assertAlmostEqual(dt_s, sample_dt_s, places=9)

    def _assert_lin_flyby_junction_is_uniform(self, trajectory_name: str, expected_speed_mm_s: float) -> None:
        result = self._compute_fixture_trajectory(trajectory_name)
        self._assert_uniform_sample_ticks(result)
        samples = self._flatten_samples(result)
        boundary_index = next(
            index
            for index, (segment_index, sample_index, _sample) in enumerate(samples)
            if segment_index == 2 and sample_index == 0
        )
        previous_sample = samples[boundary_index - 1][2]
        first_next_sample = samples[boundary_index][2]
        dt_s = first_next_sample.time - previous_sample.time
        self.assertAlmostEqual(dt_s, 0.004, places=9)
        self.assertAlmostEqual(previous_sample.cartesian_velocity[1], expected_speed_mm_s, delta=2.0)
        self.assertAlmostEqual(first_next_sample.cartesian_velocity[1], expected_speed_mm_s, delta=2.0)

        local_samples = [
            sample
            for segment_index, _sample_index, sample in samples
            if segment_index in (1, 2) and abs(sample.pose[1]) <= 20.0
        ]
        local_max_jerk = max(
            max(abs(value) for value in sample.articular_jerk)
            for sample in local_samples
            if sample.articular_jerk_valid
        )
        self.assertLess(local_max_jerk, 10000.0)

    def test_fixture_flyby_junctions_use_uniform_sampling_grid(self) -> None:
        self._assert_lin_flyby_junction_is_uniform("LIN_2_seg_same_v_fly.json", 500.0)
        self._assert_lin_flyby_junction_is_uniform("LIN_2_seg_diff_v_fly.json", 250.0)

    def test_fixture_stop_junctions_keep_stop_and_durations(self) -> None:
        same_speed = self._compute_fixture_trajectory("LIN_2_seg_same_v_stop.json")
        different_speed = self._compute_fixture_trajectory("LIN_2_seg_diff_v_stop.json")

        self._assert_uniform_sample_ticks(same_speed)
        self._assert_uniform_sample_ticks(different_speed)

        self.assertAlmostEqual(same_speed.segments[1].duration, 1.7291666666666592, places=9)
        self.assertAlmostEqual(same_speed.segments[2].duration, 1.7291666666666592, places=9)
        self.assertAlmostEqual(different_speed.segments[1].duration, 1.7291666666666592, places=9)
        self.assertAlmostEqual(different_speed.segments[2].duration, 2.433393251112807, places=9)

        for result in (same_speed, different_speed):
            samples = self._flatten_samples(result)
            boundary_index = next(
                index
                for index, (segment_index, sample_index, _sample) in enumerate(samples)
                if segment_index == 2 and sample_index == 0
            )
            last_previous_sample = samples[boundary_index - 1][2]
            first_next_sample = samples[boundary_index][2]
            self.assertLess(abs(last_previous_sample.cartesian_velocity[1]), 0.01)
            self.assertLess(abs(first_next_sample.cartesian_velocity[1]), 0.01)

    def test_bezier7_linear_coefficients_and_endpoints(self) -> None:
        curve = Bezier7Curve3D.linear(XYZ3(0.0, 0.0, 0.0), XYZ3(7.0, 0.0, 0.0))

        _assert_xyz_close(self, curve.coefficients.a0, XYZ3(0.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.coefficients.a1, XYZ3(7.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.coefficients.a2, XYZ3(0.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.coefficients.a7, XYZ3(0.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.point(0.0), XYZ3(0.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.point(1.0), XYZ3(7.0, 0.0, 0.0))
        self.assertGreater(curve.point(0.75).x, curve.point(0.25).x)

    def test_arc_length_straight_line_and_inverse(self) -> None:
        curve = Bezier7Curve3D.linear(XYZ3(0.0, 0.0, 0.0), XYZ3(100.0, 0.0, 0.0))
        lut = build_arc_length_lut(curve, 20)
        self.assertAlmostEqual(lut.total_length_mm, 100.0, places=6)
        self.assertAlmostEqual(parameter_at_distance(lut, 50.0), 0.5, places=6)
        self.assertEqual(parameter_at_distance(lut, -1.0), 0.0)
        self.assertEqual(parameter_at_distance(lut, 1000.0), 1.0)

    def test_arc_length_cancel_stops_sampling(self) -> None:
        curve = Bezier7Curve3D.linear(XYZ3(0.0, 0.0, 0.0), XYZ3(100.0, 0.0, 0.0))
        lut = build_arc_length_lut(curve, 100, _CancelToken())
        self.assertLess(len(lut.parameters_u), 101)

    def test_ptp_duration_uses_degree7_peak_speed_scale(self) -> None:
        self.assertAlmostEqual(ptp_duration_s(100.0, 50.0), 100.0 * S_CURVE_PEAK_SPEED_SCALE / 50.0)

    def test_ptp_jerk_duration_uses_degree7_peak_jerk_scale(self) -> None:
        self.assertAlmostEqual(ptp_jerk_duration_s(90.0, 420.0), (90.0 * S_CURVE_PEAK_JERK_SCALE / 420.0) ** (1.0 / 3.0))

    def test_ptp_duration_includes_jerk_limit(self) -> None:
        builder = object.__new__(TrajectoryBuilder)
        builder.robot_model = _PtpDurationRobotModel()
        segment = TrajectorySegment(TrajectoryKeypoint(), TrajectoryKeypoint(ptp_speed_percent=100.0))
        delta = JointAngles6(90.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.assertAlmostEqual(builder._ptp_duration(segment, delta, 0.100), ptp_jerk_duration_s(90.0, 420.0))

    def test_ptp_duration_is_zero_without_joint_motion(self) -> None:
        builder = object.__new__(TrajectoryBuilder)
        builder.robot_model = _PtpDurationRobotModel()
        segment = TrajectorySegment(TrajectoryKeypoint(), TrajectoryKeypoint(ptp_speed_percent=100.0))
        delta = JointAngles6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        self.assertEqual(builder._ptp_duration(segment, delta, 0.100), 0.0)

    def test_ptp_analytic_articular_dynamics_are_valid_at_bounds(self) -> None:
        delta = JointAngles6(90.0, -45.0, 10.0, 0.0, 5.0, -2.5)
        duration_s = 2.0

        for local_time_s in (0.0, duration_s):
            sample = TrajectorySample()
            sample.reachable = True
            TrajectoryBuilder._apply_ptp_analytic_articular_dynamics(sample, delta, duration_s, local_time_s)

            self.assertTrue(sample.articular_velocity_valid)
            self.assertTrue(sample.articular_acceleration_valid)
            self.assertTrue(sample.articular_jerk_valid)
            for axis in range(6):
                self.assertAlmostEqual(sample.articular_velocity[axis], 0.0, places=9)
                self.assertAlmostEqual(sample.articular_acceleration[axis], 0.0, places=9)
                self.assertAlmostEqual(sample.articular_jerk[axis], 0.0, places=9)

    def test_ptp_analytic_articular_dynamics_are_not_finite_difference_based(self) -> None:
        delta = JointAngles6(90.0, -45.0, 0.0, 0.0, 0.0, 0.0)
        sample = TrajectorySample()
        sample.reachable = True

        TrajectoryBuilder._apply_ptp_analytic_articular_dynamics(sample, delta, 2.0, 1.0)

        self.assertTrue(sample.articular_velocity_valid)
        self.assertTrue(sample.articular_acceleration_valid)
        self.assertTrue(sample.articular_jerk_valid)
        self.assertGreater(sample.articular_velocity[0], 0.0)
        self.assertLess(sample.articular_velocity[1], 0.0)
        self.assertAlmostEqual(sample.articular_acceleration[0], 0.0, places=9)
        self.assertLess(sample.articular_jerk[0], 0.0)

    def test_long_profile_has_cruise_and_continuous_bounds(self) -> None:
        resolution = resolve_segment_dynamic_profile(1000.0, 500.0, 0.0, 0.0, 1000.0, 10000.0)
        self.assertEqual(resolution.profile_kind, SegmentDynamicProfileKind.TRAPEZOIDAL)
        self.assertTrue(resolution.target_speed_reached)
        self.assertAlmostEqual(resolution.peak_speed_mm_s, 500.0, places=6)
        self.assertGreater(resolution.cruise_distance_mm, 0.0)

        profile = build_distance_profile(0, 1000.0, 500.0, 0.0, 0.0, 1000.0, 10000.0)
        kinds = [phase.kind for phase in profile.phases]
        self.assertIn(SegmentDynamicPhaseKind.CRUISE, kinds)
        start = profile.evaluate(0.0)
        end = profile.evaluate(profile.duration_s)
        self.assertAlmostEqual(start.velocity, 0.0, places=6)
        self.assertAlmostEqual(start.acceleration, 0.0, places=6)
        self.assertAlmostEqual(end.velocity, 0.0, places=6)
        self.assertAlmostEqual(end.acceleration, 0.0, places=6)

    def test_short_profile_removes_cruise(self) -> None:
        resolution = resolve_segment_dynamic_profile(10.0, 500.0, 0.0, 0.0, 1000.0, 10000.0)
        self.assertEqual(resolution.profile_kind, SegmentDynamicProfileKind.TRIANGULAR)
        self.assertFalse(resolution.target_speed_reached)
        self.assertLess(resolution.peak_speed_mm_s, 500.0)
        self.assertEqual(resolution.cruise_distance_mm, 0.0)

        profile = build_distance_profile(0, 10.0, 500.0, 0.0, 0.0, 1000.0, 10000.0)
        self.assertNotIn(SegmentDynamicPhaseKind.CRUISE, [phase.kind for phase in profile.phases])
        self.assertAlmostEqual(profile.evaluate(profile.duration_s).position, 10.0, places=5)

    def test_very_short_profile_never_creates_negative_or_zero_cruise(self) -> None:
        resolution = resolve_segment_dynamic_profile(0.001, 500.0, 0.0, 0.0, 1000.0, 10000.0)
        self.assertEqual(resolution.profile_kind, SegmentDynamicProfileKind.TRIANGULAR)
        self.assertEqual(resolution.cruise_distance_mm, 0.0)
        self.assertGreaterEqual(resolution.accel_distance_mm, 0.0)
        self.assertGreaterEqual(resolution.decel_distance_mm, 0.0)

        profile = build_distance_profile(0, 0.001, 500.0, 0.0, 0.0, 1000.0, 10000.0)
        self.assertNotIn(SegmentDynamicPhaseKind.CRUISE, [phase.kind for phase in profile.phases])
        self.assertTrue(all(phase.duration_s > 0.0 for phase in profile.phases))

    def test_symmetric_triangular_profile_peaks_at_midpoint(self) -> None:
        resolution = resolve_segment_dynamic_profile(10.0, 500.0, 0.0, 0.0, 1000.0, 10000.0)
        profile = build_distance_profile(0, 10.0, 500.0, 0.0, 0.0, 1000.0, 10000.0)
        midpoint_time_s = profile.duration_s * 0.5
        before = profile.evaluate(midpoint_time_s - profile.duration_s * 0.1)
        midpoint = profile.evaluate(midpoint_time_s)
        after = profile.evaluate(midpoint_time_s + profile.duration_s * 0.1)

        self.assertEqual(len(profile.phases), 2)
        self.assertAlmostEqual(profile.phases[0].end_time_s(), midpoint_time_s, places=9)
        self.assertAlmostEqual(midpoint.velocity, resolution.peak_speed_mm_s, places=6)
        self.assertAlmostEqual(midpoint.acceleration, 0.0, places=6)
        self.assertLess(before.velocity, midpoint.velocity)
        self.assertLess(after.velocity, midpoint.velocity)

    def test_speed_transition_keeps_non_zero_exit_speed(self) -> None:
        resolution = resolve_segment_dynamic_profile(300.0, 500.0, 0.0, 200.0, 1000.0, 10000.0)
        self.assertGreaterEqual(resolution.peak_speed_mm_s, 200.0)
        self.assertGreaterEqual(resolution.cruise_distance_mm, 0.0)

        profile = build_distance_profile(0, 300.0, 500.0, 0.0, 200.0, 1000.0, 10000.0)
        end = profile.evaluate(profile.duration_s)
        self.assertAlmostEqual(end.velocity, 200.0, places=5)
        self.assertAlmostEqual(end.acceleration, 0.0, places=5)

    def test_full_builder_continues_on_error_by_default(self) -> None:
        builder = object.__new__(TrajectoryBuilder)
        builder.behavior = TrajectoryBuilderBehavior.CONTINUE_ON_ERROR
        segment = SegmentResult()
        segment.status = TrajectoryComputationStatus.JERK_LIMIT_EXCEEDED

        self.assertFalse(builder._should_stop_on_error(segment))

    def test_full_builder_can_stop_on_error(self) -> None:
        builder = object.__new__(TrajectoryBuilder)
        builder.behavior = TrajectoryBuilderBehavior.STOP_ON_ERROR
        segment = SegmentResult()
        segment.status = TrajectoryComputationStatus.JERK_LIMIT_EXCEEDED

        self.assertTrue(builder._should_stop_on_error(segment))


if __name__ == "__main__":
    unittest.main()
