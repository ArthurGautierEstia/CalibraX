import unittest

from models.types import JointAngles6, XYZ3
from trajectory_engine.v2.arc_length import build_arc_length_lut, parameter_at_distance
from trajectory_engine.models import SegmentResult, TrajectoryBuilderBehavior, TrajectoryComputationStatus, TrajectorySample
from trajectory_engine.v2.builders.full_builder import TrajectoryBuilderV2
from trajectory_engine.v2.dynamics import (
    S_CURVE_PEAK_SPEED_SCALE,
    SegmentDynamicProfileKind,
    build_distance_profile,
    ptp_duration_s,
    resolve_segment_dynamic_profile,
)
from trajectory_engine.v2.geometry import Bezier7Curve3D
from trajectory_engine.v2.models import Bezier7ControlPoints3D, SegmentDynamicPhaseKind


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


class TrajectoryEngineV2Tests(unittest.TestCase):
    def test_bezier7_linear_coefficients_and_endpoints(self) -> None:
        curve = Bezier7Curve3D.linear(XYZ3(0.0, 0.0, 0.0), XYZ3(7.0, 0.0, 0.0))

        _assert_xyz_close(self, curve.coefficients.a0, XYZ3(0.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.coefficients.a1, XYZ3(7.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.coefficients.a2, XYZ3(0.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.coefficients.a7, XYZ3(0.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.point(0.0), XYZ3(0.0, 0.0, 0.0))
        _assert_xyz_close(self, curve.point(1.0), XYZ3(7.0, 0.0, 0.0))
        self.assertGreater(curve.point(0.75).x, curve.point(0.25).x)

    def test_bezier7_c3_continuation_derivatives_match(self) -> None:
        first = Bezier7Curve3D(
            Bezier7ControlPoints3D(
                XYZ3(0.0, 0.0, 0.0),
                XYZ3(1.0, 0.0, 0.0),
                XYZ3(2.0, 0.0, 0.0),
                XYZ3(3.0, 0.0, 0.0),
                XYZ3(4.0, 1.0, 0.0),
                XYZ3(5.0, 1.0, 0.0),
                XYZ3(6.0, 1.0, 0.0),
                XYZ3(7.0, 1.0, 0.0),
            )
        )
        second = Bezier7Curve3D.c3_continuation(first, XYZ3(14.0, 2.0, 0.0))
        _assert_xyz_close(self, first.point(1.0), second.point(0.0))
        _assert_xyz_close(self, first.first_derivative(1.0), second.first_derivative(0.0))
        _assert_xyz_close(self, first.second_derivative(1.0), second.second_derivative(0.0))
        _assert_xyz_close(self, first.third_derivative(1.0), second.third_derivative(0.0))

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

    def test_ptp_analytic_articular_dynamics_are_valid_at_bounds(self) -> None:
        delta = JointAngles6(90.0, -45.0, 10.0, 0.0, 5.0, -2.5)
        duration_s = 2.0

        for local_time_s in (0.0, duration_s):
            sample = TrajectorySample()
            sample.reachable = True
            TrajectoryBuilderV2._apply_ptp_analytic_articular_dynamics(sample, delta, duration_s, local_time_s)

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

        TrajectoryBuilderV2._apply_ptp_analytic_articular_dynamics(sample, delta, 2.0, 1.0)

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

    def test_full_builder_v2_continues_on_error_by_default(self) -> None:
        builder = object.__new__(TrajectoryBuilderV2)
        builder.behavior = TrajectoryBuilderBehavior.CONTINUE_ON_ERROR
        segment = SegmentResult()
        segment.status = TrajectoryComputationStatus.JERK_LIMIT_EXCEEDED

        self.assertFalse(builder._should_stop_on_error(segment))

    def test_full_builder_v2_can_stop_on_error(self) -> None:
        builder = object.__new__(TrajectoryBuilderV2)
        builder.behavior = TrajectoryBuilderBehavior.STOP_ON_ERROR
        segment = SegmentResult()
        segment.status = TrajectoryComputationStatus.JERK_LIMIT_EXCEEDED

        self.assertTrue(builder._should_stop_on_error(segment))


if __name__ == "__main__":
    unittest.main()
