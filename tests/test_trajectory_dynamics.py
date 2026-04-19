import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from models.trajectory_result import (
    SegmentResult,
    TrajectoryDynamicViolation,
    TrajectoryDynamicViolationKind,
    TrajectoryDynamicViolationSeverity,
    TrajectorySample,
    TrajectorySampleErrorCode,
)
from utils.trajectory_builder import TrajectoryBuilder
from utils.trajectory_status import build_segment_issue_messages, build_segment_warning_messages


class TrajectoryDynamicsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.builder = TrajectoryBuilder.__new__(TrajectoryBuilder)
        self.builder.sample_dt_s = 0.1

    def test_articular_jerk_is_computed_from_acceleration_delta(self) -> None:
        previous = TrajectorySample()
        previous.time = 0.0
        previous.joints = [0.0] * 6
        previous.articular_velocity = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        previous.articular_acceleration = [20.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        sample = TrajectorySample()
        sample.time = 0.1
        sample.joints = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.builder._update_articular_dynamics(sample, previous, 0.1)

        self.assertAlmostEqual(sample.articular_velocity[0], 20.0)
        self.assertAlmostEqual(sample.articular_acceleration[0], 100.0)
        self.assertAlmostEqual(sample.articular_jerk[0], 800.0)

    def test_speed_violation_is_blocking_error(self) -> None:
        previous = TrajectorySample()
        sample = TrajectorySample()
        sample.time = 0.1
        sample.articular_velocity[0] = 12.0

        self.builder._apply_dynamic_limits_if_needed(
            sample,
            previous,
            [10.0] * 6,
            [1000.0] * 6,
            [1000.0] * 6,
        )

        self.assertEqual(sample.error_code, TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED)
        self.assertEqual(sample.error_axis, 0)
        self.assertEqual(len(sample.dynamic_violations), 1)
        self.assertEqual(sample.dynamic_violations[0].kind, TrajectoryDynamicViolationKind.SPEED)
        self.assertEqual(sample.dynamic_violations[0].severity, TrajectoryDynamicViolationSeverity.ERROR)

    def test_jerk_violation_is_blocking_error(self) -> None:
        previous = TrajectorySample()
        sample = TrajectorySample()
        sample.time = 0.1
        sample.articular_jerk[1] = 50.0

        self.builder._apply_dynamic_limits_if_needed(
            sample,
            previous,
            [1000.0] * 6,
            [1000.0] * 6,
            [20.0] * 6,
        )

        self.assertEqual(sample.error_code, TrajectorySampleErrorCode.JERK_LIMIT_EXCEEDED)
        self.assertEqual(sample.error_axis, 1)
        self.assertEqual(sample.dynamic_violations[0].kind, TrajectoryDynamicViolationKind.JERK)

    def test_acceleration_violation_is_warning_only(self) -> None:
        previous = TrajectorySample()
        sample = TrajectorySample()
        sample.time = 0.1
        sample.articular_acceleration[2] = 80.0

        self.builder._apply_dynamic_limits_if_needed(
            sample,
            previous,
            [1000.0] * 6,
            [50.0] * 6,
            [1000.0] * 6,
        )

        self.assertEqual(sample.error_code, TrajectorySampleErrorCode.NONE)
        self.assertIsNone(sample.error_axis)
        self.assertEqual(len(sample.dynamic_violations), 1)
        self.assertEqual(sample.dynamic_violations[0].kind, TrajectoryDynamicViolationKind.ACCELERATION)
        self.assertEqual(sample.dynamic_violations[0].severity, TrajectoryDynamicViolationSeverity.WARNING)

    def test_multiple_dynamic_violations_are_preserved(self) -> None:
        previous = TrajectorySample()
        sample = TrajectorySample()
        sample.time = 0.1
        sample.articular_velocity[2] = 20.0
        sample.articular_acceleration[0] = 80.0
        sample.articular_jerk[1] = 70.0

        self.builder._apply_dynamic_limits_if_needed(
            sample,
            previous,
            [10.0] * 6,
            [50.0] * 6,
            [60.0] * 6,
        )

        self.assertEqual(sample.error_code, TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED)
        self.assertEqual(sample.error_axis, 2)
        self.assertEqual(
            {(violation.kind, violation.axis) for violation in sample.dynamic_violations},
            {
                (TrajectoryDynamicViolationKind.SPEED, 2),
                (TrajectoryDynamicViolationKind.ACCELERATION, 0),
                (TrajectoryDynamicViolationKind.JERK, 1),
            },
        )

    def test_status_messages_split_errors_and_warnings(self) -> None:
        segment = SegmentResult()
        sample = TrajectorySample()
        sample.error_code = TrajectorySampleErrorCode.SPEED_LIMIT_EXCEEDED
        sample.error_axis = 0
        sample.dynamic_violations = [
            TrajectoryDynamicViolation(
                TrajectoryDynamicViolationKind.SPEED,
                0,
                12.0,
                10.0,
                TrajectoryDynamicViolationSeverity.ERROR,
            ),
            TrajectoryDynamicViolation(
                TrajectoryDynamicViolationKind.ACCELERATION,
                2,
                80.0,
                50.0,
                TrajectoryDynamicViolationSeverity.WARNING,
            ),
        ]
        segment.samples.append(sample)

        self.assertEqual(build_segment_issue_messages(segment, 0), ["Segment 1: vitesse depassee (J1)"])
        self.assertEqual(
            build_segment_warning_messages(segment, 0),
            ["Segment 1: acceleration estimee depassee (J3)"],
        )


class TrajectoryExportAndGraphTest(unittest.TestCase):
    def test_export_header_and_violation_format_include_jerk_and_dynamic_columns(self) -> None:
        from controllers.trajectory_controller import TrajectoryController

        header = TrajectoryController._trajectory_export_header()
        self.assertIn("dddj1", header)
        self.assertIn("dddx", header)
        self.assertIn("dynamic_errors", header)
        self.assertIn("dynamic_warnings", header)

        sample = TrajectorySample()
        sample.dynamic_violations.append(
            TrajectoryDynamicViolation(
                TrajectoryDynamicViolationKind.JERK,
                4,
                42.0,
                21.0,
                TrajectoryDynamicViolationSeverity.ERROR,
            )
        )
        self.assertEqual(
            TrajectoryController._format_dynamic_violations(sample, TrajectoryDynamicViolationSeverity.ERROR),
            "JERK:J5:42.000000/21.000000",
        )

    def test_graph_panel_accepts_jerk_series(self) -> None:
        from PyQt6.QtWidgets import QApplication
        from widgets.trajectory_view.trajectory_graph_panel_widget import GraphMode, TrajectoryGraphPanelWidget

        app = QApplication.instance() or QApplication([])
        panel = TrajectoryGraphPanelWidget(GraphMode.ARTICULAR)
        series = [[0.0, 1.0] for _ in range(6)]

        panel.set_trajectories([0.0, 0.1], series, series, series, series)

        self.assertEqual(len(panel._plots), 4)
        self.assertEqual(panel._plot_data[3][0], [0.0, 1.0])
        panel.deleteLater()
        app.processEvents()


if __name__ == "__main__":
    unittest.main()
