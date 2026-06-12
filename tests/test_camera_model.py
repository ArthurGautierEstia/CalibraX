from __future__ import annotations

import os
import unittest

import numpy as np

from models.camera_model import (
    CameraConfiguration,
    CameraConfigurationFile,
    CameraFov,
    CameraTargetBody,
    CameraTargetPoint,
    CameraVisual,
    CameraVisibilityState,
    evaluate_camera_fov,
)
from models.types import Pose6


class CameraModelTest(unittest.TestCase):
    def test_load_default_camera_setup(self) -> None:
        config = CameraConfigurationFile.load(
            os.path.join("default_data", "configurations", "cameras", "default_camera_setup.json")
        )
        self.assertEqual(len(config.cameras), 8)
        self.assertEqual(config.validate(), [])

    def test_duplicate_camera_ids_are_invalid(self) -> None:
        camera = CameraConfiguration.default(1)
        config = CameraConfigurationFile([camera, camera])
        errors = config.validate()
        self.assertTrue(any("duplique" in error for error in errors))

    def test_range_m_is_converted_to_mm(self) -> None:
        fov = CameraFov.from_dict({"horizontal_deg": 50.0, "vertical_deg": 40.0, "range_m": 3.5})
        self.assertEqual(fov.range_mm, 3500.0)

    def test_legacy_visual_flags_are_converted_to_marker_flags(self) -> None:
        visual = CameraVisual.from_dict({"show_line_to_tcp": False, "verify_tcp_in_fov": False})
        self.assertFalse(visual.show_lines_to_markers)
        self.assertFalse(visual.verify_markers_in_fov)
        self.assertFalse(visual.show_lines_to_target_points)
        self.assertFalse(visual.verify_target_points_in_fov)
        self.assertFalse(visual.show_line_to_tcp)
        self.assertFalse(visual.verify_tcp_in_fov)

    def test_camera_configuration_file_loads_default_target_body(self) -> None:
        config = CameraConfigurationFile.from_dict({"name": "Legacy", "cameras": []})
        self.assertEqual(config.target_body.name, "Rigid Body")
        self.assertEqual(config.target_body.parent_frame, "frame_6")
        self.assertEqual(len(config.target_body.points), 0)
        self.assertEqual(config.validate(), [])

    def test_camera_configuration_file_serializes_target_body(self) -> None:
        target_body = CameraTargetBody(
            name="Calibration body",
            parent_frame="tool",
            pose=Pose6(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
            points=(CameraTargetPoint("P1", "Point 1", 10.0, 20.0, 30.0),),
        )
        config = CameraConfigurationFile(cameras=[], target_body=target_body)
        data = config.to_dict()
        self.assertEqual(data["target_body"]["parent_frame"], "tool")
        self.assertEqual(data["target_body"]["pose"], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(data["target_body"]["markers"][0]["x"], 10.0)
        self.assertEqual(data["target_body"]["markers"][0]["diameter_mm"], 20.0)

    def test_duplicate_target_point_ids_are_invalid(self) -> None:
        target_body = CameraTargetBody(
            points=(
                CameraTargetPoint("P1", "Point 1"),
                CameraTargetPoint("P1", "Point 1 copy"),
            )
        )
        config = CameraConfigurationFile(cameras=[], target_body=target_body)
        errors = config.validate()
        self.assertTrue(any("ID marker duplique" in error for error in errors))

    def test_evaluate_camera_fov_visible_and_outside(self) -> None:
        camera = CameraConfiguration(
            camera_id="cam_test",
            name="Camera test",
            fov=CameraFov(horizontal_deg=60.0, vertical_deg=40.0, range_mm=1000.0),
        )
        identity = np.eye(4, dtype=float)
        visible = evaluate_camera_fov(camera, np.array([0.0, 0.0, 500.0], dtype=float), identity)
        self.assertEqual(visible.state, CameraVisibilityState.VISIBLE)

        outside = evaluate_camera_fov(camera, np.array([500.0, 0.0, 500.0], dtype=float), identity)
        self.assertEqual(outside.state, CameraVisibilityState.OUT_OF_FOV)

        behind = evaluate_camera_fov(camera, np.array([0.0, 0.0, -100.0], dtype=float), identity)
        self.assertEqual(behind.state, CameraVisibilityState.OUT_OF_FOV)

    def test_mount_and_optical_pose_are_composed(self) -> None:
        camera = CameraConfiguration(
            camera_id="cam_test",
            name="Camera test",
            mount_pose=Pose6(100.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            optical_pose=Pose6(25.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertAlmostEqual(float(camera.optical_matrix()[0, 3]), 125.0)


if __name__ == "__main__":
    unittest.main()
