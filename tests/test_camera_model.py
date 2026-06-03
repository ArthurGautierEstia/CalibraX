from __future__ import annotations

import os
import unittest

import numpy as np

from models.camera_model import (
    CameraConfiguration,
    CameraConfigurationFile,
    CameraFov,
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
