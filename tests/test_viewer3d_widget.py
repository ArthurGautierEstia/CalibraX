import unittest

from models.robot_model import RobotModel
from widgets.viewer_3d_widget import Viewer3DWidget


class Viewer3DWidgetTests(unittest.TestCase):
    def test_resolve_link_color_uses_neutral_color_when_robot_has_no_configured_colors(self) -> None:
        robot_model = RobotModel()
        viewer = Viewer3DWidget.__new__(Viewer3DWidget)
        viewer._robot_model = robot_model

        color_base = viewer._resolve_link_color(0)
        color_axis_1 = viewer._resolve_link_color(1)
        color_axis_6 = viewer._resolve_link_color(6)

        self.assertEqual((0.5, 0.5, 0.5, 0.5), color_base)
        self.assertEqual((0.5, 0.5, 0.5, 0.5), color_axis_1)
        self.assertEqual((0.5, 0.5, 0.5, 0.5), color_axis_6)

    def test_resolve_link_color_uses_robot_specific_palette(self) -> None:
        robot_model = RobotModel()
        robot_model.set_robot_cad_colors(["#112233", "", "", "", "", "", "#AABBCC"])
        viewer = Viewer3DWidget.__new__(Viewer3DWidget)
        viewer._robot_model = robot_model

        self.assertEqual((17 / 255.0, 34 / 255.0, 51 / 255.0, 0.5), viewer._resolve_link_color(0))
        self.assertEqual((0.5, 0.5, 0.5, 0.5), viewer._resolve_link_color(1))
        self.assertEqual((170 / 255.0, 187 / 255.0, 204 / 255.0, 0.5), viewer._resolve_link_color(6))


if __name__ == "__main__":
    unittest.main()
