import os
import unittest
from pathlib import Path

from PyQt6.QtWidgets import QApplication

from controllers.robot_view.robot_configuration_controller import RobotConfigurationController
from models.robot_configuration_file import RobotConfigurationFile
from models.robot_model import RobotModel
from models.tool_config_file import ToolConfigFile
from widgets.robot_view.robot_configuration_widget import RobotConfigurationWidget
from utils.mgi import RobotTool


class _ToolControllerSpy:
    def __init__(self) -> None:
        self.loaded_paths: list[tuple[str, bool]] = []
        self.reset_calls = 0

    def load_tool_profile_from_path(self, file_path: str, show_errors: bool = False) -> bool:
        self.loaded_paths.append((file_path, show_errors))
        return True

    def reset_tool_configuration(self) -> None:
        self.reset_calls += 1


class RobotConfigurationDefaultToolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        cls._app = QApplication.instance() or QApplication([])

    def test_roundtrip_preserves_default_tool_profile(self) -> None:
        robot_model = RobotModel()
        robot_model.set_robot_name("DemoRobot")

        config = RobotConfigurationFile.from_robot_model(
            robot_model,
            default_tool_profile="./default_data/tools/demo_tool.json",
        )

        exported = config.to_dict()
        reloaded = RobotConfigurationFile.from_dict(exported)

        self.assertEqual("./default_data/tools/demo_tool.json", reloaded.default_tool_profile)

    def test_loading_robot_configuration_loads_default_tool_profile_only_when_enabled(self) -> None:
        output_path = Path("tests") / "_tmp_robot_config_with_tool.json"
        tool_path = Path("tests") / "_tmp_tool_profile_autoload.json"
        robot_model = RobotModel()
        robot_model.set_robot_name("RobotWithTool")
        ToolConfigFile.from_robot_tool(
            "DemoTool",
            RobotTool(),
            "",
            0.0,
            True,
            [],
            [True] * 6,
        ).save(str(tool_path))
        config = RobotConfigurationFile.from_robot_model(
            robot_model,
            default_tool_profile=f"./{tool_path.as_posix()}",
        )
        config.save(str(output_path))

        try:
            widget = RobotConfigurationWidget()
            tool_controller = _ToolControllerSpy()
            controller = RobotConfigurationController(robot_model, widget, tool_controller=tool_controller)

            loaded = controller.load_configuration_from_path(str(output_path), show_errors=False)

            self.assertTrue(loaded)
            self.assertEqual(f"./{tool_path.as_posix()}", widget.get_default_tool_profile())
            self.assertEqual([(f"./{tool_path.as_posix()}", False)], tool_controller.loaded_paths)
            self.assertEqual(0, tool_controller.reset_calls)
        finally:
            output_path.unlink(missing_ok=True)
            tool_path.unlink(missing_ok=True)

    def test_loading_robot_configuration_does_not_load_default_tool_profile_when_disabled(self) -> None:
        output_path = Path("tests") / "_tmp_robot_config_with_disabled_tool.json"
        tool_path = Path("tests") / "_tmp_tool_profile_no_autoload.json"
        robot_model = RobotModel()
        robot_model.set_robot_name("RobotWithDisabledTool")
        ToolConfigFile.from_robot_tool(
            "DemoTool",
            RobotTool(),
            "",
            0.0,
            False,
            [],
            [True] * 6,
        ).save(str(tool_path))
        config = RobotConfigurationFile.from_robot_model(
            robot_model,
            default_tool_profile=f"./{tool_path.as_posix()}",
        )
        config.save(str(output_path))

        try:
            widget = RobotConfigurationWidget()
            tool_controller = _ToolControllerSpy()
            controller = RobotConfigurationController(robot_model, widget, tool_controller=tool_controller)

            loaded = controller.load_configuration_from_path(str(output_path), show_errors=False)

            self.assertTrue(loaded)
            self.assertEqual(f"./{tool_path.as_posix()}", widget.get_default_tool_profile())
            self.assertEqual([], tool_controller.loaded_paths)
            self.assertEqual(1, tool_controller.reset_calls)
        finally:
            output_path.unlink(missing_ok=True)
            tool_path.unlink(missing_ok=True)

    def test_loading_robot_configuration_without_default_tool_resets_tool(self) -> None:
        output_path = Path("tests") / "_tmp_robot_config_without_tool.json"
        robot_model = RobotModel()
        robot_model.set_robot_name("RobotWithoutTool")
        config = RobotConfigurationFile.from_robot_model(robot_model)
        config.save(str(output_path))

        try:
            widget = RobotConfigurationWidget()
            tool_controller = _ToolControllerSpy()
            controller = RobotConfigurationController(robot_model, widget, tool_controller=tool_controller)

            loaded = controller.load_configuration_from_path(str(output_path), show_errors=False)

            self.assertTrue(loaded)
            self.assertEqual("", widget.get_default_tool_profile())
            self.assertEqual([], tool_controller.loaded_paths)
            self.assertEqual(1, tool_controller.reset_calls)
        finally:
            output_path.unlink(missing_ok=True)


if __name__ == "__main__":
    unittest.main()
