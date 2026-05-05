from PyQt6.QtCore import QObject, pyqtSignal
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from views.robot_view import RobotView
from views.tool_view import ToolView
from controllers.robot_view.robot_configuration_controller import RobotConfigurationController
from controllers.robot_view.robot_mgi_configuration_controller import RobotMgiConfigurationController
from controllers.tool_controller import ToolController
from controllers.calibration_view.measurement_controller import MeasurementController


class RobotController(QObject):
    configuration_loaded = pyqtSignal()

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        robot_view: RobotView,
        tool_view: ToolView,
        parent: QObject = None,
    ):
        super().__init__(parent)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.robot_view = robot_view
        self.tool_view = tool_view

        self.tool_controller = ToolController(self.tool_model, self.tool_view.get_configuration_widget())
        self.dh_controller = RobotConfigurationController(
            self.robot_model,
            self.robot_view.get_configuration_widget(),
            tool_controller=self.tool_controller,
        )
        self.mgi_configuration_controller = RobotMgiConfigurationController(
            self.robot_model,
            self.robot_view.get_mgi_configuration_widget(),
        )

        self._setup_connections()

    def _setup_connections(self) -> None:
        self.dh_controller.configuration_loaded.connect(self.configuration_loaded.emit)
