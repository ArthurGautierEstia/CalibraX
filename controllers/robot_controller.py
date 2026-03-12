from PyQt6.QtCore import QObject, pyqtSignal
from models.robot_model import RobotModel
from views.robot_view import RobotView
from views.calibration_view import CalibrationView
from controllers.robot_view.dh_table_controller import DHTableController
from controllers.robot_view.measurement_controller import MeasurementController


class RobotController(QObject):
    configuration_loaded = pyqtSignal()

    def __init__(self, robot_model: RobotModel, robot_view: RobotView, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.robot_view = robot_view

        self.dh_controller = DHTableController(self.robot_model, self.robot_view.get_dh_widget())

        self._setup_connections()

    def _setup_connections(self) -> None:
        self.dh_controller.configuration_loaded.connect(self.configuration_loaded.emit)
