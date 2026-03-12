from PyQt6.QtCore import QObject, pyqtSignal
from models.robot_model import RobotModel
from views.robot_view import RobotView
from views.calibration_view import CalibrationView
from controllers.calibration_view.measurement_controller import MeasurementController
from controllers.correction_table_controller import CorrectionTableController


class CalibrationController(QObject):
    configuration_loaded = pyqtSignal()

    def __init__(self, robot_model: RobotModel, calibration_view: CalibrationView, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.calibration_view = calibration_view

        self.measurement_controller = MeasurementController(self.robot_model, self.calibration_view.get_measurement_widget())
        self.correction_table_controller = CorrectionTableController(self.robot_model, self.calibration_view.get_correction_widget())
