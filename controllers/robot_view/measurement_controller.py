from PyQt5.QtCore import QObject

from models.robot_model import RobotModel
from widgets.robot_view.measurement_widget import MeasurementWidget

class MeasurementController(QObject):
    def __init__(self, robot_model: RobotModel, measurement_widget: MeasurementWidget, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.measurement_widget = measurement_widget
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        # Signals from Robot Model
        self.robot_model.measurements_changed.connect(self._on_robot_measurements_changed)
        self.robot_model.measurements_points_changed.connect(self._on_robot_measurements_points_changed)

        # Signals from View
        self.measurement_widget.import_measurements_requested.connect(self._on_view_import_measurements_requested)

    # ======
    # Connection callbacks
    # ======

    def _on_robot_measurements_changed(self) -> None:
        self.measurement_widget.set_measure_filename(self.robot_model.get_measurements_filename())

    def _on_robot_measurements_points_changed(self) -> None:
        self.measurement_widget.set_measurements_data(self.robot_model.get_measurements())

    def _on_view_import_measurements_requested(self) -> None:
        pass
