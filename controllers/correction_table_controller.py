from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from widgets.calibration_view.measurement_widget import MeasurementWidget


class CorrectionTableController(QObject):
    def __init__(self, robot_model: RobotModel, correction_table_widget: MeasurementWidget, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.correction_table_widget = correction_table_widget

        self._setup_connections()

    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modèle du robot"""
        # Modèle → Vue
        self.robot_model.corrections_changed.connect(self._on_model_corrections_changed)
        # Vue → Modèle
        self.correction_table_widget.corrections_changed.connect(self._on_view_corrections_changed)

    def _on_model_corrections_changed(self):
        self.correction_table_widget.set_corrections(self.robot_model.get_corrections())

    def _on_view_corrections_changed(self, corrections: list[list[float]]):
        self.robot_model._set_corrections(corrections)
        self.robot_model._update_tcp_pose()
