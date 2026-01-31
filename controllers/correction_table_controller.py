from PyQt5.QtCore import QObject

from models.robot_model import RobotModel
from widgets.correction_table_widget import CorrectionTableWidget


class CorrectionTableController(QObject):
    def __init__(self, robot_model: RobotModel, correction_table_widget: CorrectionTableWidget, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.correction_table_widget = correction_table_widget

        self._setup_connections()

    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modèle du robot"""
        pass
