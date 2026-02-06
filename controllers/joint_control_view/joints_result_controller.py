from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from widgets.joint_control_view.joints_result_table_widget import JointsResultTableWidget


class JointsResultController(QObject):
    def __init__(self, robot_model: RobotModel, joint_result_widget: JointsResultTableWidget, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.joint_result_widget = joint_result_widget
        self._is_view_updating = False
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modèle du robot"""
        self.robot_model.tcp_pose_changed.connect(self._model_tcp_changed)
        
    def _model_tcp_changed(self) -> None:
        self.joint_result_widget.update_results(
            self.robot_model.get_tcp_pose(),
            self.robot_model.get_corrected_tcp_pose(),
            self.robot_model.get_tcp_deviation()
        )
