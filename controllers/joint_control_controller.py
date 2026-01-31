from PyQt5.QtCore import QObject

from models.robot_model import RobotModel
from views.joint_control_view import JointControlView


class JointControlController(QObject):
    def __init__(self, robot_model: RobotModel, joint_control_view: JointControlView, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.joint_control_view = joint_control_view
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modèle du robot"""
        pass
