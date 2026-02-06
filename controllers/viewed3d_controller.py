from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from widgets.viewer_3d_widget import Viewer3DWidget

class Viewed3DController(QObject):
    def __init__(self, robot_model: RobotModel, viewer_3d_widget: Viewer3DWidget, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.viewer_3d_widget = viewer_3d_widget

        self._setup_connections()

    def _setup_connections(self) -> None:
        self.robot_model.tcp_pose_changed.connect(self._update_tcp_pose)

    def _update_tcp_pose(self) -> None:
        self.viewer_3d_widget.update_robot(self.robot_model)
