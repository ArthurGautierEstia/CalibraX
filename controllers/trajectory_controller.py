from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from views.trajectory_view import TrajectoryView
from controllers.viewer3d_controller import Viewer3DController


class TrajectoryController(QObject):
    def __init__(
        self,
        robot_model: RobotModel,
        trajectory_view: TrajectoryView,
        viewer3d_controller: Viewer3DController,
        parent: QObject = None,
    ):
        super().__init__(parent)

        self.robot_model = robot_model
        self.trajectory_view = trajectory_view
        self.viewer3d_controller = viewer3d_controller
        self.config_widget = self.trajectory_view.get_config_widget()

        self._setup_connections()

    def _setup_connections(self) -> None:
        self.config_widget.showRobotGhostRequested.connect(self._on_show_robot_ghost_requested)
        self.config_widget.hideRobotGhostRequested.connect(self._on_hide_robot_ghost_requested)
        self.config_widget.updateRobotGhostRequested.connect(self._on_update_robot_ghost_requested)

    def _on_show_robot_ghost_requested(self) -> None:
        self.viewer3d_controller.show_robot_ghost()

    def _on_hide_robot_ghost_requested(self) -> None:
        self.viewer3d_controller.hide_robot_ghost()

    def _on_update_robot_ghost_requested(self, joints: list[float]) -> None:
        self.viewer3d_controller.update_robot_ghost(joints)
