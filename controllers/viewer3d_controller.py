from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from widgets.viewer_3d_widget import Viewer3DWidget

class Viewer3DController(QObject):
    def __init__(self, robot_model: RobotModel, viewer_3d_widget: Viewer3DWidget, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.viewer_3d_widget = viewer_3d_widget
        self._ghost_visible = False
        self._ghost_joints: list[float] = [0.0] * 6

        self._setup_connections()

    def _setup_connections(self) -> None:
        self.robot_model.tcp_pose_changed.connect(self._update_tcp_pose)
        self.robot_model.robot_cad_models_changed.connect(self._on_robot_cad_models_changed)
        self.robot_model.tool_cad_model_changed.connect(self._on_tool_cad_model_changed)

    def _update_tcp_pose(self) -> None:
        self.viewer_3d_widget.update_robot(self.robot_model)

    def _on_robot_cad_models_changed(self) -> None:
        self.viewer_3d_widget.load_cad(self.robot_model)

    def _on_tool_cad_model_changed(self) -> None:
        self.viewer_3d_widget.reload_tool_cad(self.robot_model)

    def show_robot_ghost(self) -> None:
        self._ghost_visible = True
        self.viewer_3d_widget.show_robot_ghost()

    def hide_robot_ghost(self) -> None:
        self._ghost_visible = False
        self.viewer_3d_widget.hide_robot_ghost()

    def update_robot_ghost(self, joints: list[float]) -> None:
        if len(joints) < 6:
            self.hide_robot_ghost()
            return

        self._ghost_joints = [float(joint) for joint in joints[:6]]
        if not self._ghost_visible:
            self.show_robot_ghost()

        self.viewer_3d_widget.update_robot_ghost(self._ghost_joints)

    def set_trajectory_path(self, points_xyz: list[list[float]]) -> None:
        self.viewer_3d_widget.set_trajectory_path(points_xyz)

    def clear_trajectory_path(self) -> None:
        self.viewer_3d_widget.clear_trajectory_path()

    def set_trajectory_keypoints(
        self,
        points_xyz: list[list[float]],
        selected_index: int | None = None,
        editing_index: int | None = None,
    ) -> None:
        self.viewer_3d_widget.set_trajectory_keypoints(points_xyz, selected_index, editing_index)

    def clear_trajectory_keypoints(self) -> None:
        self.viewer_3d_widget.clear_trajectory_keypoints()

    def set_trajectory_edit_tangents(
        self,
        tangent_out_segment: list[list[float]] | None,
        tangent_in_segment: list[list[float]] | None,
    ) -> None:
        self.viewer_3d_widget.set_trajectory_edit_tangents(tangent_out_segment, tangent_in_segment)

    def clear_trajectory_edit_tangents(self) -> None:
        self.viewer_3d_widget.clear_trajectory_edit_tangents()
