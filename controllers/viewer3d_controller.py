from PyQt6.QtCore import QObject

from controllers.cartesian_control_view.cartesian_widget_controller import CartesianWidgetController
from controllers.joint_control_view.joints_controller import JointsController
from models.collision_scene_model import CollisionSceneModel
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.types import XYZ3
from models.workspace_model import WorkspaceModel
from widgets.viewer_3d_widget import Viewer3DWidget


TangentSegment = tuple[XYZ3, XYZ3]


class Viewer3DController(QObject):
    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        collision_scene_model: CollisionSceneModel,
        viewer_3d_widget: Viewer3DWidget,
        parent: QObject = None,
    ):
        super().__init__(parent)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.collision_scene_model = collision_scene_model
        self.viewer_3d_widget = viewer_3d_widget
        self._ghost_visible = False
        self._ghost_joints: list[float] = [0.0] * 6
        self._overlay_joints_controller = JointsController(
            self.robot_model,
            self.viewer_3d_widget.get_overlay_joints_widget(),
            self,
        )
        self._overlay_cartesian_controller = CartesianWidgetController(
            self.robot_model,
            self.workspace_model,
            self.viewer_3d_widget.get_overlay_cartesian_widget(),
            self,
        )

        self._setup_connections()
        self._initialize_overlay_controls()
        self.viewer_3d_widget.update_workspace(self.workspace_model)
        self.viewer_3d_widget.update_collision_scene(self.collision_scene_model)

    def _setup_connections(self) -> None:
        self.robot_model.tcp_pose_changed.connect(self._update_tcp_pose)
        self.robot_model.robot_cad_models_changed.connect(self._on_robot_cad_models_changed)

        self.tool_model.tool_changed.connect(self._on_tool_state_changed)
        self.tool_model.tool_visual_changed.connect(self._on_tool_visual_changed)

        self.workspace_model.workspace_changed.connect(self._on_workspace_changed)
        self.collision_scene_model.scene_changed.connect(self._on_collision_scene_changed)
        self._overlay_cartesian_controller.new_target_computed.connect(self._on_overlay_cartesian_target_computed)

    def _update_tcp_pose(self) -> None:
        self.viewer_3d_widget.update_robot(self.robot_model, self.tool_model)

    def _on_robot_cad_models_changed(self) -> None:
        self.viewer_3d_widget.load_cad(self.robot_model, self.tool_model)

    def _on_tool_state_changed(self) -> None:
        self.viewer_3d_widget.update_robot(self.robot_model, self.tool_model)

    def _on_tool_visual_changed(self) -> None:
        self.viewer_3d_widget.reload_tool_cad(self.robot_model, self.tool_model)
        self.viewer_3d_widget.update_collision_scene(self.collision_scene_model)

    def _on_collision_scene_changed(self) -> None:
        self.viewer_3d_widget.update_collision_scene(self.collision_scene_model)

    def _on_workspace_changed(self) -> None:
        self.viewer_3d_widget.update_workspace(self.workspace_model)

    def _on_overlay_cartesian_target_computed(self) -> None:
        target = self._overlay_cartesian_controller.get_new_target()
        mgi_result = self.robot_model.compute_ik_target(target, tool=self.tool_model.get_tool())
        best_sol = self.robot_model.get_best_mgi_solution(mgi_result)
        if not best_sol:
            return
        _config_key, solution = best_sol
        self.robot_model.set_joints(solution.joints)

    def _initialize_overlay_controls(self) -> None:
        overlay_joints_widget = self.viewer_3d_widget.get_overlay_joints_widget()
        overlay_joints_widget.update_axis_limits(self.robot_model.get_axis_limits())
        overlay_joints_widget.set_all_joints(self.robot_model.get_joints())
        overlay_joints_widget.set_configuration(self.robot_model.get_current_axis_config())

        self._overlay_cartesian_controller._apply_cartesian_slider_limits()
        self._overlay_cartesian_controller._on_model_tcp_changed()

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

    def update_robot_ghost_with_matrices(self, joints: list[float], corrected_matrices: list) -> None:
        if len(joints) < 6 or not corrected_matrices:
            self.hide_robot_ghost()
            return

        self._ghost_joints = [float(joint) for joint in joints[:6]]
        if not self._ghost_visible:
            self.show_robot_ghost()

        self.viewer_3d_widget.update_robot_ghost_from_matrices(corrected_matrices)

    def set_trajectory_path_segments(
        self,
        segments: list[tuple[list[list[float]], tuple[float, float, float, float]]],
    ) -> None:
        self.viewer_3d_widget.set_trajectory_path_segments(segments)

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
        tangent_out_segments: list[TangentSegment] | None,
        tangent_in_segments: list[TangentSegment] | None,
    ) -> None:
        self.viewer_3d_widget.set_trajectory_edit_tangents(tangent_out_segments, tangent_in_segments)

    def clear_trajectory_edit_tangents(self) -> None:
        self.viewer_3d_widget.clear_trajectory_edit_tangents()
