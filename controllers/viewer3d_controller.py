from PyQt6.QtCore import QObject, QTimer
import numpy as np

from controllers.cartesian_control_view.cartesian_widget_controller import CartesianWidgetController
from controllers.joint_control_view.joints_controller import JointsController
from models.collision_scene_model import CollisionSceneModel
from models.reference_frame import ReferenceFrame
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.types import Pose6, XYZ3
from models.workspace_model import WorkspaceModel
import utils.math_utils as math_utils
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
        self._overlay_jog_delta = 1.0
        self._overlay_active_joint: tuple[int, int] | None = None
        self._overlay_active_cartesian: tuple[int, int] | None = None
        self._overlay_jog_timer = QTimer(self)
        self._overlay_jog_timer.setInterval(20)
        self._overlay_jog_timer.timeout.connect(self._on_overlay_jog_tick)
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
        self.viewer_3d_widget.get_overlay_cartesian_widget().cartesian_value_changed.connect(
            self._on_overlay_cartesian_value_changed
        )
        self.viewer_3d_widget.viewer_control_overlay.jog_delta_changed.connect(self._on_overlay_jog_delta_changed)
        self.viewer_3d_widget.get_overlay_joints_widget().spinbox_jog_pressed.connect(self._on_overlay_joint_pressed)
        self.viewer_3d_widget.get_overlay_joints_widget().spinbox_jog_released.connect(self._on_overlay_joint_released)
        self.viewer_3d_widget.get_overlay_cartesian_widget().spinbox_jog_pressed.connect(self._on_overlay_cartesian_pressed)
        self.viewer_3d_widget.get_overlay_cartesian_widget().spinbox_jog_released.connect(self._on_overlay_cartesian_released)

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

    def _on_overlay_cartesian_value_changed(self, _axis_index: int, _value: float) -> None:
        self._on_overlay_cartesian_target_computed()

    def _initialize_overlay_controls(self) -> None:
        self._overlay_jog_delta = self.viewer_3d_widget.viewer_control_overlay.get_jog_delta()
        overlay_joints_widget = self.viewer_3d_widget.get_overlay_joints_widget()
        overlay_joints_widget.update_axis_limits(self.robot_model.get_axis_limits())
        overlay_joints_widget.set_all_joints(self.robot_model.get_joints())
        overlay_joints_widget.set_configuration(self.robot_model.get_current_axis_config())
        overlay_joints_widget.set_jog_increment(self._overlay_jog_delta)

        self._overlay_cartesian_controller._apply_cartesian_slider_limits()
        self._overlay_cartesian_controller._on_model_tcp_changed()
        self.viewer_3d_widget.get_overlay_cartesian_widget().set_jog_increment(self._overlay_jog_delta)

    def _on_overlay_jog_delta_changed(self, value: float) -> None:
        self._overlay_jog_delta = max(0.01, float(value))

    def _on_overlay_joint_pressed(self, joint_index: int, direction: int) -> None:
        if not self.robot_model.has_configuration:
            return
        self._overlay_jog_timer.stop()
        self._overlay_active_joint = (joint_index, direction)
        self._overlay_active_cartesian = None
        self._jog_overlay_joint(joint_index, direction * self._overlay_jog_delta * 0.1)
        self._overlay_jog_timer.start()

    def _on_overlay_joint_released(self, joint_index: int, _direction: int) -> None:
        if self._overlay_active_joint and self._overlay_active_joint[0] == joint_index:
            self._overlay_jog_timer.stop()
            self._overlay_active_joint = None

    def _on_overlay_cartesian_pressed(self, axis_index: int, direction: int) -> None:
        if not self.robot_model.has_configuration:
            return
        self._overlay_jog_timer.stop()
        self._overlay_active_cartesian = (axis_index, direction)
        self._overlay_active_joint = None
        self._jog_overlay_cartesian(
            axis_index,
            direction * self._overlay_jog_delta * (1.0 if axis_index < 3 else 0.1),
        )
        self._overlay_jog_timer.start()

    def _on_overlay_cartesian_released(self, axis_index: int, _direction: int) -> None:
        if self._overlay_active_cartesian and self._overlay_active_cartesian[0] == axis_index:
            self._overlay_jog_timer.stop()
            self._overlay_active_cartesian = None

    def _on_overlay_jog_tick(self) -> None:
        if self._overlay_active_joint is not None:
            joint_index, direction = self._overlay_active_joint
            self._jog_overlay_joint(joint_index, direction * self._overlay_jog_delta * 0.1)
            return
        if self._overlay_active_cartesian is not None:
            axis_index, direction = self._overlay_active_cartesian
            self._jog_overlay_cartesian(
                axis_index,
                direction * self._overlay_jog_delta * (1.0 if axis_index < 3 else 0.1),
            )

    def _jog_overlay_joint(self, joint_index: int, delta: float) -> None:
        current_value = self.robot_model.get_joint(joint_index)
        min_limit, max_limit = self.robot_model.get_axis_limit(joint_index)
        new_value = max(min_limit, min(max_limit, current_value + delta))
        if new_value != current_value:
            self.robot_model.set_joint(joint_index, new_value)

    def _jog_overlay_cartesian(self, axis_index: int, delta: float) -> None:
        try:
            widget = self.viewer_3d_widget.get_overlay_cartesian_widget()
            reference_frame = ReferenceFrame.from_value(widget.get_reference_frame())

            if reference_frame == ReferenceFrame.TOOL:
                current_tcp_pose = self.robot_model.get_tcp_pose()
                target = current_tcp_pose.copy()
                if axis_index < 3:
                    delta_pos = np.array([0.0, 0.0, 0.0], dtype=float)
                    delta_pos[axis_index] = delta
                    delta_in_base = self.robot_model.get_tcp_rotation_matrix() @ delta_pos
                    target = Pose6(
                        target.x + float(delta_in_base[0]),
                        target.y + float(delta_in_base[1]),
                        target.z + float(delta_in_base[2]),
                        target.a,
                        target.b,
                        target.c,
                    )
                else:
                    delta_rotation = (
                        math_utils.rot_z(delta)
                        if axis_index == 3
                        else (math_utils.rot_y(delta) if axis_index == 4 else math_utils.rot_x(delta))
                    )
                    new_tcp_rotation = self.robot_model.get_tcp_rotation_matrix() @ delta_rotation
                    new_abc = math_utils.rotation_matrix_to_euler_zyx(new_tcp_rotation)
                    target = Pose6(
                        target.x,
                        target.y,
                        target.z,
                        float(new_abc[0]),
                        float(new_abc[1]),
                        float(new_abc[2]),
                    )
                mgi_result = self.robot_model.compute_ik_target(target, tool=self.tool_model.get_tool())
                if not mgi_result:
                    return
                best_solution = self.robot_model.get_best_mgi_solution(mgi_result)
                if best_solution is None:
                    return
                self.robot_model.set_joints(best_solution[1].joints)
                widget.set_all_cartesian(self.robot_model.get_tcp_pose())
            else:
                axis_limits = widget.get_axis_limits()
                min_limit, max_limit = axis_limits[axis_index]
                current_value = widget.get_cartesian_value(axis_index)
                new_value = max(min_limit, min(max_limit, current_value + delta))
                if new_value == current_value:
                    return
                widget.spinboxes_cart[axis_index].setValue(new_value)
        except Exception as exc:
            print(f"Erreur lors du jog cartésien overlay: {exc}")

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
