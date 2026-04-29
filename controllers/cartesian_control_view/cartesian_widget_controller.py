from PyQt6.QtCore import QObject, pyqtSignal

from models.reference_frame import ReferenceFrame
from models.robot_model import RobotModel
from models.workspace_model import WorkspaceModel
from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget
import utils.math_utils as math_utils
from utils.reference_frame_utils import convert_pose_from_base_frame, convert_pose_to_base_frame


class CartesianWidgetController(QObject):

    new_target_computed = pyqtSignal()

    def __init__(
        self,
        robot_model: RobotModel,
        workspace_model: WorkspaceModel,
        cartesian_control_widget: CartesianControlWidget,
        parent: QObject = None,
    ):
        super().__init__(parent)

        self.robot_model = robot_model
        self.workspace_model = workspace_model
        self.cartesian_control_widget = cartesian_control_widget
        self.new_target = [0.0] * 6
        self._limits_cache_key: tuple | None = None
        self._limits_cache_value: list[tuple[float, float]] | None = None
        self._last_display_pose: list[float] = [0.0] * 6
        self._last_reference_frame: str = self.cartesian_control_widget.get_reference_frame()
        self._tool_anchor_display_pose: list[float] = [0.0] * 6
        self._tool_anchor_tcp_pose: list[float] = [0.0] * 6
        self._setup_connections()
        self._apply_cartesian_slider_limits()


    def _setup_connections(self):
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_changed)
        self.robot_model.cartesian_slider_limits_changed.connect(self._apply_cartesian_slider_limits)
        self.workspace_model.workspace_changed.connect(self._on_workspace_changed)
        self.cartesian_control_widget.cartesian_value_changed.connect(self._on_view_cartesian_value_changed)
        self.cartesian_control_widget.reference_frame_changed.connect(self._on_reference_frame_changed)

    def _on_workspace_changed(self) -> None:
        self._apply_cartesian_slider_limits()
        self._on_model_tcp_changed()

    def _on_model_tcp_changed(self):
        tcp_pose_base = self.robot_model.get_tcp_pose()
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()
        display_pose = self._convert_pose_from_base_frame(
            tcp_pose_base,
            self.cartesian_control_widget.get_reference_frame(),
            robot_base_transform,
        )
        self._last_display_pose = list(display_pose)
        self._last_reference_frame = self.cartesian_control_widget.get_reference_frame()
        self.cartesian_control_widget.set_all_cartesian(display_pose)

    def _on_view_cartesian_value_changed(self, idx: int, value: float):
        if idx < 0 or idx >= 6:
            return
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()
        displayed_pose = self._convert_pose_from_base_frame(
            self.robot_model.get_tcp_pose(),
            self.cartesian_control_widget.get_reference_frame(),
            robot_base_transform,
        )
        displayed_pose[idx] = value
        self.new_target = self._convert_pose_to_base_frame(
            displayed_pose,
            self.cartesian_control_widget.get_reference_frame(),
            robot_base_transform,
        )
        self.new_target_computed.emit()

    def _on_reference_frame_changed(self, _reference_frame: str) -> None:
        next_frame = ReferenceFrame.from_value(self.cartesian_control_widget.get_reference_frame())
        previous_frame = ReferenceFrame.from_value(self._last_reference_frame)
        if next_frame == ReferenceFrame.TOOL and previous_frame != ReferenceFrame.TOOL:
            tcp_pose_base = list(self.robot_model.get_tcp_pose())
            self._tool_anchor_display_pose = tcp_pose_base
            self._tool_anchor_tcp_pose = tcp_pose_base
        self._apply_cartesian_slider_limits()
        self._on_model_tcp_changed()

    def _apply_cartesian_slider_limits(self) -> None:
        xyz_limits = self._get_display_cartesian_slider_limits_xyz()
        self.cartesian_control_widget.update_axis_limits(list(xyz_limits[:3]) + [(-180.0, 180.0)] * 3)

    def _get_display_cartesian_slider_limits_xyz(self) -> list[tuple[float, float]]:
        xyz_limits = [
            (float(min_val), float(max_val))
            for min_val, max_val in self.robot_model.get_cartesian_slider_limits_xyz()
        ]
        reference_frame = ReferenceFrame.from_value(self.cartesian_control_widget.get_reference_frame())
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()
        revision = robot_base_transform.revision if reference_frame == ReferenceFrame.WORLD else -1
        cache_key = (reference_frame.value, tuple(xyz_limits), revision)
        if cache_key == self._limits_cache_key and self._limits_cache_value is not None:
            return self._limits_cache_value

        if reference_frame == ReferenceFrame.WORLD:
            limits = math_utils.transform_xyz_limits_yaw_only(xyz_limits, robot_base_transform.pose)
        else:
            limits = xyz_limits

        self._limits_cache_key = cache_key
        self._limits_cache_value = limits
        return limits

    def _convert_pose_from_base_frame(
        self,
        pose_base: object,
        reference_frame: ReferenceFrame | str,
        robot_base_transform: object,
    ) -> list[float]:
        frame = ReferenceFrame.from_value(reference_frame)
        if frame == ReferenceFrame.TOOL:
            return list(self._tool_anchor_display_pose)
        return list(
            convert_pose_from_base_frame(
                pose_base,
                frame,
                robot_base_transform,
            )
        )

    def _convert_pose_to_base_frame(
        self,
        pose: object,
        reference_frame: ReferenceFrame | str,
        robot_base_transform: object,
    ) -> list[float]:
        frame = ReferenceFrame.from_value(reference_frame)
        if frame == ReferenceFrame.TOOL:
            relative_pose = [
                float(pose[idx]) - float(self._tool_anchor_display_pose[idx])
                for idx in range(6)
            ]
            return self.compose_tool_relative_target(relative_pose)
        return list(
            convert_pose_to_base_frame(
                pose,
                frame,
                robot_base_transform,
            )
        )

    def compose_tool_relative_target(self, relative_pose: object) -> list[float]:
        current_tcp_transform = math_utils.pose_zyx_to_matrix(self._tool_anchor_tcp_pose)
        relative_tool_transform = math_utils.pose_zyx_to_matrix(relative_pose)
        target_transform = current_tcp_transform @ relative_tool_transform
        return list(math_utils.matrix_to_pose_zyx(target_transform))
    
    def get_new_target(self) -> list[float]:
        return self.new_target
