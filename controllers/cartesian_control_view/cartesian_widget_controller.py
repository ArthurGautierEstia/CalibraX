from PyQt6.QtCore import QObject, pyqtSignal

from models.reference_frame import ReferenceFrame
from models.robot_model import RobotModel
from models.types import Pose6
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
        self.new_target = Pose6.zeros()
        self._limits_cache_key: tuple | None = None
        self._limits_cache_value: list[tuple[float, float]] | None = None
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
        display_pose = convert_pose_from_base_frame(
            tcp_pose_base,
            ReferenceFrame.from_value(self.cartesian_control_widget.get_reference_frame()),
            robot_base_transform,
        )
        self.cartesian_control_widget.set_all_cartesian(display_pose.to_list())

    def _on_view_cartesian_value_changed(self, idx: int, value: float):
        if idx < 0 or idx >= 6:
            return
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()
        displayed_pose = convert_pose_from_base_frame(
            self.robot_model.get_tcp_pose(),
            ReferenceFrame.from_value(self.cartesian_control_widget.get_reference_frame()),
            robot_base_transform,
        )
        displayed_values = displayed_pose.to_list()
        displayed_values[idx] = value
        displayed_pose = Pose6(*displayed_values)
        self.new_target = convert_pose_to_base_frame(
            displayed_pose,
            ReferenceFrame.from_value(self.cartesian_control_widget.get_reference_frame()),
            robot_base_transform,
        )
        self.new_target_computed.emit()

    def _on_reference_frame_changed(self, _reference_frame: str) -> None:
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
    
    def get_new_target(self) -> Pose6:
        return self.new_target.copy()
