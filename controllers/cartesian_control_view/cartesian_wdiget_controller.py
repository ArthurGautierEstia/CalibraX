from PyQt6.QtCore import QObject, pyqtSignal

from models.robot_model import RobotModel
from models.workspace_model import WorkspaceModel
from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget
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
        self._setup_connections()
        self._apply_cartesian_slider_limits()


    def _setup_connections(self):
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_changed)
        self.robot_model.cartesian_slider_limits_changed.connect(self._apply_cartesian_slider_limits)
        self.workspace_model.workspace_changed.connect(self._on_model_tcp_changed)
        self.cartesian_control_widget.cartesian_value_changed.connect(self._on_view_cartesian_value_changed)
        self.cartesian_control_widget.reference_frame_changed.connect(self._on_reference_frame_changed)

    def _on_model_tcp_changed(self):
        tcp_pose_base = self.robot_model.get_tcp_pose()
        display_pose = convert_pose_from_base_frame(
            tcp_pose_base,
            self.cartesian_control_widget.get_reference_frame(),
            self.workspace_model.get_robot_base_pose_world(),
        )
        self.cartesian_control_widget.set_all_cartesian(display_pose)

    def _on_view_cartesian_value_changed(self, idx: int, value: float):
        if idx < 0 or idx >= 6:
            return
        displayed_pose = convert_pose_from_base_frame(
            self.robot_model.get_tcp_pose(),
            self.cartesian_control_widget.get_reference_frame(),
            self.workspace_model.get_robot_base_pose_world(),
        )
        displayed_pose[idx] = value
        self.new_target = convert_pose_to_base_frame(
            displayed_pose,
            self.cartesian_control_widget.get_reference_frame(),
            self.workspace_model.get_robot_base_pose_world(),
        )
        self.new_target_computed.emit()

    def _on_reference_frame_changed(self, _reference_frame: str) -> None:
        self._on_model_tcp_changed()

    def _apply_cartesian_slider_limits(self) -> None:
        xyz_limits = self.robot_model.get_cartesian_slider_limits_xyz()
        self.cartesian_control_widget.update_axis_limits(list(xyz_limits[:3]) + [(-180.0, 180.0)] * 3)
    
    def get_new_target(self) -> list[float]:
        return self.new_target
