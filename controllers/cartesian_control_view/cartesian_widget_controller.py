from PyQt6.QtCore import QEvent, QObject, pyqtSignal

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
        orientation_limits_deg: tuple[float, float] = (-180.0, 180.0),
        parent: QObject = None,
    ):
        super().__init__(parent)

        self.robot_model = robot_model
        self.workspace_model = workspace_model
        self.cartesian_control_widget = cartesian_control_widget
        self._orientation_limits_deg: tuple[float, float] = (
            float(orientation_limits_deg[0]),
            float(orientation_limits_deg[1]),
        )
        self.new_target = Pose6.zeros()
        self._limits_cache_key: tuple | None = None
        self._limits_cache_value: list[tuple[float, float]] | None = None
        self._last_display_pose: Pose6 = Pose6.zeros()
        self._last_reference_frame: str = self.cartesian_control_widget.get_reference_frame()
        self._tool_anchor_display_pose: Pose6 = Pose6.zeros()
        self._tool_anchor_tcp_pose: Pose6 = Pose6.zeros()
        self.cartesian_control_widget.installEventFilter(self)
        self._setup_connections()
        self._apply_cartesian_slider_limits()


    def eventFilter(self, obj, event) -> bool:
        if obj is self.cartesian_control_widget and event.type() == QEvent.Type.Show:
            self.force_refresh()
        return super().eventFilter(obj, event)

    def force_refresh(self) -> None:
        tcp_pose_base = self.robot_model.get_tcp_pose()
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()
        display_pose = convert_pose_from_base_frame(
            tcp_pose_base,
            ReferenceFrame.from_value(self.cartesian_control_widget.get_reference_frame()),
            robot_base_transform,
        )
        self._last_display_pose = display_pose.copy()
        self._last_reference_frame = self.cartesian_control_widget.get_reference_frame()
        self.cartesian_control_widget.set_all_cartesian(display_pose)

    def _setup_connections(self):
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_changed)
        self.robot_model.cartesian_slider_limits_changed.connect(self._apply_cartesian_slider_limits)
        self.workspace_model.workspace_changed.connect(self._on_workspace_changed)
        self.cartesian_control_widget.cartesian_value_changed.connect(self._on_view_cartesian_value_changed)
        self.cartesian_control_widget.reference_frame_changed.connect(self._on_reference_frame_changed)

    def _on_workspace_changed(self) -> None:
        self._apply_cartesian_slider_limits()
        self._on_model_tcp_changed()

    def _on_model_tcp_changed(self) -> None:
        if not self.cartesian_control_widget.isVisible():
            return
        self.force_refresh()

    def _on_view_cartesian_value_changed(self, idx: int, value: float) -> None:
        if idx < 0 or idx >= 6:
            return
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()
        displayed_pose = convert_pose_from_base_frame(
            self.robot_model.get_tcp_pose(),
            ReferenceFrame.from_value(self.cartesian_control_widget.get_reference_frame()),
            robot_base_transform,
        )
        # B (Ry, index 4) est borné à [-90, 90] : plage réelle de la décomposition
        # ZYX. En saisie absolue, on s'arrête net au mur du gimbal lock plutôt que de
        # commander |B|>90 (qui se replierait en inversant A/C -> oscillation).
        if idx == 4:
            value = max(-90.0, min(90.0, value))
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
        next_frame = ReferenceFrame.from_value(self.cartesian_control_widget.get_reference_frame())
        previous_frame = ReferenceFrame.from_value(self._last_reference_frame)
        if next_frame == ReferenceFrame.TOOL and previous_frame != ReferenceFrame.TOOL:
            tcp_pose_base = self.robot_model.get_tcp_pose().copy()
            self._tool_anchor_display_pose = tcp_pose_base.copy()
            self._tool_anchor_tcp_pose = tcp_pose_base.copy()
        self._apply_cartesian_slider_limits()
        self._on_model_tcp_changed()

    def _apply_cartesian_slider_limits(self) -> None:
        xyz_limits = self._get_display_cartesian_slider_limits_xyz()
        # A (Rz) et C (Rx) gardent la plage configurée ; B (Ry) est borné à [-90, 90]
        # car la décomposition ZYX ne renvoie jamais |B|>90 (mur du gimbal lock).
        a_limits = self._orientation_limits_deg
        b_limits = (
            max(self._orientation_limits_deg[0], -90.0),
            min(self._orientation_limits_deg[1], 90.0),
        )
        c_limits = self._orientation_limits_deg
        self.cartesian_control_widget.update_axis_limits(
            list(xyz_limits[:3]) + [a_limits, b_limits, c_limits]
        )

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
