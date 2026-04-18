import math

from PyQt6.QtCore import QObject, pyqtSignal

from models.reference_frame import ReferenceFrame
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
        self.workspace_model.workspace_changed.connect(self._on_workspace_changed)
        self.cartesian_control_widget.cartesian_value_changed.connect(self._on_view_cartesian_value_changed)
        self.cartesian_control_widget.reference_frame_changed.connect(self._on_reference_frame_changed)

    def _on_workspace_changed(self) -> None:
        self._apply_cartesian_slider_limits()
        self._on_model_tcp_changed()

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
        self._apply_cartesian_slider_limits()
        self._on_model_tcp_changed()

    def _apply_cartesian_slider_limits(self) -> None:
        xyz_limits = self._get_display_cartesian_slider_limits_xyz()
        self.cartesian_control_widget.update_axis_limits(list(xyz_limits[:3]) + [(-180.0, 180.0)] * 3)

    def _get_display_cartesian_slider_limits_xyz(self) -> list[tuple[float, float]]:
        xyz_limits = self.robot_model.get_cartesian_slider_limits_xyz()
        if ReferenceFrame.from_value(self.cartesian_control_widget.get_reference_frame()) != ReferenceFrame.WORLD:
            return xyz_limits
        return self._transform_xyz_limits_base_to_world(
            xyz_limits,
            self.workspace_model.get_robot_base_pose_world(),
        )

    @staticmethod
    def _transform_xyz_limits_base_to_world(
        xyz_limits: list[tuple[float, float]],
        robot_base_pose_world: list[float],
    ) -> list[tuple[float, float]]:
        pose = [float(robot_base_pose_world[idx]) if idx < len(robot_base_pose_world) else 0.0 for idx in range(6)]
        tx, ty, tz = pose[:3]
        rz_deg = pose[3]  # Pose XYZABC projet: A est la rotation autour de Z.
        rz_rad = math.radians(rz_deg)
        cos_rz = math.cos(rz_rad)
        sin_rz = math.sin(rz_rad)

        x_min, x_max = xyz_limits[0]
        y_min, y_max = xyz_limits[1]
        z_min, z_max = xyz_limits[2]

        xy_world_points = []
        for x in (float(x_min), float(x_max)):
            for y in (float(y_min), float(y_max)):
                xy_world_points.append(
                    (
                        tx + x * cos_rz - y * sin_rz,
                        ty + x * sin_rz + y * cos_rz,
                    )
                )

        world_x = [point[0] for point in xy_world_points]
        world_y = [point[1] for point in xy_world_points]
        return [
            (min(world_x), max(world_x)),
            (min(world_y), max(world_y)),
            (tz + float(z_min), tz + float(z_max)),
        ]
    
    def get_new_target(self) -> list[float]:
        return self.new_target
