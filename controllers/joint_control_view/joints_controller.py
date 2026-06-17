from __future__ import annotations

from PyQt6.QtCore import QEvent, QObject
from typing import TYPE_CHECKING

from models.robot_model import RobotModel
from widgets.joint_control_view.joints_control_widget import JointsControlWidget

if TYPE_CHECKING:
    from models.external_axes_model import ExternalAxesModel


class JointsController(QObject):
    def __init__(
        self,
        robot_model: RobotModel,
        joint_control_widget: JointsControlWidget,
        external_axes_model: ExternalAxesModel | None = None,
        parent: QObject = None,
    ):
        super().__init__(parent)

        self.robot_model = robot_model
        self.joint_control_widget = joint_control_widget
        self.external_axes_model = external_axes_model
        self.joint_control_widget.installEventFilter(self)
        self._setup_connections()

    def eventFilter(self, obj, event) -> bool:
        if obj is self.joint_control_widget and event.type() == QEvent.Type.Show:
            self.force_refresh()
        return super().eventFilter(obj, event)

    def force_refresh(self) -> None:
        self.joint_control_widget.set_all_joints(self.robot_model.get_joints())
        self.joint_control_widget.set_configuration(self.robot_model.get_current_axis_config())

    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modele du robot"""
        self.robot_model.configuration_changed.connect(self._on_model_config_changed)
        self.robot_model.joints_changed.connect(self._on_model_joints_changed)
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_changed)
        self.robot_model.axis_limits_changed.connect(self._on_model_axis_limits_change)

        self.joint_control_widget.joint_value_changed.connect(self._on_view_joint_value_changed)
        self.joint_control_widget.home_position_requested.connect(self._on_view_home_position_requested)
        self.joint_control_widget.position_zero_requested.connect(self._on_view_position_zero_requested)
        self.joint_control_widget.position_calibration_requested.connect(self._on_view_position_calibration_requested)

    def _on_model_config_changed(self) -> None:
        """Callback quand le modele du robot signale un changement de configuration"""
        self.joint_control_widget.set_all_joints(self.robot_model.get_joints())
        self.joint_control_widget.update_axis_limits(self.robot_model.get_axis_limits())

    def _on_model_joints_changed(self) -> None:
        """Callback quand le modele du robot signale un changement de valeur de joint"""
        if not self.joint_control_widget.isVisible():
            return
        self.joint_control_widget.set_all_joints(self.robot_model.get_joints())

    def _on_model_tcp_changed(self):
        if not self.joint_control_widget.isVisible():
            return
        self.joint_control_widget.set_configuration(self.robot_model.get_current_axis_config())

    def _on_model_axis_limits_change(self) -> None:
        self.joint_control_widget.update_axis_limits(self.robot_model.get_axis_limits())

    def _on_view_joint_value_changed(self, index: int, value: float) -> None:
        """Callback quand la vue signale un changement de valeur de joint"""
        self.robot_model.set_joint(index, value)

    def _apply_ext_axes(self, values: list[float]) -> None:
        if self.external_axes_model is None or len(values) < 8:
            return
        axes = self.external_axes_model.get_axes()
        for i, axis in enumerate(axes[:2]):
            if axis.joints:
                self.external_axes_model.set_axis_joint_value(axis.id, 0, float(values[6 + i]))

    def _on_view_home_position_requested(self) -> None:
        self.robot_model.go_to_home_position()
        self._apply_ext_axes(self.robot_model.get_home_position())

    def _on_view_position_zero_requested(self) -> None:
        self.robot_model.go_to_position_zero()
        self._apply_ext_axes(self.robot_model.get_position_zero())

    def _on_view_position_calibration_requested(self) -> None:
        self.robot_model.go_to_position_calibration()
        self._apply_ext_axes(self.robot_model.get_position_calibration())
