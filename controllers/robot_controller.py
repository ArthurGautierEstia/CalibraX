from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QDialog

from dialogs.axis_limits_dialog import AxisLimitsDialog
from dialogs.axis_positions_dialog import AxisPositionsDialog
from models.robot_model import RobotModel
from views.robot_view import RobotView
from controllers.robot_view.dh_table_controller import DHTableController
from controllers.robot_view.measurement_controller import MeasurementController


class RobotController(QObject):
    configuration_loaded = pyqtSignal()

    def __init__(self, robot_model: RobotModel, robot_view: RobotView, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.robot_view = robot_view

        self.dh_controller = DHTableController(self.robot_model, self.robot_view.get_dh_widget())
        self.measurement_controller = MeasurementController(self.robot_model, self.robot_view.get_measurement_widget())

        self._setup_connections()

    def _setup_connections(self) -> None:
        self.dh_controller.configuration_loaded.connect(self.configuration_loaded.emit)
        self.robot_view.get_dh_widget().axis_config_requested.connect(self._on_axis_config_requested)
        self.robot_view.get_dh_widget().positions_config_requested.connect(self._on_positions_config_requested)

    def _on_axis_config_requested(self) -> None:
        dialog = AxisLimitsDialog(
            self.robot_view,
            self.robot_model.get_axis_limits(),
            self.robot_model.get_axis_speed_limits(),
            self.robot_model.get_axis_jerk_limits(),
            self.robot_model.get_axis_reversed(),
        )
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return

        limits = dialog.get_limits()
        axis_speed_limits = dialog.get_axis_speed_limits()
        axis_jerk_limits = dialog.get_axis_jerk_limits()
        axis_reversed = dialog.get_axis_reversed()

        self.robot_model.inhibit_auto_compute_fk_tcp(True)
        self.robot_model.set_axis_speed_limits(axis_speed_limits)
        self.robot_model.set_axis_jerk_limits(axis_jerk_limits)
        self.robot_model.set_axis_limits(limits)
        self.robot_model.set_axis_reversed(axis_reversed)
        self.robot_model.inhibit_auto_compute_fk_tcp(False)
        self.robot_model.compute_fk_tcp()

    def _on_positions_config_requested(self) -> None:
        dialog = AxisPositionsDialog(
            self.robot_view,
            self.robot_model.get_home_position(),
            self.robot_model.get_position_zero(),
            self.robot_model.get_position_transport(),
        )
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return

        self.robot_model.set_position_zero(dialog.get_position_zero())
        self.robot_model.set_position_transport(dialog.get_position_transport())
        self.robot_model.set_home_position(dialog.get_home_position())
