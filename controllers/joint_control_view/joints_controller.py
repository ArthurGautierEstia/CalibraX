from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QDialog

from dialogs.axis_limits_dialog import AxisLimitsDialog
from models.robot_model import RobotModel
from widgets.joint_control_view.joints_control_widget import JointsControlWidget


class JointsController(QObject):
    def __init__(self, robot_model: RobotModel, joint_control_widget: JointsControlWidget, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.joint_control_widget = joint_control_widget
        self._is_view_updating = False
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modèle du robot"""
        self.robot_model.configuration_changed.connect(self._on_model_config_changed)
        self.robot_model.joints_changed.connect(self._on_model_joints_changed)
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_changed)
        self.robot_model.axis_limits_changed.connect(self._on_model_axis_limits_change)

        self.joint_control_widget.joint_value_changed.connect(self._on_view_joint_value_changed)
        self.joint_control_widget.home_position_requested.connect(self._on_view_home_position_requested)
        self.joint_control_widget.axis_limits_config_requested.connect(self._on_view_axis_limits_config_requested)
        
    def _on_model_config_changed(self) -> None:
        """Callback quand le modèle du robot signale un changement de configuration"""
        self.joint_control_widget.set_all_joints(self.robot_model.get_joints())
        self.joint_control_widget.update_axis_limits(self.robot_model.get_axis_limits())

    def _on_model_joints_changed(self) -> None:
        """Callback quand le modèle du robot signale un changement de valeur de joint"""
        # Mettre à jour la vue
        self.joint_control_widget.set_all_joints(self.robot_model.get_joints())

    def _on_model_tcp_changed(self):
        self.joint_control_widget.set_configuration(self.robot_model.get_current_axis_config())

    def _on_model_axis_limits_change(self) -> None:
        self.joint_control_widget.update_axis_limits(self.robot_model.get_axis_limits())

    def _on_view_joint_value_changed(self, index: int, value: float) -> None:
        """Callback quand la vue signale un changement de valeur de joint"""
        # Mettre à jour le modèle du robot
        self.robot_model.set_joint(index, value)

    def _on_view_home_position_requested(self) -> None:
        self.robot_model.go_to_home_position()

    def _on_view_axis_limits_config_requested(self) -> None:
        """Callback: ouvrir la boîte de dialogue de configuration des limites"""
        dialog = AxisLimitsDialog(
            self.joint_control_widget,
            self.robot_model.get_axis_limits(),
            self.robot_model.get_home_position(),
            self.robot_model.get_axis_reversed()
        )
        
        if dialog.exec_() == QDialog.Accepted:
            # Récupérer les données du dialogue
            limits = dialog.get_limits()
            home_pos = dialog.get_home_position()
            axis_reversed = dialog.get_axis_reversed()

            # Mettre à jour le modèle
            self.robot_model.set_home_position(home_pos)
            self.robot_model.inhibit_auto_compute_fk_tcp(True)
            self.robot_model.set_axis_limits(limits)
            self.robot_model.set_axis_reversed(axis_reversed)
            self.robot_model.inhibit_auto_compute_fk_tcp(False)
            self.robot_model.compute_fk_tcp()
