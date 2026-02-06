from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from widgets.cartesian_control_view.mgi_solutions_widget import MgiSolutionsWidget
from mgi import MgiResult, MgiConfigKey, MgiResultStatus


class MgiSolutionsController(QObject):

    def __init__(self, robot_model: RobotModel, mgi_solutions_widget: MgiSolutionsWidget, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.mgi_solutions_widget = mgi_solutions_widget

        self.mgi_solutions_widget.set_axis_limits(self.robot_model.get_axis_limits())
        self.mgi_solutions_widget.set_weights(self.robot_model.get_joint_weights())

        self._set_widget_axis_from_model()
        self._is_updating_allowed_config_from_view = False

        self._setup_connection()

    def _setup_connection(self):
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_pose_changed)
        self.robot_model.axis_limits_changed.connect(self._on_model_axis_limits_changed)
        self.robot_model.allowed_config_changed.connect(self._on_model_allowed_configs_changed)
        self.robot_model.joint_weights_changed.connect(self._on_model_joint_weights_changed)

        self.mgi_solutions_widget.solution_selected.connect(self._on_view_solution_selected)
        self.mgi_solutions_widget.allowed_configs_changed.connect(self._on_view_allowed_configs_changed)
        self.mgi_solutions_widget.weights_changed.connect(self._on_view_weights_changed)

    def _on_model_tcp_pose_changed(self):
        self.mgi_solutions_widget.set_mgi_result(self.robot_model.get_current_tcp_mgi_result(), self.robot_model.get_current_axis_config())

    def _on_model_axis_limits_changed(self):
        self.mgi_solutions_widget.set_axis_limits(self.robot_model.get_axis_limits())
    
    def _on_model_allowed_configs_changed(self):
        if not self._is_updating_allowed_config_from_view:
            self._set_widget_axis_from_model()

    def _on_view_solution_selected(self, config_key: MgiConfigKey):
        all_sol = self.robot_model.get_current_tcp_mgi_result()
        sol = all_sol.get_solution(config_key)
        if sol.status == MgiResultStatus.VALID and sol.joints:
            self.robot_model.set_joints(sol.joints)

    def _on_view_allowed_configs_changed(self):
        self._is_updating_allowed_config_from_view = True
        self.robot_model.set_allowed_configurations(self.mgi_solutions_widget.get_allowed_configs())
        self._is_updating_allowed_config_from_view = False

    def _on_model_joint_weights_changed(self):
        weights = self.robot_model.get_joint_weights()
        self.mgi_solutions_widget.set_weights(weights)

    def _on_view_weights_changed(self, weights: list):
        self.robot_model.set_joint_weights(weights)

    def _set_widget_axis_from_model(self):
        """Synchronise le widget avec l'état actuel du modèle"""
        selector_widget = self.mgi_solutions_widget.get_config_selector()
        allowed = self.robot_model.get_allowed_configurations()
        
        # Bloquer les signaux pendant la mise à jour
        selector_widget.blockSignals(True)
        selector_widget.set_allowed_configurations(allowed)
        selector_widget.blockSignals(False)

    def display_mgi_result(self, mgi_result: MgiResult, selected_key: MgiConfigKey|None):
        """Met à jour le widget avec le résultat MGI actuel du modèle"""
        self.mgi_solutions_widget.set_mgi_result(mgi_result, selected_key)
    