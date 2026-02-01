from PyQt5.QtCore import QObject

from models.robot_model import RobotModel
from widgets.cartesian_control_view.mgi_solutions_widget import MgiSolutionsWidget
from mgi import MgiConfigKey, MgiResultStatus


class MgiSolutionsController(QObject):

    def __init__(self, robot_model: RobotModel, mgi_solutions_widget: MgiSolutionsWidget, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.mgi_solutions_widget = mgi_solutions_widget

        self.mgi_solutions_widget.set_axis_limits(self.robot_model.get_axis_limits())

        self._setup_connection()

    def _setup_connection(self):
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_pose_changed)
        self.robot_model.axis_limits_changed.connect(self._on_model_axis_limits_changed)

        self.mgi_solutions_widget.solution_selected.connect(self._on_view_solution_selected)
        self.mgi_solutions_widget.allowed_configs_changed.connect(self._on_view_allowed_configs_changed)

    def _on_model_tcp_pose_changed(self):
        self.mgi_solutions_widget.set_mgi_result(self.robot_model.get_current_tcp_mgi_result(), self.robot_model.get_current_axis_config())

    def _on_model_axis_limits_changed(self):
        self.mgi_solutions_widget.set_axis_limits(self.robot_model.get_axis_limits())

    def _on_view_solution_selected(self, config_key: MgiConfigKey):
        all_sol = self.robot_model.get_current_tcp_mgi_result()
        sol = all_sol.get_solution(config_key)
        if sol.status == MgiResultStatus.VALID and sol.joints:
            self.robot_model.set_joints(sol.joints)

    def _on_view_allowed_configs_changed(self):
        # TODO: update configs filter
        pass