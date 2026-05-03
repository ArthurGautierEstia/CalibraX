from PyQt6.QtCore import QObject
import math

from models.robot_model import RobotModel
from utils.mgi import MgiConfigKey, MgiResult
from utils.mgi_jacobien import MgiJacobienResultat
from widgets.cartesian_control_view.mgi_solutions_widget import MgiSolutionsWidget


class MgiSolutionsController(QObject):
    def __init__(
        self,
        robot_model: RobotModel,
        mgi_solutions_widget: MgiSolutionsWidget,
        parent: QObject | None = None,
    ):
        super().__init__(parent)
        self.robot_model = robot_model
        self.mgi_solutions_widget = mgi_solutions_widget

        self.mgi_solutions_widget.set_axis_limits(self.robot_model.get_axis_limits())
        self.mgi_solutions_widget.set_weights(self.robot_model.get_joint_weights())

        self._setup_connection()

    def _setup_connection(self) -> None:
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_pose_changed)
        self.robot_model.axis_limits_changed.connect(self._on_model_axis_limits_changed)
        self.robot_model.joint_weights_changed.connect(self._on_model_joint_weights_changed)
        self.mgi_solutions_widget.solution_item_selected.connect(self._on_view_solution_item_selected)

    def _on_model_tcp_pose_changed(self) -> None:
        self.mgi_solutions_widget.set_mgi_result(
            self.robot_model.get_current_tcp_mgi_result(),
            self.robot_model.get_current_axis_config(),
            selected_joints=self.robot_model.get_joints(),
        )

    def _on_model_axis_limits_changed(self) -> None:
        self.mgi_solutions_widget.set_axis_limits(self.robot_model.get_axis_limits())

    def _on_model_joint_weights_changed(self) -> None:
        self.mgi_solutions_widget.set_weights(self.robot_model.get_joint_weights())

    def _on_view_solution_item_selected(self, config_key: MgiConfigKey, joints: list[float]) -> None:
        if joints and len(joints) >= 6:
            self.robot_model.set_joints([float(value) for value in joints[:6]])
            return
        self._on_view_solution_selected(config_key)

    def _on_view_solution_selected(self, config_key: MgiConfigKey) -> None:
        all_sol = self.robot_model.get_current_tcp_mgi_result()
        current_joints_rad = [math.radians(float(value)) for value in self.robot_model.get_joints()[:6]]
        while len(current_joints_rad) < 6:
            current_joints_rad.append(0.0)

        best = all_sol.get_best_solution_from_current(
            current_joints_rad,
            self.robot_model.get_joint_weights(),
            allowed_configs={config_key},
        )
        if best is None:
            return
        _, solution = best
        if solution.joints:
            self.robot_model.set_joints(solution.joints)

    def display_mgi_result(self, mgi_result: MgiResult, selected_key: MgiConfigKey | None) -> None:
        self.mgi_solutions_widget.set_mgi_result(
            mgi_result,
            selected_key,
            selected_joints=self.robot_model.get_joints(),
        )

    def afficher_resultat_jacobien(self, resultat: MgiJacobienResultat | None) -> None:
        self.mgi_solutions_widget.set_jacobien_resultat(resultat)
