from PyQt6.QtCore import QObject

from controllers.cartesian_control_view.cartesian_widget_controller import CartesianWidgetController
from controllers.mgi_controller import MgiController
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from views.cartesian_control_view import CartesianControlView


class CartesianControlController(QObject):
    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        cartesian_control_view: CartesianControlView,
        mgi_controller: MgiController,
        parent: QObject | None = None,
    ):
        super().__init__(parent)
        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.cartesian_control_view = cartesian_control_view
        self.mgi_controller = mgi_controller

        self.cartesian_widget_controller = CartesianWidgetController(
            self.robot_model,
            self.workspace_model,
            self.cartesian_control_view.get_cartesian_control_widget(),
        )

        self._setup_connections()

    def _setup_connections(self) -> None:
        self.cartesian_widget_controller.new_target_computed.connect(self._on_cartesian_new_target_computed)

    def _on_cartesian_new_target_computed(self) -> None:
        target = self.cartesian_widget_controller.get_new_target()

        mgi_result = self.robot_model.compute_ik_target(target, tool=self.tool_model.get_tool())
        best_sol = self.robot_model.get_best_mgi_solution(mgi_result)

        if not best_sol:
            self.mgi_controller.display_mgi_result(mgi_result, None)
            return

        _config_key, sol_analytique = best_sol

        if self.mgi_controller.is_jacobien_enabled():
            params = self.mgi_controller.get_jacobien_params()
            jacobien_result = self.robot_model.compute_ik_optimise(
                target,
                sol_analytique.joints,
                params,
                tool=self.tool_model.get_tool(),
            )
            jacobien_result.joints_analytiques = list(sol_analytique.joints)
            self.robot_model.set_joints(jacobien_result.joints)
            self.mgi_controller.set_jacobien_resultat(jacobien_result)
            return

        self.robot_model.set_joints(sol_analytique.joints)
