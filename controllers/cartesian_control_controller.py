from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from views.cartesian_control_view import CartesianControlView
from controllers.cartesian_control_view.cartesian_wdiget_controller import CartesianWidgetController
from controllers.cartesian_control_view.mgi_solutions_controller import MgiSolutionsController
from controllers.correction_table_controller import CorrectionTableController


class CartesianControlController(QObject):
    def __init__(self, robot_model: RobotModel, cartesian_control_view: CartesianControlView, parent: QObject = None):
        super().__init__(parent)
        
        self.robot_model = robot_model
        self.cartesian_control_view = cartesian_control_view

        self.cartesian_widget_controller = CartesianWidgetController(self.robot_model, self.cartesian_control_view.get_cartesian_control_widget())
        self.mgi_solutions_controller = MgiSolutionsController(self.robot_model, self.cartesian_control_view.get_mgi_solutions_widget())
        self.correction_table_controller = CorrectionTableController(robot_model, self.cartesian_control_view.get_correction_widget())

        self._setup_connections()

    def _setup_connections(self):
        self.cartesian_widget_controller.new_target_computed.connect(self._on_cartesian_new_target_computed)

    def _on_cartesian_new_target_computed(self):
        target = self.cartesian_widget_controller.get_new_target()
        mgi_result = self.robot_model.compute_ik_target(target)
        best_sol = self.robot_model.get_best_mgi_solution(mgi_result)

        if best_sol:
            self.robot_model.set_joints(best_sol[1].joints)
        else:
            self.mgi_solutions_controller.display_mgi_result(mgi_result, None)
    
