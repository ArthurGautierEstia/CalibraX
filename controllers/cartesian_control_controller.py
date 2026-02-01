from PyQt5.QtCore import QObject

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
        self.mig_solution_controller = MgiSolutionsController(self.robot_model, self.cartesian_control_view.get_mgi_solutions_widget())
        self.correction_table_controller = CorrectionTableController(robot_model, self.cartesian_control_view.get_correction_widget())
