from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from views.joint_control_view import JointControlView
from controllers.joint_control_view.joints_controller import JointsController
from controllers.joint_control_view.joints_result_controller import JointsResultController


class JointControlController(QObject):
    def __init__(self, robot_model: RobotModel, joint_control_view: JointControlView, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.joint_control_view = joint_control_view

        self.joints_controller = JointsController(robot_model, self.joint_control_view.get_joints_widget())
        self.joints_result_controller = JointsResultController(robot_model, self.joint_control_view.get_joints_result_widget())
