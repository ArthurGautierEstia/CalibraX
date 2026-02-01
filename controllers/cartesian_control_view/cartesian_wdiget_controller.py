from PyQt5.QtCore import QObject

from models.robot_model import RobotModel
from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget


class CartesianWidgetController(QObject):

    def __init__(self, robot_model: RobotModel, cartesian_control_widget: CartesianControlWidget, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.cartesian_control_widget = cartesian_control_widget
        self._setup_connections()

    def _setup_connections(self):
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_changed)
        self.cartesian_control_widget.cartesian_value_changed.connect(self._on_view_cartesian_value_changed)

    def _on_model_tcp_changed(self):
        self.cartesian_control_widget.set_all_cartesian(self.robot_model.get_tcp_pose())

    def _on_view_cartesian_value_changed(self, idx: int, value: float):
        pass
    