from PyQt6.QtCore import QObject, pyqtSignal

from models.robot_model import RobotModel
from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget


class CartesianWidgetController(QObject):

    new_target_computed = pyqtSignal()

    def __init__(self, robot_model: RobotModel, cartesian_control_widget: CartesianControlWidget, parent: QObject = None):
        super().__init__(parent)

        self.robot_model = robot_model
        self.cartesian_control_widget = cartesian_control_widget
        self.new_target = [0.0] * 6
        self._setup_connections()


    def _setup_connections(self):
        self.robot_model.tcp_pose_changed.connect(self._on_model_tcp_changed)
        self.cartesian_control_widget.cartesian_value_changed.connect(self._on_view_cartesian_value_changed)

    def _on_model_tcp_changed(self):
        self.cartesian_control_widget.set_all_cartesian(self.robot_model.get_tcp_pose())

    def _on_view_cartesian_value_changed(self, idx: int, value: float):
        if idx < 0 or idx >= 6:
            return
        self.new_target = list(self.robot_model.get_tcp_pose())
        self.new_target[idx] = value
        self.new_target_computed.emit()
    
    def get_new_target(self) -> list[float]:
        return self.new_target
