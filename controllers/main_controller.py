from PyQt5.QtCore import QObject

from models.robot_model import RobotModel
from controllers.robot_controller import RobotController
from controllers.joint_control_controller import JointControlController
from controllers.cartesian_control_controller import CartesianControlController
from controllers.jog_controller import JogController
from controllers.viewed3d_controller import Viewed3DController
from views.main_window import MainWindow


class MainController(QObject):
    def __init__(self, robot_model: RobotModel, main_window: MainWindow, parent: QObject = None):
        super().__init__(parent)
        
        self.robot_model = robot_model
        self.main_window = main_window

        # controllers
        self.robot_controller = RobotController(robot_model, main_window.get_robot_view())
        self.joint_control_controller = JointControlController(robot_model, main_window.get_joint_control_view())
        self.cartesian_control_controller = CartesianControlController(robot_model, main_window.get_cartesian_control_view())
        self.jog_controller = JogController(robot_model, main_window.get_jog_view())
        self.viewed3d_controller = Viewed3DController(robot_model, main_window.get_viewer3d())

        self._on_robot_model_config_changed()  # initial update

        self._setup_connections()

    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre la vue et le modèle du robot"""
        self.robot_model.configuration_changed.connect(self._on_robot_model_config_changed)

        self.robot_controller.configuration_loaded.connect(self._on_config_loaded)

    def _on_robot_model_config_changed(self) -> None:
        self.main_window.update_enabled_tabs(self.robot_model.get_has_confifiguration())
    
    def _on_config_loaded(self) -> None:
        self.main_window.viewer3d.load_cad(self.robot_model)
        self.main_window.viewer3d.set_transparency(True)
    