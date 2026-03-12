from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QTabWidget, QSplitter

from models.robot_model import RobotModel
from widgets.viewer_3d_widget import Viewer3DWidget
from views.robot_view import RobotView
from views.calibration_view import CalibrationView
from views.joint_control_view import JointControlView
from views.cartesian_control_view import CartesianControlView
from views.jog_view import JogView
from views.trajectory_view import TrajectoryView

class MainWindow(QMainWindow):

    def __init__(self, robot_model: RobotModel, parent: QWidget=None):
        super().__init__(parent)
        self.setWindowTitle("Calibrax")

        self.tabs = QTabWidget()

        self.robot_view = RobotView()
        self.calibration_view = CalibrationView()
        self.joint_control_view = JointControlView()
        self.cartesian_control_view = CartesianControlView()
        self.jog_view = JogView()
        self.trajectory_view = TrajectoryView(robot_model)

        self.viewer3d = Viewer3DWidget()

        self._setup_ui()
    
    def _setup_ui(self) -> None:
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        self.tabs.addTab(self.robot_view, "Robot")
        self.tabs.addTab(self.calibration_view, "Calibration")
        self.tabs.addTab(self.joint_control_view, "Contrôle articulaire")
        self.tabs.addTab(self.cartesian_control_view, "Contrôle cartésien")
        self.tabs.addTab(self.jog_view, "Jog")
        self.tabs.addTab(self.trajectory_view, "Trajectoire")

        splitter = QSplitter(Qt.Orientation.Horizontal, central_widget)
        splitter.setHandleWidth(6)
        splitter.addWidget(self.tabs)
        splitter.addWidget(self.viewer3d)

        # Taille initiale (équivalent de tes "2" et "3")
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        splitter.setSizes([400, 600])  # optionnel : donne une taille de départ plus stable

        # Optionnel : empêcher que l'un des deux disparaisse complètement
        splitter.setChildrenCollapsible(False)

        layout = QVBoxLayout(central_widget)
        layout.addWidget(splitter)

    ####################
    # VIEW GETTERS
    ####################

    def get_robot_view(self) -> RobotView:
        """Retourne la vue de configuration du robot"""
        return self.robot_view
    
    def get_calibration_view(self) -> CalibrationView:
        """Retourne la vue de calibration du robot"""
        return self.calibration_view

    def get_joint_control_view(self) -> JointControlView:
        """Retourne la vue de contrôle articulaire"""
        return self.joint_control_view
    
    def get_cartesian_control_view(self) -> CartesianControlView:
        """Retourne la vue de contrôle cartésien"""
        return self.cartesian_control_view
    
    def get_jog_view(self) -> JogView:
        """Retourne la vue de jog"""
        return self.jog_view

    def get_trajectory_view(self) -> TrajectoryView:
        """Retourne la vue de trajectoire"""
        return self.trajectory_view
    
    def get_viewer3d(self) -> Viewer3DWidget:
        """Retourne la vue du viewer 3D"""
        return self.viewer3d

    #####################
    # Functions
    #####################

    def update_enabled_tabs(self, robot_has_configuration: bool) -> None:
        """Active ou désactive les onglets de contrôle en fonction de la configuration du robot"""
        self.tabs.setTabEnabled(2, robot_has_configuration)
        self.tabs.setTabEnabled(3, robot_has_configuration)
        self.tabs.setTabEnabled(4, robot_has_configuration)
        self.tabs.setTabEnabled(5, robot_has_configuration)
    
