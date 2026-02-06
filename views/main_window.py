from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QTabWidget

from widgets.viewer_3d_widget import Viewer3DWidget
from views.robot_view import RobotView
from views.joint_control_view import JointControlView
from views.cartesian_control_view import CartesianControlView
from views.jog_view import JogView

class MainWindow(QMainWindow):

    def __init__(self, parent: QWidget=None):
        super().__init__(parent)
        self.setWindowTitle("Calibrax")

        self.tabs = QTabWidget()

        self.robot_view = RobotView()
        self.joint_control_view = JointControlView()
        self.cartesian_control_view = CartesianControlView()
        self.jog_view = JogView()

        self.viewer3d = Viewer3DWidget()

        self._setup_ui()
    
    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur de la fenêtre principale"""

        central_widget = QWidget()
        
        self.tabs.addTab(self.robot_view, "Robot")
        self.tabs.addTab(self.joint_control_view, "Contrôle articulaire")
        self.tabs.addTab(self.cartesian_control_view, "Contrôle cartésien")
        self.tabs.addTab(self.jog_view, "Jog")

        layout = QHBoxLayout(central_widget)
        layout.addWidget(self.tabs, 2)
        layout.addWidget(self.viewer3d, 3)

        self.setCentralWidget(central_widget)

    ####################
    # VIEW GETTERS
    ####################

    def get_robot_view(self) -> RobotView:
        """Retourne la vue de configuration du robot"""
        return self.robot_view
    
    def get_joint_control_view(self) -> JointControlView:
        """Retourne la vue de contrôle articulaire"""
        return self.joint_control_view
    
    def get_cartesian_control_view(self) -> CartesianControlView:
        """Retourne la vue de contrôle cartésien"""
        return self.cartesian_control_view
    
    def get_jog_view(self) -> JogView:
        """Retourne la vue de jog"""
        return self.jog_view
    
    def get_viewer3d(self) -> Viewer3DWidget:
        """Retourne la vue du viewer 3D"""
        return self.viewer3d

    #####################
    # Functions
    #####################

    def update_enabled_tabs(self, robot_has_configuration: bool) -> None:
        """Active ou désactive les onglets de contrôle en fonction de la configuration du robot"""
        self.tabs.setTabEnabled(1, robot_has_configuration)
        self.tabs.setTabEnabled(2, robot_has_configuration)
        self.tabs.setTabEnabled(3, robot_has_configuration)
    