from PyQt5.QtWidgets import QMainWindow, QTabWidget, QWidget
from PyQt5.QtCore import pyqtSignal

from RobotTab.cartesiancontrol import CartesianControlWindow
from RobotTab.robotwindow import RobotWindow
from RobotTab.jointcontrol import JointControlWindow
from RobotTab.robotmodel import RobotModel

class MainWindow(QMainWindow):
    """Fenêtre principale avec système d'onglets pour les différentes fonctionnalités"""
    on_tab_changed = pyqtSignal(int)

    def __init__(self, robot_model: RobotModel, parent: QWidget = None):
        super().__init__(parent)
        self.setWindowTitle("MGD Robot Compensator")
        self.resize(2000, 1100)
        
        # ====================================================================
        # RÉGION: Initialisation du système d'onglets
        # ====================================================================
        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)
        
        # Créer les fenêtres pour chaque onglet
        self.robot_model = robot_model

        LEFT_STRECTH = 40
        RIGHT_STRECTH = 100 - LEFT_STRECTH

        self.joint_control = JointControlWindow(robot_model, LEFT_STRECTH, RIGHT_STRECTH)
        self.cartesian_control = CartesianControlWindow(robot_model, LEFT_STRECTH, RIGHT_STRECTH)
        self.robot_window = RobotWindow(robot_model, self, self.joint_control, self.cartesian_control, LEFT_STRECTH, RIGHT_STRECTH)
        
        # ====================================================================
        # RÉGION: Configuration des onglets
        # ====================================================================
        self._setup_tabs()

        self._setup_connections()
    
    def _setup_tabs(self) -> None:
        """Configure les onglets de la fenêtre principale"""
        # Onglet Robot (configuration et contrôle)
        self.tab_widget.addTab(self.robot_window, "Robot")
        self.tab_widget.addTab(self.joint_control, "Contrôle articulaire")
        self.tab_widget.addTab(self.cartesian_control, "Contrôle cartésien")

        self.tab_widget.setTabEnabled(1, False)  # Désactive l'onglet de contrôle articulaire
        self.tab_widget.setTabEnabled(2, False)  # Désactive l'onglet de contrôle cartésien

        # Les prochains onglets seront ajoutés ici à l'avenir
        # self.tab_widget.addTab(self.calibration_window, "Calibration")
        # self.tab_widget.addTab(self.analysis_window, "Analyse")

    def get_robot_window(self) -> RobotWindow:
        """Retourne la fenêtre de configuration du robot"""
        return self.robot_window
    
    def _setup_connections(self) -> None:
        """Configure les connexions de signaux entre les composants de la fenêtre principale"""
        self.tab_widget.currentChanged.connect(self.on_tab_changed.emit)
        self.robot_model.configuration_changed.connect(self._on_configuration_changed)
    
    def _on_configuration_changed(self):
        """Gère les changements de configuration du robot"""
        # Activer les onglets de contrôle lorsque la configuration change
        self.tab_widget.setTabEnabled(1, True)  # Active l'onglet de contrôle articulaire
        self.tab_widget.setTabEnabled(2, True)  # Active l'onglet de contrôle cartésien