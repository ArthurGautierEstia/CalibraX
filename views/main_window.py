from PyQt5.QtWidgets import QMainWindow, QTabWidget
from views.robotwindow import RobotWindow

class MainWindow(QMainWindow):
    """Fenêtre principale avec système d'onglets pour les différentes fonctionnalités"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MGD Robot Compensator")
        self.resize(2000, 1100)
        
        # ====================================================================
        # RÉGION: Initialisation du système d'onglets
        # ====================================================================
        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)
        
        # Créer les fenêtres pour chaque onglet
        self.robot_window = RobotWindow()
        
        # ====================================================================
        # RÉGION: Configuration des onglets
        # ====================================================================
        self._setup_tabs()
    
    def _setup_tabs(self):
        """Configure les onglets de la fenêtre principale"""
        # Onglet Robot (configuration et contrôle)
        self.tab_widget.addTab(self.robot_window, "Robot")
        
        # Les prochains onglets seront ajoutés ici à l'avenir
        # self.tab_widget.addTab(self.calibration_window, "Calibration")
        # self.tab_widget.addTab(self.analysis_window, "Analyse")
    
    # ============================================================================
    # RÉGION: Getters pour les fenêtres
    # ============================================================================
    
    def get_robot_window(self):
        """Retourne la fenêtre du robot"""
        return self.robot_window