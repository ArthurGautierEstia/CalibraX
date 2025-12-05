import sys
from PyQt5.QtWidgets import QApplication
from views.main_window import MainWindow
from models.robot_model import RobotModel
from models.kinematics_engine import KinematicsEngine
from models.measurement_model import MeasurementModel
from models.correction_model import CorrectionModel
from controllers.robot_controller import RobotController, JointController, ResultController
from controllers.visualization_controller import VisualizationController
from controllers.measurement_controller import MeasurementController

class MGDApplication:
    """Classe principale de l'application"""
    
    def __init__(self):
        # Créer l'application Qt
        self.app = QApplication(sys.argv)
        
        # Charger le thème
        self.load_theme()
        
        # Créer la fenêtre principale
        self.window = MainWindow()
        
        # Créer les modèles
        self.robot_model = RobotModel()
        self.kinematics_engine = KinematicsEngine(self.robot_model)
        self.measurement_model = MeasurementModel()
        self.correction_model = CorrectionModel(
            self.robot_model,
            self.kinematics_engine,
            self.measurement_model
        )
        
        # Créer les contrôleurs
        self.setup_controllers()
        
        # Connecter les signaux additionnels
        self.setup_additional_connections()
    
    def load_theme(self):
        """Charge le thème de l'application"""
        try:
            with open("dark_theme.qss", "r") as f:
                self.app.setStyleSheet(f.read())
        except FileNotFoundError:
            print("Fichier dark_theme.qss non trouvé, thème par défaut utilisé")
    
    def setup_controllers(self):
        """Initialise tous les contrôleurs"""
        # Contrôleur de configuration robot
        self.robot_controller = RobotController(
            self.robot_model,
            self.kinematics_engine,
            self.window.get_dh_widget(),
            self.window.get_correction_widget()
        )
        
        # Contrôleur des joints
        self.joint_controller = JointController(
            self.robot_model,
            self.window.get_joint_widget()
        )
        
        # Contrôleur des résultats
        self.result_controller = ResultController(
            self.kinematics_engine,
            self.window.get_result_widget()
        )
        
        # Contrôleur de visualisation
        self.visualization_controller = VisualizationController(
            self.kinematics_engine,
            self.robot_model,
            self.window.get_viewer_widget()
        )
        
        # Contrôleur de mesures
        self.measurement_controller = MeasurementController(
            self.measurement_model,
            self.correction_model,
            self.window.get_measurement_widget()
        )
    
    def setup_additional_connections(self):
        """Configure les connexions additionnelles entre composants"""
        # Connecter le mode pas à pas
        self.window.get_joint_widget().step_by_step_requested.connect(
            self.visualization_controller.toggle_step_mode
        )
        
        # Connecter la visibilité du CAD
        self.window.get_dh_widget().cad_toggled.connect(
            self.visualization_controller.toggle_cad_visibility
        )
        
        # Recharger le CAD quand la configuration change
        self.robot_model.configuration_changed.connect(
            self.on_configuration_changed
        )
        
        # Calculer la cinématique au démarrage
        self.kinematics_engine.compute_forward_kinematics()
    
    def on_configuration_changed(self):
        """Callback quand la configuration change"""
        # Recharger le CAD si une nouvelle config est chargée
        if self.robot_model.current_config_file:
            self.visualization_controller.reload_robot_cad()
    
    def run(self):
        """Lance l'application"""
        self.window.showMaximized()
        self.window.show()
        sys.exit(self.app.exec_())


if __name__ == "__main__":
    app = MGDApplication()
    app.run()
