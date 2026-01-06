from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout
from views.widgets.dh_table_widget import DHTableWidget
from views.widgets.joint_control_widget import JointControlWidget
from views.widgets.measurement_widget import MeasurementWidget
from views.widgets.result_table_widget import ResultTableWidget
from views.widgets.correction_table_widget import CorrectionTableWidget
from views.widgets.viewer_3d_widget import Viewer3DWidget
from robotmodel import RobotModel
from robotcontroller import RobotController

class RobotWindow(QWidget):
    """Fenêtre de configuration et contrôle du robot avec son propre MVC"""
    
    def __init__(self):
        super().__init__()
        
        # ====================================================================
        # RÉGION: Initialisation des widgets
        # ====================================================================
        self.dh_widget = DHTableWidget()
        self.measurement_widget = MeasurementWidget()
        self.joint_widget = JointControlWidget()
        self.result_widget = ResultTableWidget()
        self.correction_widget = CorrectionTableWidget()
        self.viewer_widget = Viewer3DWidget()
        
        # ====================================================================
        # RÉGION: Création du modèle
        # ====================================================================
        self.robot_model = RobotModel()
        
        # ====================================================================
        # RÉGION: Création du contrôleur
        # ====================================================================
        self.robot_controller = RobotController(
            self.robot_model,
            self.dh_widget,
            self.correction_widget,
            self.joint_widget,
            self.result_widget,
            self.measurement_widget,
            self.viewer_widget
        )
        
        # ====================================================================
        # RÉGION: Configuration de l'interface
        # ====================================================================
        self._setup_ui()
        
        # ====================================================================
        # RÉGION: Configuration des connexions de signaux
        # ====================================================================
        self.robot_controller.setup_connections()
    
    def _setup_ui(self):
        """Configure l'interface utilisateur"""
        # Layout principal
        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(10)
        
        # ====================================================================
        # RÉGION: Organisation des widgets
        # ====================================================================
        
        # Colonne gauche: Tables DH et Mesures
        left_layout = QVBoxLayout()
        left_layout.setSpacing(5)
        left_layout.addWidget(self.dh_widget)
        left_layout.addWidget(self.measurement_widget)
        main_layout.addLayout(left_layout, 1)
        
        # Colonne centrale: Contrôle joints + Résultats + Corrections
        center_layout = QVBoxLayout()
        center_layout.setSpacing(5)
        center_layout.addWidget(self.joint_widget)
        center_layout.addWidget(self.result_widget)
        center_layout.addWidget(self.correction_widget)
        main_layout.addLayout(center_layout, 1)
        
        # Colonne droite: Viewer 3D
        right_layout = QVBoxLayout()
        right_layout.setSpacing(5)
        right_layout.addWidget(self.viewer_widget)
        main_layout.addLayout(right_layout, 1)
    
    # ============================================================================
    # RÉGION: Getters
    # ============================================================================
    
    def get_dh_widget(self):
        return self.dh_widget
    
    def get_measurement_widget(self):
        return self.measurement_widget
    
    def get_joint_widget(self):
        return self.joint_widget
    
    def get_result_widget(self):
        return self.result_widget
    
    def get_correction_widget(self):
        return self.correction_widget
    
    def get_viewer_widget(self):
        return self.viewer_widget
    
    def get_robot_model(self):
        return self.robot_model
    
    def get_robot_controller(self):
        return self.robot_controller