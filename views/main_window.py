from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout
)
from views.widgets.dh_table_widget import DHTableWidget
from views.widgets.joint_control_widget import JointControlWidget
from views.widgets.measurement_widget import MeasurementWidget
from views.widgets.result_table_widget import ResultTableWidget
from views.widgets.correction_table_widget import CorrectionTableWidget
from views.widgets.viewer_3d_widget import Viewer3DWidget

class MainWindow(QMainWindow):
    """Fenêtre principale de l'application"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MGD Robot Compensator")
        self.resize(2000, 1100)
        
        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout principal
        main_layout = QHBoxLayout(central_widget)
        
        # Créer les widgets
        self.dh_widget = DHTableWidget()
        self.measurement_widget = MeasurementWidget()
        self.joint_widget = JointControlWidget()
        self.result_widget = ResultTableWidget()
        self.correction_widget = CorrectionTableWidget()
        self.viewer_widget = Viewer3DWidget()
        
        # Organiser les widgets
        self._setup_layout(main_layout)
    
    def _setup_layout(self, main_layout):
        """Configure le layout de la fenêtre"""
        # Colonne gauche: Tables DH et Mesures
        left_layout = QVBoxLayout()
        left_layout.addWidget(self.dh_widget)
        left_layout.addWidget(self.measurement_widget)
        main_layout.addLayout(left_layout)
        
        # Colonne centrale: Contrôle joints + Résultats + Corrections
        center_layout = QVBoxLayout()
        center_layout.addWidget(self.joint_widget)
        center_layout.addWidget(self.result_widget)
        center_layout.addWidget(self.correction_widget)
        main_layout.addLayout(center_layout)
        
        # Colonne droite: Viewer 3D
        right_layout = QVBoxLayout()
        right_layout.addWidget(self.viewer_widget)
        main_layout.addLayout(right_layout)
    
    def get_dh_widget(self):
        """Retourne le widget de configuration DH"""
        return self.dh_widget
    
    def get_measurement_widget(self):
        """Retourne le widget de mesures"""
        return self.measurement_widget
    
    def get_joint_widget(self):
        """Retourne le widget de contrôle des joints"""
        return self.joint_widget
    
    def get_result_widget(self):
        """Retourne le widget de résultats"""
        return self.result_widget
    
    def get_correction_widget(self):
        """Retourne le widget de corrections"""
        return self.correction_widget
    
    def get_viewer_widget(self):
        """Retourne le widget de visualisation 3D"""
        return self.viewer_widget
