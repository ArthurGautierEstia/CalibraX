from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QTabWidget
from RobotTab.widgets.dh_table_widget import DHTableWidget
from RobotTab.widgets.joint_control_widget import JointControlWidget
from RobotTab.widgets.cartesian_control_widget import CartesianControlWidget
from RobotTab.widgets.measurement_widget import MeasurementWidget
from RobotTab.widgets.joints_result_table_widget import JointsResultTableWidget
from RobotTab.widgets.correction_table_widget import CorrectionTableWidget
from RobotTab.widgets.viewer_3d_widget import Viewer3DWidget
from RobotTab.widgets.mgi_solutions_widget import MgiSolutionsWidget
from RobotTab.robotmodel import RobotModel
from RobotTab.robotcontroller import RobotController

class RobotWindow(QWidget):
    """Fenêtre de configuration et contrôle du robot avec son propre MVC"""
    
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        
        # ====================================================================
        # RÉGION: Création du modèle
        # ====================================================================
        self.robot_model = RobotModel()

        # ====================================================================
        # RÉGION: Initialisation des widgets
        # ====================================================================
        self.dh_widget = DHTableWidget()
        self.measurement_widget = MeasurementWidget()
        self.joint_widget = JointControlWidget(self.robot_model)
        self.joints_result_widget = JointsResultTableWidget()
        self.cartesian_widget = CartesianControlWidget()
        self.mgi_solutions_widget = MgiSolutionsWidget()
        self.correction_widget = CorrectionTableWidget()
        self.viewer_widget = Viewer3DWidget()
        
        # ====================================================================
        # RÉGION: Création du TabWidget pour les contrôles
        # ====================================================================
        self.control_tabs: QTabWidget = QTabWidget()

        joint_tab_layout = QVBoxLayout()
        joint_tab_layout.addWidget(self.joint_widget)
        joint_tab_layout.addWidget(self.joints_result_widget)
        joint_tab_layout.addStretch()

        joint_tab_widget = QWidget()
        joint_tab_widget.setLayout(joint_tab_layout)

        cartesian_tab_layout = QVBoxLayout()
        cartesian_tab_layout.addWidget(self.cartesian_widget)
        cartesian_tab_layout.addWidget(self.mgi_solutions_widget)
        cartesian_tab_layout.addStretch()

        cartesian_tab_widget = QWidget()
        cartesian_tab_widget.setLayout(cartesian_tab_layout)

        self.control_tabs.setStyleSheet("border-radius: 8px;")
        self.control_tabs.addTab(joint_tab_widget, "Contrôle articulaire")
        self.control_tabs.addTab(cartesian_tab_widget, "Contrôle cartésien")
                
        # ====================================================================
        # RÉGION: Création du contrôleur
        # ====================================================================
        self.robot_controller = RobotController(
            self.robot_model,
            self.dh_widget,
            self.correction_widget,
            self.joint_widget,
            self.joints_result_widget,
            self.cartesian_widget,
            self.mgi_solutions_widget,
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
    
    def _setup_ui(self) -> None:
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
        
        # Colonne centrale: Tabs de contrôle (Joints/Cartésien) + Résultats + Corrections
        center_layout = QVBoxLayout()
        center_layout.setSpacing(5)
        center_layout.addWidget(self.control_tabs)
        
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
    
    def get_dh_widget(self) -> DHTableWidget:
        return self.dh_widget
    
    def get_measurement_widget(self) -> MeasurementWidget:
        return self.measurement_widget
    
    def get_joint_widget(self) -> JointControlWidget:
        return self.joint_widget
    
    def get_cartesian_widget(self) -> CartesianControlWidget:
        return self.cartesian_widget
    
    def get_control_tabs(self) -> QTabWidget:
        return self.control_tabs
    
    def get_result_widget(self) -> JointsResultTableWidget:
        return self.joints_result_widget
    
    def get_correction_widget(self) -> CorrectionTableWidget:
        return self.correction_widget
    
    def get_viewer_widget(self) -> Viewer3DWidget:
        return self.viewer_widget
    
    def get_robot_model(self) -> RobotModel:
        return self.robot_model
    
    def get_robot_controller(self) -> RobotController:
        return self.robot_controller
