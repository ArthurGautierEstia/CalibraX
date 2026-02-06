from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout

from widgets.jog_view.jog_cartestian_control_widget import JogCartesianControlWidget
from widgets.jog_view.jog_joint_control_widget import JogJointControlWidget
from widgets.jog_view.jog_angles_visualization_widget import JogAnglesVisualizationWidget
from widgets.jog_view.jog_tcp_visualization_widget import JogTCPVisualizationWidget


class JogView(QWidget):
    """Vue principale pour le contrôle Jog avec visualisation des angles et TCP"""
    
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        
        # Créer les widgets
        self.jog_cartesian_control_widget = JogCartesianControlWidget()
        self.jog_joint_control_widget = JogJointControlWidget()
        self.jog_angles_visualization_widget = JogAnglesVisualizationWidget()
        self.jog_tcp_visualization_widget = JogTCPVisualizationWidget()
        
        self._setup_ui()
    
    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur pour la vue Jog"""
        main_layout = QVBoxLayout(self)
        
        # ====================================================================
        # 1ER ÉTAGE : Jog Articulaire et Jog Cartésien
        # ====================================================================
        first_floor_layout = QHBoxLayout()
        first_floor_layout.addWidget(self.jog_joint_control_widget)
        first_floor_layout.addWidget(self.jog_cartesian_control_widget)
        
        # ====================================================================
        # 2ÈME ÉTAGE : Visualisation des angles et TCP
        # ====================================================================
        second_floor_layout = QHBoxLayout()
        second_floor_layout.addWidget(self.jog_angles_visualization_widget)
        second_floor_layout.addWidget(self.jog_tcp_visualization_widget)
        
        # Ajouter les deux étages au layout principal
        main_layout.addLayout(first_floor_layout)
        main_layout.addLayout(second_floor_layout)
        main_layout.addStretch()
    
    ####################
    # WIDGET GETTERS
    ####################

    def get_jog_joint_widget(self) -> JogJointControlWidget:
        """Retourne le widget de contrôle joint"""
        return self.jog_joint_control_widget

    def get_jog_cartesian_widget(self) -> JogCartesianControlWidget:
        """Retourne le widget de contrôle cartesian"""
        return self.jog_cartesian_control_widget
    
    def get_jog_angles_visualization_widget(self) -> JogAnglesVisualizationWidget:
        """Retourne le widget de visualisation des angles"""
        return self.jog_angles_visualization_widget
    
    def get_jog_tcp_visualization_widget(self) -> JogTCPVisualizationWidget:
        """Retourne le widget de visualisation du TCP"""
        return self.jog_tcp_visualization_widget
