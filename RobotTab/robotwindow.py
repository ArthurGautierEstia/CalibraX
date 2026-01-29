from PyQt5.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout
from RobotTab.cartesiancontrol import CartesianControlWindow
from RobotTab.jointcontrol import JointControlWindow
from RobotTab.widgets.dh_table_widget import DHTableWidget
from RobotTab.widgets.measurement_widget import MeasurementWidget
from RobotTab.widgets.viewer_3d_widget import Viewer3DWidget
from RobotTab.robotmodel import RobotModel
from RobotTab.robotcontroller import RobotController

class RobotWindow(QWidget):
    """Fenêtre de configuration et contrôle du robot avec son propre MVC"""
    
    def __init__(self, robot_model: RobotModel, main_window, joint_control: JointControlWindow, cartesian_control: CartesianControlWindow, leftStrect=1, rightStrect=2, parent: QWidget = None):
        super().__init__(parent)

        self.left_stretch = leftStrect
        self.right_stretch = rightStrect

        self.robot_model = robot_model
        
        # ====================================================================
        # RÉGION: Initialisation des widgets
        # ====================================================================
        self.dh_widget = DHTableWidget()
        self.measurement_widget = MeasurementWidget()

        self.joint_control_window = joint_control
        self.cartesian_control_window = cartesian_control
        
        self.viewer_widget = Viewer3DWidget()
        self.current_active_tab_index = 0
                
        # ====================================================================
        # RÉGION: Création du contrôleur
        # ====================================================================
        self.robot_controller = RobotController(
            self.robot_model,
            main_window,
            self.dh_widget,
            self.joint_control_window,
            self.cartesian_control_window,
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

        main_window.on_tab_changed.connect(self._transfert_3dviewer)
    
    def _setup_ui(self) -> None:
        """Configure l'interface utilisateur"""
        # Layout principal
        self._layout = QHBoxLayout(self)
        #self._layout.setContentsMargins(5, 5, 5, 5)
        #self._layout.setSpacing(10)
        
        # ====================================================================
        # RÉGION: Organisation des widgets
        # ====================================================================
        
        # Colonne gauche: Tables DH et Mesures
        left_layout = QVBoxLayout()
        left_layout.setSpacing(5)
        left_layout.addWidget(self.dh_widget)
        left_layout.addWidget(self.measurement_widget)
        
        # Assemblage
        self._layout.addLayout(left_layout, self.left_stretch)
        self._layout.addWidget(self.viewer_widget, self.right_stretch) # Afficheur 3D colone droite
    
    # ============================================================================
    # RÉGION: Getters
    # ============================================================================
    
    def get_robot_controller(self):
        return self.robot_controller

    def get_dh_widget(self):
        return self.dh_widget
    
    def get_measurement_widget(self):
        return self.measurement_widget
    
    def get_joint_widget(self):
        return self.joint_control_window
    
    def get_cartesian_widget(self):
        return self.cartesian_control_window
    
    def get_viewer_widget(self):
        return self.viewer_widget
    
    def get_robot_model(self):
        return self.robot_model
    
    def get_robot_controller(self):
        return self.robot_controller

    def add_viewer(self, viewer_widget):
        """Ajoute le widget de visualisation 3D au layout"""
        self._layout.addWidget(viewer_widget, self.right_stretch)
    
    def pick_viewer(self):
        """Retourne le widget de visualisation 3D"""
        return self._layout.itemAt(1).widget()

    def remove_viewer(self):
        """Retire le widget de visualisation 3D du layout"""
        viewer_widget = self.pick_viewer()
        if viewer_widget:
            self._layout.removeWidget(viewer_widget)
            viewer_widget.setParent(None)

    def _transfert_3dviewer(self, new_tab_index: int):
        if new_tab_index != self.current_active_tab_index:

            match new_tab_index:
                case 1:  # Joint tab
                    new_parent = self.joint_control_window
                case 2:  # Cartesian tab
                    new_parent = self.cartesian_control_window
                case _:  # Robot tab
                    new_parent = self
            
            match self.current_active_tab_index:
                case 1:  # Joint tab
                    old_parent = self.joint_control_window
                case 2:  # Cartesian tab
                    old_parent = self.cartesian_control_window
                case _:  # Robot tab
                    old_parent = self
            
            self.current_active_tab_index = new_tab_index
            old_parent.remove_viewer()
            new_parent.add_viewer(self.viewer_widget)

            self.viewer_widget.update_viewer()
