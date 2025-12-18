from PyQt5.QtCore import QObject
from PyQt5.QtWidgets import QDialog
from utils.file_io import FileIOHandler
from views.dialogs.axis_limits_dialog import AxisLimitsDialog
from PyQt5.QtCore import pyqtSignal

class RobotController(QObject):
    """Contrôleur pour la gestion de la configuration du robot"""

    def __init__(self, robot_model, kinematics_engine, dh_widget, correction_widget, visualization_controller):
        super().__init__()
        self.robot_model = robot_model
        self.kinematics_engine = kinematics_engine
        self.dh_widget = dh_widget
        self.correction_widget = correction_widget
        self.visualization_controller = visualization_controller
        self.file_io = FileIOHandler()
        
        # Connecter les signaux des widgets aux méthodes du contrôleur
        self.dh_widget.load_config_requested.connect(self.load_configuration)
        self.dh_widget.save_config_requested.connect(self.save_configuration)
        self.dh_widget.dh_value_changed.connect(self.on_dh_value_changed)
        self.correction_widget.correction_value_changed.connect(self.on_correction_value_changed)
        
        # Connecter les changements du modèle à la vue
        self.robot_model.configuration_changed.connect(self.update_view_from_model)
        self.robot_model.joints_changed.emit()  # Forcer une mise à jour initiale
    
    def on_dh_value_changed(self, row, col, value):
        """Callback quand une valeur DH change dans la vue"""
        try:
            parsed_value = float(value) if value else 0
            current_params = self.robot_model.dh_params
            current_params[row][col] = parsed_value
            self.robot_model.set_dh_params(current_params)
        except ValueError:
            print(f"Valeur DH invalide: {value}")
    
    def on_correction_value_changed(self, row, col, value):
        """Callback quand une correction change dans la vue"""
        try:
            parsed_value = float(value) if value else 0
            current_corr = self.robot_model.corrections
            current_corr[row][col] = parsed_value
            self.robot_model.set_corrections(current_corr)
        except ValueError:
            print(f"Valeur de correction invalide: {value}")
    
    def save_configuration(self):
        """Sauvegarde la configuration actuelle"""
        data = self.robot_model.to_dict()
        file_name = self.file_io.save_json(
            self.dh_widget, 
            "Sauvegarder configuration", 
            data
        )
        if file_name:
            self.robot_model.current_config_file = file_name
            print(f"Configuration sauvegardée: {file_name}")
    
    def load_configuration(self):
        """Charge une configuration depuis un fichier"""
        file_name, data = self.file_io.load_json(
            self.dh_widget,
            "Charger configuration"
        )

        

        if data:
            # Charger dans le modèle
            self.robot_model.from_dict(data)
            self.robot_model.current_config_file = file_name
            
            
            # Mettre à jour les vues
            self.update_view_from_model()
            
            print(f"Configuration chargée: {file_name}")
        
        if self.visualization_controller.cad_visible:
            self.visualization_controller.load_robot_cad()
    
    def update_view_from_model(self):
        """Met à jour la vue depuis le modèle"""
        # Mettre à jour le widget DH
        self.dh_widget.set_robot_name(self.robot_model.robot_name)
        self.dh_widget.set_dh_params(
            [[str(val) for val in row] for row in self.robot_model.dh_params]
        )
        
        # Mettre à jour le widget de corrections
        self.correction_widget.set_corrections(
            [[str(val) for val in row] for row in self.robot_model.corrections]
        )
        self.robot_model.set_axis_limits(self.robot_model.axis_limits)
        if self.visualization_controller.cad_loaded:
            self.robot_model.set_all_joints(self.robot_model.joint_values)
        else:
            self.robot_model.set_all_joints(self.robot_model.initial_joint_values)

class JointController(QObject):
    """Contrôleur pour la gestion des coordonnées articulaires"""
    axis_parameters_changed = pyqtSignal()
    
    def __init__(self, robot_model, joint_widget, visualization_controller):
        super().__init__()
        self.robot_model = robot_model
        self.joint_widget = joint_widget
        self.visualization_controller = visualization_controller
        
        # Connecter les signaux
        self.joint_widget.joint_value_changed.connect(self.on_joint_value_changed)
        self.joint_widget.home_position_requested.connect(self.apply_home_position)
        self.joint_widget.axis_limits_config_requested.connect(self.configure_axis_limits)
        
        # Connecter les changements du modèle à la vue
        self.robot_model.joints_changed.connect(self.update_view_from_model)
        self.robot_model.limits_changed.connect(self.update_limits_in_view)
        
        # Initialiser les limites dans la vue
        self.update_limits_in_view()
    
    def on_joint_value_changed(self, index, value):
        """Callback quand une valeur de joint change"""
        self.robot_model.set_joint_value(index, value)
    
    def apply_home_position(self):
        """Applique la position home"""
        self.robot_model.set_all_joints(self.robot_model.home_position)
        self.update_view_from_model()
    
    def configure_axis_limits(self):
        """Ouvre le dialogue de configuration des limites d'axes"""
        dialog = AxisLimitsDialog(
            self.joint_widget,
            self.robot_model.axis_limits,
            self.robot_model.home_position,
            self.robot_model.axis_reversed
        )
        
        if dialog.exec_() == QDialog.Accepted:
            # Récupérer les nouvelles limites et home position
            new_limits = dialog.get_limits()
            new_home = dialog.get_home_position()
            new_axis_reversed = dialog.get_axis_reversed()
            
            
            # Mettre à jour le modèle
            self.robot_model.set_axis_limits(new_limits)
            self.robot_model.set_home_position(new_home)
            self.robot_model.set_reverse_axis(new_axis_reversed)

            # Mettre à jour la vue
            self.visualization_controller.update_visualization()
    
    def update_view_from_model(self):
        """Met à jour la vue depuis le modèle"""
        self.joint_widget.set_all_joints(self.robot_model.joint_values)
    
    def update_limits_in_view(self):
        """Met à jour les limites dans la vue"""
        self.joint_widget.update_axis_limits(self.robot_model.axis_limits)


class ResultController(QObject):
    """Contrôleur pour la gestion des résultats (TCP)"""
    
    def __init__(self, kinematics_engine, result_widget):
        super().__init__()
        self.kinematics_engine = kinematics_engine
        self.result_widget = result_widget
        
        # Connecter les signaux
        self.kinematics_engine.kinematics_updated.connect(self.update_results)
        self.result_widget.jog_increment_requested.connect(self.on_jog_increment)
    
    def update_results(self):
        """Met à jour l'affichage des résultats"""
        tcp_pose = self.kinematics_engine.get_tcp_pose()
        corrected_tcp_pose = self.kinematics_engine.get_corrected_tcp_pose()
        deviations = self.kinematics_engine.get_pose_deviation()
        
        self.result_widget.update_results(tcp_pose, corrected_tcp_pose, deviations)
    
    def on_jog_increment(self, row, delta):
        """Incrémente/décrémente une valeur TCP (pour jog manuel)"""
        # Récupérer les valeurs actuelles
        value_tcp = self.result_widget.get_value(row, 0)
        value_tcp_corr = self.result_widget.get_value(row, 1)
        
        # Incrémenter
        value_tcp += delta
        value_tcp_corr += delta
        
        # Mettre à jour l'affichage
        self.result_widget.set_value(row, 0, value_tcp)
        self.result_widget.set_value(row, 1, value_tcp_corr)
