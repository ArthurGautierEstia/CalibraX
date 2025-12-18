import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal

class RobotModel(QObject):
    """Modèle contenant les paramètres et l'état du robot"""
    
    # Signaux pour notifier les changements
    configuration_changed = pyqtSignal()
    joints_changed = pyqtSignal()
    limits_changed = pyqtSignal()
    axis_reversed_changed = pyqtSignal()
    dh_params_changed = pyqtSignal()
    corrections_changed = pyqtSignal()
    home_position_changed = pyqtSignal()
    robot_name_changed = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        
        # Paramètres du robot
        self.robot_name = ""
        self.axis_limits = [(-180, 180) for _ in range(6)]
        self.home_position = [0, -90, 90, 0, 90, 0]
        
        # État actuel des joints (degrés)
        self.joint_values = [0, 0, 0, 0, 0, 0]
        self.reel_joint_values = [0, 0, 0, 0, 0, 0]
        self.axis_reversed = [1, 1, 1, 1, 1, 1]  # 1 = normal, -1 = inversé
        
        # Paramètres DH (7 lignes: 6 joints + tool)
        self.dh_params = [[0, 0, 0, 0] for _ in range(7)]
        
        # Corrections 6D (6 lignes pour 6 joints)
        self.corrections = [[0, 0, 0, 0, 0, 0] for _ in range(6)]
        
        # Fichier de configuration actuel
        self.current_config_file = None
    
    def set_robot_name(self, name):
        """Définit le nom du robot"""
        self.robot_name = name
        self.robot_name_changed.emit(name)
    
    def set_axis_limits(self, limits):
        """Définit les limites des axes"""
        self.axis_limits = limits
        self.limits_changed.emit()
    
    def set_home_position(self, home_pos):
        """Définit la position home"""
        self.home_position = home_pos
        self.home_position_changed.emit()
    
    def set_joint_value(self, index, value):
        """Modifie la valeur d'un joint"""
        if 0 <= index < 6:
            self.joint_values[index] = value
            self.reel_joint_values[index] = value * self.axis_reversed[index]
            self.joints_changed.emit()
        
    def set_all_joints(self, values):
        """Définit toutes les valeurs de joints"""
        self.joint_values = values[:6]
        self.reel_joint_values = [values[i] * self.axis_reversed[i] for i in range(6)]
        self.joints_changed.emit()
    
    def set_reverse_axis(self, axis_reversed):
        """Définit les multiplicateurs d'axes (1 ou -1)"""
        self.axis_reversed = axis_reversed
        self.axis_reversed_changed.emit()
    
    def set_dh_params(self, params):
        """Définit les paramètres DH"""
        self.dh_params = params
        self.dh_params_changed.emit()
    
    def set_corrections(self, corrections):
        """Définit les corrections 6D"""
        self.corrections = corrections
        self.corrections_changed.emit()
    
    def get_dh_param(self, row, col):
        """Récupère un paramètre DH spécifique"""
        if 0 <= row < 7 and 0 <= col < 4:
            return self.dh_params[row][col]
        return 0
    
    def get_correction(self, row, col):
        """Récupère une correction 6D spécifique"""
        if 0 <= row < 6 and 0 <= col < 6:
            return self.corrections[row][col]
        return 0
    
    def to_dict(self):
        """Export vers dictionnaire (pour sauvegarde JSON)"""
        return {
            "dh": [[str(val) for val in row] for row in self.dh_params[:6]],
            "corr": [[str(val) for val in row] for row in self.corrections],
            "q": self.joint_values,
            "name": [self.robot_name],
            "axis_limits": self.axis_limits,
            "axis_reversed": self.axis_reversed,
            "home_position": self.home_position
        }
    
    def load_configuration(self, data, file_name):
        """Import depuis dictionnaire (pour chargement JSON)"""
        # Nom de la configuration
        self.robot_model.current_config_file = file_name

        # DH params
        if "dh" in data:
            self.dh_params = [[float(val) if val else 0 for val in row] for row in data["dh"]]
            # Ajouter la ligne 7 si nécessaire
            while len(self.dh_params) < 7:
                self.dh_params.append([0, 0, 0, 0])
        
        # Corrections
        if "corr" in data:
            self.corrections = [[float(val) if val else 0 for val in row] for row in data["corr"]]

        # Multiplicateurs d'axes
        if "axis_reversed" in data:
            self.axis_reversed = data["axis_reversed"]
        else:
            self.axis_reversed = [1, 1, 1, 1, 1, 1]
        
        # Joints
        if "q" in data:
            self.initial_joint_values = data["q"][:6]
        else:
            self.initial_joint_values = [0, 0, 0, 0, 0, 0]
        
        # Nom
        if "name" in data and len(data["name"]) > 0:
            self.robot_name = data["name"][0]
        
        # Limites
        if "axis_limits" in data:
            self.axis_limits = data["axis_limits"]
        
        
        # Home position
        if "home_position" in data:
            self.home_position = data["home_position"]
        else:
            self.home_position = [0, -90, 90, 0, 90, 0]
        
        self.configuration_changed.emit()
        
