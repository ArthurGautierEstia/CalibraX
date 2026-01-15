from PyQt5.QtCore import QObject, pyqtSignal
from typing import List, Tuple

class RobotModel(QObject):
    """Modèle centralisé contenant tous les paramètres et l'état du robot"""
    
    # ============================================================================
    # SIGNAUX
    # ============================================================================
    
    # Configuration générale
    configuration_changed = pyqtSignal()
    robot_name_changed = pyqtSignal(str)
    
    # Paramètres DH
    dh_params_changed = pyqtSignal()
    
    # Joints et axes
    joints_changed = pyqtSignal()
    axis_reversed_changed = pyqtSignal()
    limits_changed = pyqtSignal()
    home_position_changed = pyqtSignal()
    
    # Corrections
    corrections_changed = pyqtSignal()
    
    # Résultats (TCP et cinématique)
    tcp_pose_changed = pyqtSignal()
    corrected_tcp_pose_changed = pyqtSignal()
    pose_deviation_changed = pyqtSignal()
    
    # Mesures
    measurements_changed = pyqtSignal()
    measurement_points_changed = pyqtSignal()
    
    def __init__(self, parent: QObject = None):
        super().__init__(parent)
        
        # ====================================================================
        # RÉGION: Configuration générale
        # ====================================================================
        self.robot_name = ""
        self.current_config_file = None
        
        # ====================================================================
        # RÉGION: Paramètres des joints et axes
        # ====================================================================
        # Limites des axes (min, max) pour chaque joint
        self.axis_limits: List[Tuple[float, float]] = [(-180, 180) for _ in range(6)]
        
        # Position home du robot
        self.home_position: List[float] = [0, -90, 90, 0, 90, 0]
        
        # Valeurs actuelles des joints (en degrés)
        self.joint_values: List[float] = [0, 0, 0, 0, 0, 0]
        
        # Valeurs réelles des joints (appliquant les multiplicateurs d'inversion)
        self.reel_joint_values: List[float] = [0, 0, 0, 0, 0, 0]
        
        # Multiplicateurs d'axes (1 = normal, -1 = inversé)
        self.axis_reversed: List[int] = [1, 1, 1, 1, 1, 1]
        
        # ====================================================================
        # RÉGION: Paramètres Denavit-Hartenberg
        # ====================================================================
        # 7 lignes : 6 joints + outil (tool frame)
        # Chaque ligne contient [a, alpha, d, theta]
        self.dh_params: List[List[float]] = [[0, 0, 0, 0] for _ in range(7)]
        
        # ====================================================================
        # RÉGION: Corrections 6D
        # ====================================================================
        # 6 lignes pour 6 joints, 6 colonnes pour 6 DDL (X, Y, Z, Rx, Ry, Rz)
        self.corrections: List[List[float]] = [[0, 0, 0, 0, 0, 0] for _ in range(6)]
        
        # ====================================================================
        # RÉGION: Résultats cinématique
        # ====================================================================
        # Pose TCP non corrigée [X, Y, Z, Rx, Ry, Rz]
        self.tcp_pose: List[float] = [0, 0, 0, 0, 0, 0]
        
        # Pose TCP corrigée (appliquant les corrections)
        self.corrected_tcp_pose: List[float] = [0, 0, 0, 0, 0, 0]
        
        # Déviation entre TCP et TCP corrigé
        self.pose_deviation: List[float] = [0, 0, 0, 0, 0, 0]
        
        # ====================================================================
        # RÉGION: Mesures et points de mesure
        # ====================================================================
        # Liste des mesures enregistrées
        self.measurements: List[float] = []
        
        # Points de mesure (positions de référence)
        self.measurement_points: List[List[float]] = []
    
    # ============================================================================
    # RÉGION: Getters - Configuration générale
    # ============================================================================
    
    def get_robot_name(self) -> str:
        """Retourne le nom du robot"""
        return self.robot_name
    
    def get_current_config_file(self) -> str:
        """Retourne le chemin du fichier de configuration actuel"""
        return self.current_config_file
    
    # ============================================================================
    # RÉGION: Setters - Configuration générale
    # ============================================================================
    
    def set_robot_name(self, name: str):
        """Définit le nom du robot"""
        self.robot_name = name
        self.robot_name_changed.emit(name)
    
    def set_current_config_file(self, file_path: str):
        """Définit le chemin du fichier de configuration actuel"""
        self.current_config_file = file_path
    
    # ============================================================================
    # RÉGION: Getters - Joints et axes
    # ============================================================================
    
    def get_joint_value(self, index: int) -> float:
        """Retourne la valeur d'un joint spécifique ou 0 si l'index est invalide"""
        return self.joint_values[index] if 0 <= index < 6 else 0
    
    def get_all_joint_values(self):
        """Retourne toutes les valeurs des joints"""
        return self.joint_values.copy()
    
    def get_reel_joint_value(self, index):
        """Retourne la valeur réelle d'un joint (avec inversion appliquée)"""
        return self.reel_joint_values[index] if 0 <= index < 6 else 0
    
    def get_all_reel_joint_values(self):
        """Retourne toutes les valeurs réelles des joints"""
        return self.reel_joint_values.copy()
    
    def get_axis_limits(self):
        """Retourne les limites de tous les axes"""
        return self.axis_limits.copy()
    
    def get_axis_limit(self, index: int):
        """Retourne les limites d'un axe spécifique (min, max)"""
        return self.axis_limits[index] if 0 <= index < 6 else (-180, 180)
    
    def get_home_position(self):
        """Retourne la position home"""
        return self.home_position.copy()
    
    def get_axis_reversed(self):
        """Retourne les multiplicateurs d'axes"""
        return self.axis_reversed.copy()
    
    def is_axis_reversed(self, index: int) -> bool:
        """Retourne True si l'axe est inversé"""
        return self.axis_reversed[index] == -1 if 0 <= index < 6 else False
    
    # ============================================================================
    # RÉGION: Setters - Joints et axes
    # ============================================================================
    
    def set_joint_value(self, index: int, value: float):
        """Modifie la valeur d'un joint spécifique"""
        if 0 <= index < 6:
            self.joint_values[index] = float(value)
            self.reel_joint_values[index] = float(value) * self.axis_reversed[index]
            self.joints_changed.emit()
    
    def set_all_joint_values(self, values: list[float]):
        """Définit toutes les valeurs des joints"""
        if len(values) >= 6:
            self.joint_values = list(values[:6])
            self.reel_joint_values = [
                self.joint_values[i] * self.axis_reversed[i] for i in range(6)
            ]
            self.joints_changed.emit()
    
    def set_axis_limits(self, limits: List[Tuple[float, float]]):
        """Définit les limites de tous les axes"""
        self.axis_limits = limits
        self.limits_changed.emit()
    
    def set_axis_limit(self, index: int, min_val: float, max_val: float):
        """Définit les limites d'un axe spécifique"""
        if 0 <= index < 6:
            self.axis_limits[index] = (min_val, max_val)
            self.limits_changed.emit()
    
    def set_home_position(self, home_pos: list[float]):
        """Définit la position home"""
        if len(home_pos) >= 6:
            self.home_position = list(home_pos[:6])
            self.home_position_changed.emit()
    
    def set_axis_reversed(self, axis_reversed: list[int]):
        """Définit les multiplicateurs d'axes (1 ou -1)"""
        if len(axis_reversed) >= 6:
            self.axis_reversed = list(axis_reversed[:6])
            # Recalculer les valeurs réelles
            self.reel_joint_values = [
                self.joint_values[i] * self.axis_reversed[i] for i in range(6)
            ]
            self.axis_reversed_changed.emit()
    
    def set_axis_reversed_single(self, index: int, reversed_value: bool):
        """Inverse un axe spécifique"""
        if 0 <= index < 6:
            self.axis_reversed[index] = -1 if reversed_value else 1
            self.reel_joint_values[index] = self.joint_values[index] * self.axis_reversed[index]
            self.axis_reversed_changed.emit()
    
    # ============================================================================
    # RÉGION: Getters - Paramètres DH
    # ============================================================================
    
    def get_dh_params(self):
        """Retourne tous les paramètres DH"""
        return [row.copy() for row in self.dh_params]
    
    def get_dh_param(self, row: int, col: int):
        """Retourne un paramètre DH spécifique"""
        if 0 <= row < 7 and 0 <= col < 4:
            return self.dh_params[row][col]
        return 0
    
    def get_dh_row(self, row: int):
        """Retourne une ligne complète de paramètres DH"""
        if 0 <= row < 7:
            return self.dh_params[row].copy()
        return [0, 0, 0, 0]
    
    # ============================================================================
    # RÉGION: Setters - Paramètres DH
    # ============================================================================
    
    def set_dh_params(self, params: list[list[float]]):
        """Définit tous les paramètres DH"""
        self.dh_params = [list(row) for row in params]
        # Assurer 7 lignes
        while len(self.dh_params) < 7:
            self.dh_params.append([0, 0, 0, 0])
        self.dh_params = self.dh_params[:7]
        self.dh_params_changed.emit()
    
    def set_dh_param(self, row: int, col: int, value: float):
        """Définit un paramètre DH spécifique"""
        if 0 <= row < 7 and 0 <= col < 4:
            try:
                self.dh_params[row][col] = float(value)
                self.dh_params_changed.emit()
            except (ValueError, TypeError):
                print(f"Erreur: valeur DH invalide [{row},{col}] = {value}")
        else:
            print(f"Erreur: index DH invalide [{row},{col}]")
    
    def set_dh_row(self, row: int, values: list[float]):
        """Définit une ligne complète de paramètres DH"""
        if 0 <= row < 7 and len(values) >= 4:
            try:
                self.dh_params[row] = [float(v) for v in values[:4]]
                self.dh_params_changed.emit()
            except (ValueError, TypeError):
                print(f"Erreur: valeurs DH invalides pour la ligne {row}")
    
    # ============================================================================
    # RÉGION: Getters - Corrections
    # ============================================================================
    
    def get_corrections(self):
        """Retourne toutes les corrections 6D"""
        return [row.copy() for row in self.corrections]
    
    def get_correction(self, row: int, col: int):
        """Retourne une correction 6D spécifique"""
        return self.corrections[row][col] if 0 <= row < 6 and 0 <= col < 6 else 0
    
    def get_correction_row(self, row: int):
        """Retourne une ligne complète de corrections"""
        return self.corrections[row].copy() if 0 <= row < 6 else [0, 0, 0, 0, 0, 0]
    
    def get_correction_joint(self, joint_index: int):
        """Retourne le vecteur de correction 6D pour un joint"""
        return self.corrections[joint_index].copy() if 0 <= joint_index < 6 else [0, 0, 0, 0, 0, 0]
    
    # ============================================================================
    # RÉGION: Setters - Corrections
    # ============================================================================
    
    def set_corrections(self, corrections: list[list[float]]):
        """Définit toutes les corrections 6D"""
        self.corrections = [list(row) for row in corrections]
        # Assurer 6 lignes x 6 colonnes
        while len(self.corrections) < 6:
            self.corrections.append([0, 0, 0, 0, 0, 0])
        self.corrections = [row[:6] + [0]*(6-len(row)) for row in self.corrections[:6]]
        self.corrections_changed.emit()
    
    def set_correction(self, row: int, col: int, value: float):
        """Définit une correction 6D spécifique"""
        if 0 <= row < 6 and 0 <= col < 6:
            try:
                self.corrections[row][col] = float(value)
                self.corrections_changed.emit()
            except (ValueError, TypeError):
                print(f"Erreur: correction invalide [{row},{col}] = {value}")
        else:
            print(f"Erreur: index de correction invalide [{row},{col}]")
    
    def set_correction_row(self, row: int, values: list[float]):
        """Définit une ligne complète de corrections"""
        if 0 <= row < 6 and len(values) >= 6:
            try:
                self.corrections[row] = [float(v) for v in values[:6]]
                self.corrections_changed.emit()
            except (ValueError, TypeError):
                print(f"Erreur: valeurs de correction invalides pour la ligne {row}")
    
    # ============================================================================
    # RÉGION: Getters - Résultats cinématique
    # ============================================================================
    
    def get_tcp_pose(self):
        """Retourne la pose TCP non corrigée"""
        return self.tcp_pose.copy()
    
    def get_corrected_tcp_pose(self):
        """Retourne la pose TCP corrigée"""
        return self.corrected_tcp_pose.copy()
    
    def get_pose_deviation(self):
        """Retourne la déviation entre TCP et TCP corrigé"""
        return self.pose_deviation.copy()
    
    def get_tcp_position(self):
        """Retourne la position (X, Y, Z) du TCP"""
        return self.tcp_pose[:3]
    
    def get_tcp_rotation(self):
        """Retourne la rotation (Rx, Ry, Rz) du TCP"""
        return self.tcp_pose[3:6]
    
    # ============================================================================
    # RÉGION: Setters - Résultats cinématique
    # ============================================================================
    
    def set_tcp_pose(self, pose: list[float]):
        """Définit la pose TCP non corrigée"""
        if len(pose) >= 6:
            self.tcp_pose = list(pose[:6])
            self._compute_deviation()
            self.tcp_pose_changed.emit()
    
    def set_corrected_tcp_pose(self, pose: list[float]):
        """Définit la pose TCP corrigée"""
        if len(pose) >= 6:
            self.corrected_tcp_pose = list(pose[:6])
            self._compute_deviation()
            self.corrected_tcp_pose_changed.emit()
    
    def _compute_deviation(self):
        """Calcule la déviation entre TCP et TCP corrigé"""
        self.pose_deviation = [
            self.corrected_tcp_pose[i] - self.tcp_pose[i] for i in range(6)
        ]
        self.pose_deviation_changed.emit()
    
    # ============================================================================
    # RÉGION: Getters - Mesures
    # ============================================================================
    
    def get_measurements(self):
        """Retourne la liste des mesures enregistrées"""
        return self.measurements.copy()
    
    def get_measurement(self, index: int):
        """Retourne une mesure spécifique"""
        return self.measurements[index] if 0 <= index < len(self.measurements) else None
    
    def get_measurement_count(self):
        """Retourne le nombre de mesures enregistrées"""
        return len(self.measurements)
    
    def get_measurement_points(self):
        """Retourne la liste des points de mesure"""
        return self.measurement_points.copy()
    
    def get_measurement_point(self, index):
        """Retourne un point de mesure spécifique"""
        return self.measurement_points[index] if 0 <= index < len(self.measurement_points) else  None
    
    # ============================================================================
    # RÉGION: Setters - Mesures
    # ============================================================================
    
    def add_measurement(self, measurement: float):
        """Ajoute une nouvelle mesure"""
        self.measurements.append(measurement)
        self.measurements_changed.emit()
    
    def add_measurement_point(self, point: list[float]):
        """Ajoute un nouveau point de mesure"""
        self.measurement_points.append(point)
        self.measurement_points_changed.emit()
    
    def clear_measurements(self):
        """Efface toutes les mesures"""
        self.measurements.clear()
        self.measurements_changed.emit()
    
    def clear_measurement_points(self):
        """Efface tous les points de mesure"""
        self.measurement_points.clear()
        self.measurement_points_changed.emit()
    
    def set_measurements(self, measurements: list[float]):
        """Définit la liste des mesures"""
        self.measurements = list(measurements)
        self.measurements_changed.emit()
    
    def set_measurement_points(self, points: list[list[float]]):
        """Définit la liste des points de mesure"""
        self.measurement_points = list(points)
        self.measurement_points_changed.emit()
    
    # ============================================================================
    # RÉGION: Sérialisation / Désérialisation
    # ============================================================================
    
    def to_dict(self):
        """Export vers dictionnaire (pour sauvegarde JSON)"""
        return {
            "name": [self.robot_name],
            "dh": [[str(val) for val in row] for row in self.dh_params[:6]],
            "corr": [[str(val) for val in row] for row in self.corrections],
            "q": self.joint_values,
            "axis_limits": self.axis_limits,
            "axis_reversed": self.axis_reversed,
            "home_position": self.home_position,
        }
    
    def load_from_dict(self, data: str, file_name: str=None):
        """Import depuis dictionnaire (pour chargement JSON)"""
        # Nom du robot
        if "name" in data and len(data["name"]) > 0:
            self.robot_name = data["name"][0]
        
        # Paramètres DH
        if "dh" in data:
            dh_list = [[float(val) if val else 0 for val in row] for row in data["dh"]]
            while len(dh_list) < 7:
                dh_list.append([0, 0, 0, 0])
            self.dh_params = dh_list[:7]
        
        # Corrections
        if "corr" in data:
            corr_list = [[float(val) if val else 0 for val in row] for row in data["corr"]]
            while len(corr_list) < 6:
                corr_list.append([0, 0, 0, 0, 0, 0])
            self.corrections = corr_list[:6]
        
        # Valeurs des joints
        if "q" in data:
            self.joint_values = list(data["q"][:6])
        
        # Multiplicateurs d'axes
        if "axis_reversed" in data:
            self.axis_reversed = list(data["axis_reversed"][:6])
        
        # Limites des axes
        if "axis_limits" in data:
            self.axis_limits = list(data["axis_limits"][:6])
        
        # Position home
        if "home_position" in data:
            self.home_position = list(data["home_position"][:6])
        
        # Recalculer les valeurs réelles
        self.reel_joint_values = [
            self.joint_values[i] * self.axis_reversed[i] for i in range(6)
        ]
        
        # Fichier de configuration
        if file_name:
            self.current_config_file = file_name
        
        # Émettre le signal de configuration changée
        self.configuration_changed.emit()