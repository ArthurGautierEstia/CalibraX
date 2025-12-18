import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal
from utils.math_utils import euler_to_rotation_matrix, rotation_matrix_to_euler_zyx

class MeasurementModel(QObject):
    """Modèle pour les mesures de repères"""
    
    # Signaux
    measurements_changed = pyqtSignal()
    reference_changed = pyqtSignal(str)  # Nom du repère de référence
    
    
    def __init__(self):
        super().__init__()
        
        # Liste des repères mesurés
        # Chaque repère: {"name": str, "X": float, "Y": float, "Z": float, "A": float, "B": float, "C": float}
        self.reperes = []
        
        # Nom du repère de référence
        self.ref_repere = None
    
    def load_measurements(self, data):
        """Charge les mesures depuis un dictionnaire (JSON)"""
        self.reperes = data
        self.measurements_changed.emit()
    
    def get_repere_by_name(self, name):
        """Récupère un repère par son nom"""
        return next((r for r in self.reperes if r["name"] == name), None)
    
    def get_all_repere_names(self):
        """Retourne la liste des noms de repères"""
        return [r["name"] for r in self.reperes]
    
    def set_reference(self, name):
        """Définit le repère de référence et recalcule les coordonnées relatives"""
        if name in self.get_all_repere_names():
            self.ref_repere = name
            self.update_relative_coordinates()
            self.reference_changed.emit(name)
    
    def update_relative_coordinates(self):
        """Recalcule les coordonnées relatives par rapport au repère de référence"""
        if not self.ref_repere:
            return
        
        ref = self.get_repere_by_name(self.ref_repere)
        if not ref:
            return
        
        ref_matrix = self.to_matrix(ref)
        
        for rep in self.reperes:
            if rep["name"] != self.ref_repere:
                mat = self.to_matrix(rep)
                relative = np.linalg.inv(ref_matrix) @ mat
                
                # Mise à jour des translations
                rep["X"], rep["Y"], rep["Z"] = relative[0, 3], relative[1, 3], relative[2, 3]
                
                # Extraire la matrice de rotation et les angles
                R = relative[:3, :3]
                rep["A"], rep["B"], rep["C"] = rotation_matrix_to_euler_zyx(R)
        
        # Remettre le repère de référence à zéro
        ref["X"], ref["Y"], ref["Z"] = 0.0, 0.0, 0.0
        ref["A"], ref["B"], ref["C"] = 0.0, 0.0, 0.0
        
        self.measurements_changed.emit()
    
    def get_rotation_matrix(self, repere):
        """Retourne la matrice de rotation 3x3 d'un repère"""
        return euler_to_rotation_matrix(repere["A"], repere["B"], repere["C"], degrees=True)
    
    def repere_to_matrix(self, repere):
        """Convertit un repère en matrice homogène 4x4"""
        R = euler_to_rotation_matrix(repere["A"], repere["B"], repere["C"], degrees=True)
        mat = np.eye(4)
        mat[:3, :3] = R
        mat[0, 3], mat[1, 3], mat[2, 3] = repere["X"], repere["Y"], repere["Z"]
        return mat

    def clear(self):
        """Efface toutes les mesures"""
        self.reperes = []
        self.ref_repere = None
        self.measurements_changed.emit()

    def set_display_mode(self, mode):
        """Définit le mode d'affichage des mesures (absolu ou relatif)"""
        self.display_mode = mode
        self.measurements_changed.emit()  

    def euler_xyz_to_fixed_xyz(rx, ry, rz, degrees=True):
        """
        Convertit des angles d'Euler XYZ (intrinsèques/mobiles) en Fixed XYZ (extrinsèques/fixes).
        
        Euler XYZ (intrinsèques) : rotations autour des axes mobiles dans l'ordre X, Y, Z
        Fixed XYZ (extrinsèques) : rotations autour des axes fixes dans l'ordre X, Y, Z
        
        La relation est : Euler XYZ = Fixed ZYX inversé
        Donc : Fixed XYZ = inverse(Euler ZYX)
        
        Args:
            rx: Rotation autour de X (en degrés ou radians)
            ry: Rotation autour de Y (en degrés ou radians)
            rz: Rotation autour de Z (en degrés ou radians)
            degrees: True si les angles sont en degrés, False si en radians
        
        Returns:
            tuple: (fx, fy, fz) angles Fixed XYZ dans la même unité que l'entrée
        """
        # Conversion en radians si nécessaire
        if degrees:
            rx_rad = np.radians(rx)
            ry_rad = np.radians(ry)
            rz_rad = np.radians(rz)
        else:
            rx_rad = rx
            ry_rad = ry
            rz_rad = rz
    
        # Matrices de rotation élémentaires
        def rot_x(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([
                [1, 0, 0],
                [0, c, -s],
                [0, s, c]
            ])

        def rot_y(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([
                [c, 0, s],
                [0, 1, 0],
                [-s, 0, c]
            ])

        def rot_z(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([
                [c, -s, 0],
                [s, c, 0],
                [0, 0, 1]
            ])
        # Matrice de rotation Euler XYZ (intrinsèque)
        # R = Rz(rz) * Ry(ry) * Rx(rx) dans le repère mobile
        R_euler = rot_z(rz_rad) @ rot_y(ry_rad) @ rot_x(rx_rad)
        
        # Pour Fixed XYZ, on a : R = Rx(fx) * Ry(fy) * Rz(fz)
        # On extrait les angles depuis la matrice de rotation
        
        # Extraction des angles Fixed XYZ depuis la matrice
        # R[2,0] = -sin(fy)
        # R[0,0] = cos(fy)*cos(fz)
        # R[1,0] = cos(fy)*sin(fz)
        # R[2,1] = sin(fx)*cos(fy)
        # R[2,2] = cos(fx)*cos(fy)
        
        fy_fixed = np.arcsin(-R_euler[2, 0])
        
        # Vérification du gimbal lock
        if np.abs(np.cos(fy_fixed)) > 1e-6:
            fx_fixed = np.arctan2(R_euler[2, 1], R_euler[2, 2])
            fz_fixed = np.arctan2(R_euler[1, 0], R_euler[0, 0])
        else:
            # Cas du gimbal lock (fy = ±90°)
            fx_fixed = np.arctan2(-R_euler[1, 2], R_euler[1, 1])
            fz_fixed = 0
        
        # Conversion en degrés si nécessaire
        if degrees:
            return (np.degrees(fx_fixed), np.degrees(fy_fixed), np.degrees(fz_fixed))
        else:
            return (fx_fixed, fy_fixed, fz_fixed)

    def fixed_xyz_to_euler_xyz(fx, fy, fz, degrees=True):
        """
        Convertit des angles Fixed XYZ (extrinsèques) en Euler XYZ (intrinsèques).
        Fonction inverse de euler_xyz_to_fixed_xyz.
        
        Args:
            fx: Rotation Fixed autour de X
            fy: Rotation Fixed autour de Y
            fz: Rotation Fixed autour de Z
            degrees: True si les angles sont en degrés
        
        Returns:
            tuple: (rx, ry, rz) angles Euler XYZ
        """
        if degrees:
            fx_rad = np.radians(fx)
            fy_rad = np.radians(fy)
            fz_rad = np.radians(fz)
        else:
            fx_rad = fx
            fy_rad = fy
            fz_rad = fz
        
        def rot_x(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        
        def rot_y(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        
        def rot_z(angle):
            c, s = np.cos(angle), np.sin(angle)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        
        # Matrice Fixed XYZ
        R_fixed = rot_x(fx_rad) @ rot_y(fy_rad) @ rot_z(fz_rad)
        
        # Extraction Euler XYZ
        ry_euler = np.arcsin(R_fixed[0, 2])
        
        if np.abs(np.cos(ry_euler)) > 1e-6:
            rx_euler = np.arctan2(-R_fixed[1, 2], R_fixed[2, 2])
            rz_euler = np.arctan2(-R_fixed[0, 1], R_fixed[0, 0])
        else:
            rx_euler = np.arctan2(R_fixed[2, 1], R_fixed[1, 1])
            rz_euler = 0
        
        if degrees:
            return (np.degrees(rx_euler), np.degrees(ry_euler), np.degrees(rz_euler))
        else:
            return (rx_euler, ry_euler, rz_euler)  
    


