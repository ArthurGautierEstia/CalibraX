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
    
    def to_matrix(self, repere):
        """Convertit un repère en matrice homogène 4x4"""
        R = euler_to_rotation_matrix(repere["A"], repere["B"], repere["C"], degrees=True)
        mat = np.eye(4)
        mat[:3, :3] = R
        mat[0, 3], mat[1, 3], mat[2, 3] = repere["X"], repere["Y"], repere["Z"]
        return mat
    
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
    
    def clear(self):
        """Efface toutes les mesures"""
        self.reperes = []
        self.ref_repere = None
        self.measurements_changed.emit()
