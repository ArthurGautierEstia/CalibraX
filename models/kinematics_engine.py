import numpy as np
from PyQt5.QtCore import QObject, pyqtSignal
from utils.math_utils import dh_modified, correction_6d, matrix_to_euler_zyx

class KinematicsEngine(QObject):
    """Moteur de calcul cinématique (MGD)"""
    
    # Signal émis quand les calculs sont mis à jour
    kinematics_updated = pyqtSignal()
    
    def __init__(self, robot_model):
        super().__init__()
        self.robot_model = robot_model
        
        # Résultats des calculs
        self.dh_matrices = [np.eye(4)]
        self.corrected_matrices = [np.eye(4)]
        self.dh_pos = np.zeros(3)
        self.dh_ori = np.zeros(3)
        self.corrected_pos = np.zeros(3)
        self.corrected_ori = np.zeros(3)
        
        # Connecter au modèle pour recalculer automatiquement
        self.robot_model.configuration_changed.connect(self.compute_forward_kinematics)
        self.robot_model.joints_changed.connect(self.compute_forward_kinematics)
    
    def get_parameters(self):
        """Récupère les paramètres pour le calcul cinématique"""
        params = []
        for i in range(7):
            alpha = np.radians(self.robot_model.get_dh_param(i, 0))
            d = self.robot_model.get_dh_param(i, 1)
            theta_offset = np.radians(self.robot_model.get_dh_param(i, 2))
            r = self.robot_model.get_dh_param(i, 3)
            
            # Pour les 6 premiers joints, ajouter la valeur articulaire
            if i < 6:
                q_deg = self.robot_model.joint_values[i]
                q = np.radians(q_deg)
                theta = theta_offset + q
                corr = [self.robot_model.get_correction(i, j) for j in range(6)]
            else:
                # Joint 7 (tool) : pas de variable articulaire
                theta = theta_offset
                corr = [0, 0, 0, 0, 0, 0]
            
            params.append((alpha, d, theta, r, corr))
        
        return params
    
    def compute_forward_kinematics(self):
        """Calcule le MGD (position et orientation du TCP)"""
        params = self.get_parameters()
        
        # Réinitialiser les matrices
        self.dh_matrices = [np.eye(4)]
        self.corrected_matrices = [np.eye(4)]
        
        T_dh = np.eye(4)
        T_corrected = np.eye(4)
        
        # Calcul itératif des transformations
        for (alpha, d, theta, r, corr) in params:
            # Transformation DH standard
            T_dh = T_dh @ dh_modified(alpha, d, theta, r)
            self.dh_matrices.append(T_dh.copy())
            
            # Transformation avec correction
            T_corrected = T_corrected @ dh_modified(alpha, d, theta, r)
            T_corrected = correction_6d(T_corrected, *corr)
            self.corrected_matrices.append(T_corrected.copy())
        
        # Extraction position et orientation (DH standard)
        self.dh_pos = T_dh[:3, 3]
        self.dh_ori = matrix_to_euler_zyx(T_dh)
        
        # Extraction position et orientation (corrigée)
        self.corrected_pos = T_corrected[:3, 3]
        self.corrected_ori = matrix_to_euler_zyx(T_corrected)
        
        # Notifier les observateurs
        self.kinematics_updated.emit()
    
    def get_tcp_pose(self):
        """Retourne la pose TCP (position + orientation) standard"""
        return np.concatenate([self.dh_pos, self.dh_ori])
    
    def get_corrected_tcp_pose(self):
        """Retourne la pose TCP corrigée"""
        return np.concatenate([self.corrected_pos, self.corrected_ori])
    
    def get_pose_deviation(self):
        """Retourne les écarts entre TCP standard et corrigé"""
        pos_dev = self.corrected_pos - self.dh_pos
        ori_dev = self.corrected_ori - self.dh_ori
        return np.concatenate([pos_dev, ori_dev])
    
    def get_matrix_at_joint(self, joint_index, corrected=False):
        """Retourne la matrice de transformation à un joint donné"""
        matrices = self.corrected_matrices if corrected else self.dh_matrices
        if 0 <= joint_index < len(matrices):
            return matrices[joint_index]
        return np.eye(4)
