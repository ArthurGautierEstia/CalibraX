from PyQt5.QtCore import QObject, QTimer
import numpy as np
from typing import Tuple, Optional

from models.robot_model import RobotModel
from views.jog_view import JogView
import utils.math_utils as math_utils

class JogController(QObject):
    """Contrôleur pour la vue Jog - gère les interactions de jog articulaire et cartésien avec jog continu"""
    
    def __init__(self, robot_model: RobotModel, jog_view: JogView, parent: QObject = None):
        super().__init__(parent)
        
        self.robot_model = robot_model
        self.jog_view = jog_view

        self.jog_joint_widget = self.jog_view.get_jog_joint_widget()
        self.jog_cartesian_widget = self.jog_view.get_jog_cartesian_widget()
        self.jog_angles_visualization_widget = self.jog_view.get_jog_angles_visualization_widget()
        self.jog_tcp_visualization_widget = self.jog_view.get_jog_tcp_visualization_widget()
        self.jog_matrix_widget = self.jog_view.get_matrix_widget()
        
        # Paramètres de jog
        self.jog_step_joint: float = 1  # Incrément en degrés pour jog articulaire
        self.jog_step_cartesian: float = 1  # Incrément en mm pour jog cartésien
        self.jog_timer_interval: int = 20  # Intervalle de mise à jour en ms
        
        # Référentiel actuel (Base ou Tool)
        self.base_tool_reference = "Base"
        
        # État du jog continu
        self._jog_active_joint: Optional[Tuple[int, int]] = None  # (index, direction: -1 or 1)
        self._jog_active_cartesian: Optional[Tuple[int, int]] = None  # (index, direction: -1 or 1)
        
        # Timer pour jog continu
        self.jog_timer = QTimer()
        self.jog_timer.timeout.connect(self._on_jog_timer_tick)
        
        self.jog_joint_widget.set_delta(self.jog_step_joint)
        self.jog_cartesian_widget.set_delta(self.jog_step_cartesian)

        self._update_axis_limits()
        self._update_display_from_model()
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        """Configure les connexions entre la vue et le modèle"""
        
        # Connexions du widget de contrôle Jog articulaire
        self.jog_joint_widget.jog_joint_minus_pressed.connect(lambda idx: self._on_jog_joint_pressed(idx, -1))
        self.jog_joint_widget.jog_joint_minus_released.connect(lambda idx: self._on_jog_joint_released(idx))
        self.jog_joint_widget.jog_joint_plus_pressed.connect(lambda idx: self._on_jog_joint_pressed(idx, 1))
        self.jog_joint_widget.jog_joint_plus_released.connect(lambda idx: self._on_jog_joint_released(idx))
        self.jog_joint_widget.delta_changed.connect(self._on_jog_joint_delta_changed)

        # Connexions du widget de contrôle Jog cartésien
        self.jog_cartesian_widget.jog_cartesian_minus_pressed.connect(lambda idx: self._on_jog_cartesian_pressed(idx, -1))
        self.jog_cartesian_widget.jog_cartesian_minus_released.connect(lambda idx: self._on_jog_cartesian_released(idx))
        self.jog_cartesian_widget.jog_cartesian_plus_pressed.connect(lambda idx: self._on_jog_cartesian_pressed(idx, 1))
        self.jog_cartesian_widget.jog_cartesian_plus_released.connect(lambda idx: self._on_jog_cartesian_released(idx))
        self.jog_cartesian_widget.delta_changed.connect(self._on_jog_cartesian_delta_changed)

        self.jog_cartesian_widget.jog_base_tool_changed.connect(self._on_base_tool_changed)
        
        # Connexions du modèle pour mettre à jour l'affichage
        self.robot_model.tcp_pose_changed.connect(self._update_display_from_model)
        self.robot_model.axis_limits_changed.connect(self._update_axis_limits)
    
    def _on_jog_joint_delta_changed(self, value: float):
        self.jog_step_joint = value
    
    def _on_jog_cartesian_delta_changed(self, value: float):
        self.jog_step_cartesian = value

    def _on_jog_joint_pressed(self, joint_index: int, direction: int) -> None:
        """Appelé quand un bouton de jog articulaire est enfoncé (pressed)"""
        if not self.robot_model.has_confifiguration:
            return
        
        # Arrêter tout jog en cours
        self.jog_timer.stop()
        self._jog_active_joint = (joint_index, direction)
        self._jog_active_cartesian = None
        
        # Démarrer le timer pour le jog continu
        self.jog_timer.start(self.jog_timer_interval)
    
    def _on_jog_joint_released(self, joint_index: int) -> None:
        """Appelé quand un bouton de jog articulaire est relâché (released)"""
        # Arrêter le jog si c'est le même joint
        if self._jog_active_joint and self._jog_active_joint[0] == joint_index:
            self.jog_timer.stop()
            self._jog_active_joint = None
    
    def _on_jog_cartesian_pressed(self, axis_index: int, direction: int) -> None:
        """Appelé quand un bouton de jog cartésien est enfoncé (pressed)"""
        if not self.robot_model.has_confifiguration:
            return
        
        # Arrêter tout jog en cours
        self.jog_timer.stop()
        self._jog_active_cartesian = (axis_index, direction)
        self._jog_active_joint = None
        
        # Démarrer le timer pour le jog continu
        self.jog_timer.start(self.jog_timer_interval)
    
    def _on_jog_cartesian_released(self, axis_index: int) -> None:
        """Appelé quand un bouton de jog cartésien est relâché (released)"""
        # Arrêter le jog si c'est le même axe
        if self._jog_active_cartesian and self._jog_active_cartesian[0] == axis_index:
            self.jog_timer.stop()
            self._jog_active_cartesian = None
    
    def _on_jog_timer_tick(self) -> None:
        """Appelé périodiquement pour effectuer le jog continu"""
        if self._jog_active_joint:
            joint_index, direction = self._jog_active_joint
            self._jog_joint(joint_index, direction * self.jog_step_joint)
        
        elif self._jog_active_cartesian:
            axis_index, direction = self._jog_active_cartesian
            self._jog_cartesian(axis_index, direction * self.jog_step_cartesian)
    
    def _jog_joint(self, joint_index: int, delta: float) -> None:
        """Effectue un jog articulaire en modifiant la valeur du joint"""
        # Obtenir la valeur actuelle du joint
        current_value = self.robot_model.get_joint(joint_index)
        
        # Appliquer le delta au joint spécifié
        new_value = current_value + delta
        
        # Respecter les limites
        min_limit, max_limit = self.robot_model.get_axis_limit(joint_index)
        new_value = max(min_limit, min(max_limit, new_value))
        
        # Mettre à jour seulement si la valeur a changé (dans les limites)
        if new_value != current_value:
            self.robot_model.set_joint(joint_index, new_value)
    
    def _jog_cartesian(self, axis_index: int, delta: float) -> None:
        """Effectue un jog cartésien
        
        Crée un décalage global [dx, dy, dz, da, db, dc], le convertit au référentiel BASE 
        si nécessaire (si en mode TOOL), puis appelle le MGI pour trouver la nouvelle pose.
        
        Args:
            axis_index: 0=X, 1=Y, 2=Z, 3=A, 4=B, 5=C
            delta: Décalage à appliquer
        """
        try:
            # Obtenir la pose TCP actuelle [X, Y, Z, A, B, C]
            current_tcp_pose = np.array(self.robot_model.get_tcp_pose())
            target = current_tcp_pose.copy()

            # Si le référentiel est TOOL, convertir le décalage en référentiel BASE
            if self.base_tool_reference == "Tool":
                if axis_index < 3:
                    # transform xyz
                    pass
                    delta_pos = np.array([0., 0., 0.])
                    delta_pos[axis_index] = delta
                    delta_in_base = self.robot_model.get_tcp_rotation_matrix() @ delta_pos

                    target[0] += delta_in_base[0]
                    target[1] += delta_in_base[1]
                    target[2] += delta_in_base[2]

                else:
                    delta_rotation = math_utils.rot_z(delta) if axis_index == 3 else (math_utils.rot_y(delta) if axis_index == 4 else math_utils.rot_x(delta))
                    new_tcp_rotation =  self.robot_model.get_tcp_rotation_matrix() @ delta_rotation
                    newABC = math_utils.rotation_matrix_to_euler_zyx(new_tcp_rotation)

                    target[3] = newABC[0]
                    target[4] = newABC[1]
                    target[5] = newABC[2]

            else:  # BASE
                # Le décalage est déjà en BASE, on l'ajoute simplement
                if axis_index < 3:
                    # transform xyz
                    target[axis_index] += delta
                    
                else:
                    # transform abc
                    delta_rotation = math_utils.rot_z(delta) if axis_index == 3 else (math_utils.rot_y(delta) if axis_index == 4 else math_utils.rot_x(delta))
                    new_tcp_rotation = delta_rotation @ self.robot_model.get_tcp_rotation_matrix()
                    newABC = math_utils.rotation_matrix_to_euler_zyx(new_tcp_rotation)

                    target[3] = newABC[0]
                    target[4] = newABC[1]
                    target[5] = newABC[2]

            # Appeler le MGI sur la nouvelle pose cible
            mgi_result = self.robot_model.compute_ik_target(target)
            
            # Si des solutions existent, appliquer la meilleure
            if mgi_result:
                best_solution = self.robot_model.get_best_mgi_solution(mgi_result)
                if best_solution is not None:
                    # best_solution est en radians, convertir en degrés
                    self.robot_model.set_joints(best_solution[1].joints)
                    
        except Exception as e:
            print(f"Erreur lors du jog cartésien: {e}")

    def _on_base_tool_changed(self, reference: str) -> None:
        """Appelé quand l'utilisateur change de référentiel (Base/Tool)"""
        self.base_tool_reference = reference
    
    def _update_display_from_model(self) -> None:
        """Met à jour l'affichage depuis les données du modèle"""
        if not self.robot_model.has_confifiguration:
            return
        
        # Mettre à jour la visualisation des angles
        self.jog_angles_visualization_widget.set_joint_values(self.robot_model.joint_values)
        self.jog_angles_visualization_widget.set_axis_limits(self.robot_model.axis_limits)
        
        # Mettre à jour la visualisation du TCP
        tcp_pose = self.robot_model.get_tcp_pose()
        self.jog_tcp_visualization_widget.set_tcp_pose(tcp_pose)
        self.jog_matrix_widget.set_matrix(self.robot_model.get_tcp_rotation_matrix())
    
    def _update_axis_limits(self) -> None:
        """Met à jour les limites des axes affichées"""
        angles_viz = self.jog_view.get_jog_angles_visualization_widget()
        angles_viz.set_axis_limits(self.robot_model.axis_limits)
