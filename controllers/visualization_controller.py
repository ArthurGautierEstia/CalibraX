from PyQt5.QtCore import QObject
import numpy as np

class VisualizationController(QObject):
    """Contrôleur pour la visualisation 3D"""
    
    def __init__(self, kinematics_engine, robot_model, viewer_widget):
        super().__init__()
        self.kinematics_engine = kinematics_engine
        self.robot_model = robot_model
        self.viewer_widget = viewer_widget
        
        # État du mode pas à pas
        self.step_mode = False
        self.step_index = 0
        
        # État de l'affichage
        self.show_axes = True
        self.cad_visible = False
        
        # Connecter les signaux
        self.kinematics_engine.kinematics_updated.connect(self.update_visualization)
        self.viewer_widget.transparency_toggled.connect(self.toggle_transparency)
        self.viewer_widget.axes_toggled.connect(self.toggle_axes)
        self.viewer_widget.prev_frame_requested.connect(self.show_previous_frame)
        self.viewer_widget.next_frame_requested.connect(self.show_next_frame)
    
    def update_visualization(self):
        """Met à jour la visualisation 3D"""
        # Effacer le viewer
        self.viewer_widget.clear_viewer()
        
        # Afficher les repères si activés
        if self.show_axes:
            if self.step_mode:
                # Mode pas à pas: afficher tous les repères + le repère actuel en jaune
                self.viewer_widget.draw_all_frames(self.kinematics_engine.dh_matrices)
                if 0 <= self.step_index < len(self.kinematics_engine.dh_matrices):
                    T = self.kinematics_engine.dh_matrices[self.step_index]
                    self.viewer_widget.draw_highlighted_frame(T)
            else:
                # Mode normal: afficher tous les repères
                self.viewer_widget.draw_all_frames(self.kinematics_engine.dh_matrices)
        
        # Mettre à jour les poses du robot CAD
        #if self.cad_visible and len(self.viewer_widget.robot_links) > 0:
        self.viewer_widget.update_robot_poses(self.kinematics_engine.corrected_matrices)
    
    def toggle_step_mode(self):
        """Bascule le mode pas à pas"""
        self.step_mode = not self.step_mode
        
        if self.step_mode:
            self.step_index = 0
            self.viewer_widget.set_navigation_buttons_visible(True)
        else:
            self.viewer_widget.set_navigation_buttons_visible(False)
        
        self.update_visualization()
    
    def show_previous_frame(self):
        """Affiche le repère précédent en mode pas à pas"""
        if self.step_index > 0:
            self.step_index -= 1
            self.update_visualization()
    
    def show_next_frame(self):
        """Affiche le repère suivant en mode pas à pas"""
        if self.step_index < len(self.kinematics_engine.dh_matrices) - 1:
            self.step_index += 1
            self.update_visualization()
    
    def toggle_transparency(self):
        """Bascule la transparence des mesh"""
        transparency_enabled = not self.viewer_widget.transparency_enabled
        self.viewer_widget.set_transparency(transparency_enabled)
    
    def toggle_axes(self):
        """Bascule l'affichage des repères"""
        self.show_axes = not self.show_axes
        self.update_visualization()
    
    def toggle_cad_visibility(self, visible):
        """Bascule la visibilité du robot CAD"""
        self.cad_visible = visible
        
        if visible:
            # Si pas encore chargé, charger les mesh
            if len(self.viewer_widget.robot_links) == 0:
                self.load_robot_cad()
            else:
                self.viewer_widget.set_robot_visibility(True)
        else:
            self.viewer_widget.set_robot_visibility(False)
    
    def load_robot_cad(self):
        """Charge le CAD du robot"""
        # Vérifier qu'il y a un fichier de configuration
        if not self.robot_model.current_config_file:
            print("Aucune configuration chargée, impossible de charger le CAD")
            return
        
        # Charger les mesh avec les matrices corrigées
        self.viewer_widget.add_robot_links(self.kinematics_engine.corrected_matrices)
    
    def reload_robot_cad(self):
        """Recharge le CAD du robot (après changement de configuration)"""
        # Nettoyer les anciens mesh
        self.viewer_widget.clear_robot_links()
        
        # Recharger si CAD était visible
        if self.cad_visible:
            self.load_robot_cad()
