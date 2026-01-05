from PyQt5.QtCore import QObject
import numpy as np

class VisualizationController(QObject):
    """Contrôleur pour la visualisation 3D"""
    
    def __init__(self, kinematics_engine, robot_model, viewer_widget, dh_widget):
        super().__init__()
        self.kinematics_engine = kinematics_engine
        self.robot_model = robot_model
        self.viewer_widget = viewer_widget
        self.dh_widget = dh_widget
        # État du mode pas à pas
        self.step_mode = False
        self.step_index = 0
        
        # État de l'affichage
        self.show_axes = True
        self.frames_visibility = []  # Visibilité individuelle des repères
        self.cad_visible = False
        self.cad_loaded = False
        
    def setup_connections(self):
        """Configure les connexions entre la vue et le modèle"""    
        # Connecter les signaux
        self.kinematics_engine.kinematics_updated.connect(self.update_visualization)
        self.viewer_widget.transparency_toggled.connect(self.toggle_transparency)
        self.viewer_widget.axes_toggled.connect(self.toggle_axes)
        self.viewer_widget.frame_visibility_toggled.connect(self.toggle_single_frame)
        self.dh_widget.cad_toggled.connect(self.toggle_cad_visibility)

    def update_visualization(self):
        """Met à jour la visualisation 3D"""
        matrices = self.kinematics_engine.dh_matrices
        num_frames = len(matrices)

        # 1. Initialiser la liste de visibilité si elle est vide ou de mauvaise taille
        if len(self.frames_visibility) != num_frames:
            self.frames_visibility = [True] * num_frames

        # 2. Mettre à jour l'interface de la liste (Gras/Normal)
        self.viewer_widget.update_frame_list_ui(self.frames_visibility)

        # 3. Effacer le viewer
        self.viewer_widget.clear_viewer()

        # 4. Afficher les repères
        if self.show_axes:
            # CORRECTION ICI : On utilise la bonne méthode et on passe les DEUX arguments
            # Vérifiez si vous avez nommé la méthode 'draw_frames' ou 'draw_all_frames' dans viewer_3d_widget.py
            # Si vous avez copié mon code précédent, c'est 'draw_frames'.
            if hasattr(self.viewer_widget, 'draw_frames'):
                 self.viewer_widget.draw_all_frames(matrices, self.frames_visibility)
            else:
                 # Si vous avez gardé l'ancien nom 'draw_all_frames' mais avec les nouveaux arguments
                 self.viewer_widget.draw_all_frames(matrices, self.frames_visibility)

        # 5. Mettre à jour le robot CAD
        if self.cad_loaded:
            self.viewer_widget.update_robot_poses(self.kinematics_engine.corrected_matrices)
    
    def toggle_transparency(self):
        """Bascule la transparence des mesh"""
        transparency_enabled = not self.viewer_widget.transparency_enabled
        self.viewer_widget.set_transparency(transparency_enabled)
    
    def toggle_axes(self):
        """Bascule l'affichage des repères"""
        self.show_axes = not self.show_axes
        self.update_visualization()
    
    def toggle_single_frame(self, index):
        """Action quand on clique sur un item de la liste"""
        if 0 <= index < len(self.frames_visibility):
            old = self.frames_visibility[index]
            self.frames_visibility[index] = not old
            self.update_visualization()
    
    def toggle_cad_visibility(self, visible):
        """Bascule la visibilité du robot CAD"""
        self.cad_visible = visible
        print(f"CAD visibility set to: {visible}")
        
        if visible:
            # Charger le CAD seulement s'il n'a jamais été chargé
            if not self.cad_loaded:
                self.load_robot_cad()
            else:
                # Sinon, simplement afficher le modèle déjà chargé
                self.viewer_widget.set_robot_visibility(True)
        else:
            # Cacher sans décharger
            self.viewer_widget.set_robot_visibility(False)
    
    def load_robot_cad(self):
        """Charge le CAD du robot"""
        # Vérifier qu'il y a un fichier de configuration
        if not self.robot_model.current_config_file:
            print("Aucune configuration chargée, impossible de charger le CAD")
            return
        
        # Charger les mesh avec les matrices corrigées
        self.viewer_widget.add_robot_links(self.kinematics_engine.corrected_matrices)

        self.cad_loaded = True
    
    # def reload_robot_cad(self):
    #     """Recharge le CAD du robot (après changement de configuration)"""
    #     # Nettoyer les anciens mesh
    #     self.viewer_widget.clear_robot_links()
        
    #     # Recharger si CAD était visible
    #     if self.cad_visible:
    #         self.viewer_widget.update_robot_poses(self.kinematics_engine.corrected_matrices)
