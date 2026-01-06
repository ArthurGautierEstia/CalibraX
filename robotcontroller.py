from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QDialog
from utils.file_io import FileIOHandler
from utils.math_utils import *
from views.dialogs.axis_limits_dialog import AxisLimitsDialog

class RobotController(QObject):
    """Contrôleur centralisé pour la gestion du robot et synchronisation des widgets"""
    
    # ============================================================================
    # RÉGION: Signaux
    # ============================================================================
    
    # Signaux internes pour actions asynchrones
    configuration_loaded = pyqtSignal(dict)  # Émis quand une config est chargée
    tcp_pose_updated = pyqtSignal(list, list, list)  # tcp, corrected_tcp, deviation
    
    def __init__(self, robot_model, dh_widget, correction_widget, joint_widget, 
                 result_widget, measurement_widget, visualization_widget):
        super().__init__()
        
        # ====================================================================
        # RÉGION: Injection de dépendances
        # ====================================================================
        self.robot_model = robot_model
        self.dh_widget = dh_widget
        self.correction_widget = correction_widget
        self.joint_widget = joint_widget
        self.result_widget = result_widget
        self.measurement_widget = measurement_widget
        self.visualization_widget = visualization_widget
        self.file_io = FileIOHandler()
    
    def setup_connections(self):
        """Configure toutes les connexions de signaux"""
        
        # ====================================================================
        # RÉGION: Connexions DH Widget -> Contrôleur
        # ====================================================================
        self.dh_widget.load_config_requested.connect(self.on_load_configuration)
        self.dh_widget.save_config_requested.connect(self.on_save_configuration)
        self.dh_widget.dh_value_changed.connect(self.on_dh_value_changed)
        self.dh_widget.cad_toggled.connect(self.on_cad_toggled)
        
        # ====================================================================
        # RÉGION: Connexions Correction Widget -> Contrôleur
        # ====================================================================
        self.correction_widget.correction_value_changed.connect(self.on_correction_value_changed)
        
        # ====================================================================
        # RÉGION: Connexions Joint Widget -> Contrôleur
        # ====================================================================
        self.joint_widget.joint_value_changed.connect(self.on_joint_value_changed)
        self.joint_widget.home_position_requested.connect(self.on_home_position_requested)
        self.joint_widget.axis_limits_config_requested.connect(self.on_axis_limits_config_requested)
        self.joint_widget.step_by_step_requested.connect(self.on_step_by_step_requested)
        
        # ====================================================================
        # RÉGION: Connexions Result Widget -> Contrôleur
        # ====================================================================
        self.result_widget.jog_increment_requested.connect(self.on_jog_increment_requested)
        
        # ====================================================================
        # RÉGION: Connexions Measurement Widget -> Contrôleur
        # ====================================================================
        self.measurement_widget.import_measurements_requested.connect(self.on_import_measurements)
        self.measurement_widget.set_as_reference_requested.connect(self.on_set_as_reference)
        self.measurement_widget.calculate_corrections_requested.connect(self.on_calculate_corrections)
        self.measurement_widget.repere_selected.connect(self.on_repere_selected)
        self.measurement_widget.display_mode_changed.connect(self.on_display_mode_changed)
        self.measurement_widget.rotation_type_changed.connect(self.on_rotation_type_changed)
        self.measurement_widget.clear_measurements_requested.connect(self.on_clear_measurements)
        
        # ====================================================================
        # RÉGION: Connexions Visualization Widget -> Contrôleur
        # ====================================================================
        self.visualization_widget.transparency_toggled.connect(self.on_transparency_toggled)
        self.visualization_widget.axes_toggled.connect(self.on_axes_toggled)
        self.visualization_widget.frame_visibility_toggled.connect(self.on_frame_visibility_toggled)
        
        # ====================================================================
        # RÉGION: Connexions RobotModel -> Widgets (signaux du modèle)
        # ====================================================================
        self.robot_model.configuration_changed.connect(self.on_configuration_changed)
        self.robot_model.robot_name_changed.connect(self.on_robot_name_changed)
        self.robot_model.dh_params_changed.connect(self.on_dh_params_changed)
        self.robot_model.joints_changed.connect(self.on_joints_changed)
        self.robot_model.corrections_changed.connect(self.on_corrections_changed)
        self.robot_model.limits_changed.connect(self.on_limits_changed)
        self.robot_model.axis_reversed_changed.connect(self.on_axis_reversed_changed)
        self.robot_model.home_position_changed.connect(self.on_home_position_changed)
        self.robot_model.tcp_pose_changed.connect(self.on_tcp_pose_changed)
        self.robot_model.corrected_tcp_pose_changed.connect(self.on_corrected_tcp_pose_changed)
        self.robot_model.pose_deviation_changed.connect(self.on_pose_deviation_changed)
        self.robot_model.measurements_changed.connect(self.on_measurements_changed)
        self.robot_model.measurement_points_changed.connect(self.on_measurement_points_changed)
    
    # ============================================================================
    # RÉGION: Callbacks DH Widget
    # ============================================================================
    
    def on_load_configuration(self):
        """Callback: charger une configuration depuis fichier"""
        file_name, data = self.file_io.load_json(
            self.dh_widget,
            "Charger configuration"
        )
        if data:
            self.robot_model.load_from_dict(data, file_name)
        
    
    def on_save_configuration(self):
        """Callback: sauvegarder la configuration actuelle"""
        data = self.robot_model.to_dict()
        file_name = self.file_io.save_json(
            self.dh_widget,
            "Sauvegarder configuration",
            data
        )
        if file_name:
            self.robot_model.set_current_config_file(file_name)
    
    def on_dh_value_changed(self, row, col, value):
        """Callback: un paramètre DH a changé dans le widget"""
        try:
            self.robot_model.set_dh_param(row, col, float(value))
            # Recalculer la cinématique
            self._update_kinematics()
        except ValueError:
            print(f"Erreur: valeur DH invalide [{row},{col}] = {value}")
    
    def on_cad_toggled(self, checked):
        """Callback: affichage CAD activé/désactivé"""
        if checked:
            # Charger et afficher le modèle CAD
            self._load_robot_cad()
        else:
            # Masquer le modèle CAD
            self.visualization_widget.set_robot_visibility(False)
    
    # ============================================================================
    # RÉGION: Callbacks Correction Widget
    # ============================================================================
    
    def on_correction_value_changed(self, row, col, value):
        """Callback: une correction a changé dans le widget"""
        try:
            self.robot_model.set_correction(row, col, float(value))
            # Recalculer la cinématique corrigée
            self._update_kinematics()
        except ValueError:
            print(f"Erreur: correction invalide [{row},{col}] = {value}")
    
    # ============================================================================
    # RÉGION: Callbacks Joint Widget
    # ============================================================================
    
    def on_joint_value_changed(self, index, value):
        """Callback: la valeur d'un joint a changé"""
        self.robot_model.set_joint_value(index, value)
        self._update_kinematics()
    
    def on_home_position_requested(self):
        """Callback: retourner à la position home"""
        home_pos = self.robot_model.get_home_position()
        self.robot_model.set_all_joint_values(home_pos)
    
    def on_axis_limits_config_requested(self):
        """Callback: ouvrir la boîte de dialogue de configuration des limites"""
        dialog = AxisLimitsDialog(
            self.joint_widget,
            self.robot_model.get_axis_limits(),
            self.robot_model.get_home_position(),
            self.robot_model.get_axis_reversed()
        )
        
        if dialog.exec_() == QDialog.Accepted:
            # Récupérer les données du dialogue
            limits = dialog.get_limits()
            home_pos = dialog.get_home_position()
            axis_reversed = dialog.get_axis_reversed()
            
            # Récupérer l'ancienne valeur d'inversion avant mise à jour
            old_reversed = self.robot_model.get_axis_reversed()

            # Mettre à jour le modèle
            self.robot_model.set_axis_limits(limits)
            self.robot_model.set_home_position(home_pos)
            self.robot_model.set_axis_reversed(axis_reversed)
            
            # Appliquer l'inversion des valeurs des spinboxes si l'état d'inversion a changé
            self.joint_widget.apply_axis_inversion(old_reversed, axis_reversed)
    
    def on_step_by_step_requested(self):
        """Callback: affichage pas à pas demandé"""
        # TODO: Implémenter la fonctionnalité de pas à pas
        pass
    
    # ============================================================================
    # RÉGION: Callbacks Result Widget
    # ============================================================================
    
    def on_jog_increment_requested(self, row, delta):
        """Callback: incrémenter/décrémenter un élément de TCP"""
        # TODO: Implémenter le jog du TCP
        pass
    
    # ============================================================================
    # RÉGION: Callbacks Measurement Widget
    # ============================================================================
    
    def on_import_measurements(self):
        """Callback: importer des mesures depuis fichier"""
        # TODO: Implémenter l'import de mesures
        pass
    
    def on_set_as_reference(self):
        """Callback: définir le repère courant comme référence"""
        # TODO: Implémenter la définition de référence
        pass
    
    def on_calculate_corrections(self):
        """Callback: calculer les corrections à partir des mesures"""
        # TODO: Implémenter le calcul de corrections
        pass
    
    def on_repere_selected(self, repere_name):
        """Callback: un repère a été sélectionné dans le widget"""
        # TODO: Afficher les données du repère sélectionné
        pass
    
    def on_display_mode_changed(self, mode):
        """Callback: le mode d'affichage a changé (Repères/Écarts)"""
        # TODO: Implémenter le changement de mode d'affichage
        pass
    
    def on_rotation_type_changed(self, rotation_type):
        """Callback: le type de rotation a changé"""
        # TODO: Implémenter le changement de type de rotation
        pass
    
    def on_clear_measurements(self):
        """Callback: effacer toutes les mesures"""
        self.robot_model.clear_measurements()
        self.robot_model.clear_measurement_points()
        self.measurement_widget.clear_measurements()
    
    # ============================================================================
    # RÉGION: Callbacks Visualization Widget
    # ============================================================================
    
    def on_transparency_toggled(self):
        """Callback: transparence du modèle 3D activée/désactivée"""
        current_state = self.visualization_widget.transparency_enabled
        self.visualization_widget.set_transparency(not current_state)
    
    def on_axes_toggled(self):
        """Callback: affichage des repères global activé/désactivé"""
        self.visualization_widget.show_axes = not self.visualization_widget.show_axes
    
    def on_frame_visibility_toggled(self, frame_index):
        """Callback: visibilité d'un repère individuel a changé"""
        # TODO: Implémenter le toggle de visibilité d'un repère
        pass
    
    # ============================================================================
    # RÉGION: Callbacks RobotModel (synchronisation modèle -> widgets)
    # ============================================================================
    
    def on_configuration_changed(self):
        """Callback: la configuration globale a changé"""
        self._update_all_widgets_from_model()
    
    def on_robot_name_changed(self, name):
        """Callback: le nom du robot a changé"""
        self.dh_widget.set_robot_name(name)
        self.measurement_widget.label_robot_name_me.setText(name)
    
    def on_dh_params_changed(self):
        """Callback: les paramètres DH ont changé"""
        dh_params = self.robot_model.get_dh_params()
        # Convertir en strings pour le widget
        dh_params_str = [[str(val) for val in row] for row in dh_params]
        self.dh_widget.set_dh_params(dh_params_str)
        self._update_kinematics()
    
    def on_joints_changed(self):
        """Callback: les joints ont changé"""
        joint_values = self.robot_model.get_all_joint_values()
        self.joint_widget.set_all_joints(joint_values)
        self._update_kinematics()
    
    def on_corrections_changed(self):
        """Callback: les corrections ont changé"""
        corrections = self.robot_model.get_corrections()
        # Convertir en strings pour le widget
        corrections_str = [[str(val) for val in row] for row in corrections]
        self.correction_widget.set_corrections(corrections_str)
        self._update_kinematics()
    
    def on_limits_changed(self):
        """Callback: les limites des axes ont changé"""
        limits = self.robot_model.get_axis_limits()
        self.joint_widget.update_axis_limits(limits)
    
    def on_axis_reversed_changed(self):
        """Callback: l'inversion d'axes a changé"""
        
        # Recalculer la cinématique
        self._update_kinematics()
        

    
    def on_home_position_changed(self):
        """Callback: la position home a changé"""
        # Mettre à jour le widget si nécessaire
        pass
    
    def on_tcp_pose_changed(self):
        """Callback: la pose TCP non corrigée a changé"""
        self._update_result_display()
    
    def on_corrected_tcp_pose_changed(self):
        """Callback: la pose TCP corrigée a changé"""
        self._update_result_display()
    
    def on_pose_deviation_changed(self):
        """Callback: la déviation de pose a changé"""
        self._update_result_display()
    
    def on_measurements_changed(self):
        """Callback: les mesures ont changé"""
        # Mettre à jour l'affichage des mesures
        measurements = self.robot_model.get_measurements()
        # TODO: Mettre à jour le widget
    
    def on_measurement_points_changed(self):
        """Callback: les points de mesure ont changé"""
        # Mettre à jour l'affichage des points
        points = self.robot_model.get_measurement_points()
        # TODO: Mettre à jour le widget
    
    # ============================================================================
    # RÉGION: Méthodes utilitaires internes
    # ============================================================================
    
    def _update_kinematics(self):
        """Recalcule la cinématique directe et met à jour les poses TCP"""
        dh_matrices, corrected_matrices, dh_pose, corrected_pose, deviation = compute_forward_kinematics(self.robot_model)
        # Mettre à jour le modèle
        self.robot_model.set_tcp_pose(dh_pose)
        self.robot_model.set_corrected_tcp_pose(corrected_pose)
        
        # Mettre à jour la visualisation
        self.visualization_widget.update_robot_poses(corrected_matrices)
    
    def _update_result_display(self):
        """Met à jour l'affichage des résultats"""
        tcp_pose = self.robot_model.get_tcp_pose()
        corrected_tcp_pose = self.robot_model.get_corrected_tcp_pose()
        deviation = self.robot_model.get_pose_deviation()
        
        self.result_widget.update_results(tcp_pose, corrected_tcp_pose, deviation)
    
    def _update_all_widgets_from_model(self):
        """Met à jour tous les widgets depuis le modèle (après chargement config)"""
        # Nom du robot
        self.dh_widget.set_robot_name(self.robot_model.get_robot_name())
        self.measurement_widget.label_robot_name_me.setText(self.robot_model.get_robot_name())
        
        # Paramètres DH
        dh_params = self.robot_model.get_dh_params()
        dh_params_str = [[str(val) for val in row] for row in dh_params]
        self.dh_widget.set_dh_params(dh_params_str)
        
        # Corrections
        corrections = self.robot_model.get_corrections()
        corrections_str = [[str(val) for val in row] for row in corrections]
        self.correction_widget.set_corrections(corrections_str)
        
        # Joints
        joint_values = self.robot_model.get_all_joint_values()
        self.joint_widget.set_all_joints(joint_values)
        
        # Limites
        limits = self.robot_model.get_axis_limits()
        self.joint_widget.update_axis_limits(limits)
        
        # Recalculer la cinématique
        self._update_kinematics()
    
    def _load_robot_cad(self):
        """Charge le modèle CAD du robot"""
        joint_values = self.robot_model.get_all_reel_joint_values()
        dh_params = self.robot_model.get_dh_params()
        
        # Calculer les matrices de transformation
        matrices = self.kinematics_engine.compute_all_frames(joint_values, dh_params)
        
        # Charger les maillages STL
        self.visualization_widget.add_robot_links(matrices)