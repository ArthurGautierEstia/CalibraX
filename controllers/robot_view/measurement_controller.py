from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from widgets.calibration_view.measurement_widget import MeasurementWidget

from utils.file_io import FileIOHandler
import utils.math_utils as math_utils
import numpy as np
import os

class MeasurementController(QObject):
    def __init__(self, robot_model: RobotModel, measurement_widget: MeasurementWidget, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.measurement_widget = measurement_widget
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        # Signals from Robot Model
        self.robot_model.measurements_changed.connect(self._on_robot_measurements_changed)
        self.robot_model.measurements_points_changed.connect(self._on_robot_measurements_points_changed)

        # Signals from View
        self.measurement_widget.import_measurements_requested.connect(self._on_view_import_measurements_requested)
        self.measurement_widget.clear_measurements_requested.connect(self._on_view_clear_measurements_requested)
        self.measurement_widget.calculate_corrections_requested.connect(self._on_view_calculate_corrections_requested)
        self.measurement_widget.set_as_reference_requested.connect(self._on_view_set_as_reference_requested)
        self.measurement_widget.repere_selected.connect(self._on_view_repere_selected)
        self.measurement_widget.display_mode_changed.connect(self._on_view_display_mode_changed)
        self.measurement_widget.rotation_type_changed.connect(self._on_view_rotation_type_changed)

    # ======
    # Connection callbacks
    # ======

    def _on_robot_measurements_changed(self) -> None:
        self.measurement_widget.set_measure_filename(self.robot_model.get_measurements_filename())

    def _on_robot_measurements_points_changed(self) -> None:
        self.measurement_widget.set_measurements_data(self.robot_model.get_measurements())

    def _on_view_import_measurements_requested(self) -> None:
        currentDir = os.getcwd()
        configurationDir = os.path.join(currentDir, 'configurations') 

        file_path, data = FileIOHandler.select_and_load_json(
            self.measurement_widget,
            "Importer un ficher de mesures",
            configurationDir if os.path.exists(configurationDir) else currentDir
        )
        if data:
            # Si data est une liste de mesures (repères)
            if isinstance(data, list):
                self.robot_model.blockSignals(True)

                # Ajouter chaque mesure au modèle
                for measurement in data:
                    self.robot_model.add_measurement(measurement)
                
                # Stocker les mesures dans le widget
                self.measurement_widget.set_measurements_data(data)

                # Afficher les repères dans le widget de mesure
                repere_names = [m.get("name", f"Repère {i}") for i, m in enumerate(data)]
                self.measurement_widget.populate_tree(repere_names)

                # Afficher le nom du fichier de mesure
                file_name = file_path.split("/")[-1]
                file_name = file_name.replace(".json","")
                self.measurement_widget.set_measure_filename(file_name)

                self.robot_model.blockSignals(False)

    def _on_view_clear_measurements_requested(self) -> None:
        self.robot_model.clear_measurements()
        self.robot_model.clear_measurement_points()
        self.measurement_widget.clear_measurements()

    def _on_view_calculate_corrections_requested(self) -> None:
        # TODO: Implémenter le calcul de corrections
        pass
    
    def _on_view_set_as_reference_requested(self) -> None:
        # TODO: Implémenter la définition de référence
        pass
    
    def _on_view_repere_selected(self, repere_name: str) -> None:
        # Chercher la mesure correspondante dans le widget
        for i, measurement in enumerate(self.measurement_widget.measurements):
            if measurement.get("name") == repere_name:
                # Récupérer le mode d'affichage actuel
                display_mode = self.measurement_widget.display_mode.currentText()
                
                if display_mode == "Repères":
                    # Afficher les données du repère mesuré
                    self.measurement_widget.display_measurement(measurement)
                else:  # display_mode == "Ecarts"
                    # Afficher les écarts entre le repère mesuré et le repère DH théorique
                    self._display_measurement_deviations(i, measurement)
                break
    
    def _on_view_display_mode_changed(self, mode: str) -> None:
        # Récupérer le repère actuellement sélectionné dans l'arbre
        current_item = self.measurement_widget.tree.currentItem()
        if current_item:
            repere_name = current_item.text(0)
            # Réafficher les données avec le nouveau mode
            self._on_view_repere_selected(repere_name)

    def _on_view_rotation_type_changed(self, rotation_type: str) -> None:
        # TODO: Implémenter le changement de type de rotation
        pass

    def _display_measurement_deviations(self, measurement_index: int, measurement):
        """Affiche les écarts entre un repère mesuré et le repère DH théorique correspondant"""
        try:
            # Extraire les coordonnées et angles du repère mesuré
            x = float(measurement.get("X", 0))
            y = float(measurement.get("Y", 0))
            z = float(measurement.get("Z", 0))
            a = float(measurement.get("A", 0))
            b = float(measurement.get("B", 0))
            c = float(measurement.get("C", 0))
            
            # Créer la matrice homogène du repère mesuré
            R_measured = math_utils.euler_to_rotation_matrix(a, b, c, degrees=True)
            T_measured = np.eye(4)
            T_measured[:3, :3] = R_measured
            T_measured[:3, 3] = [x, y, z]
            
            # Récupérer les matrices DH théoriques
            # dh_matrices, _, _, _, _ = math_utils.compute_forward_kinematics(self.robot_model)
            dh_matrices, _, _, _, _ = self.robot_model.compute_fk_joints(self.robot_model.get_joints())
            
            # S'assurer que l'index est valide
            if measurement_index >= len(dh_matrices):
                print(f"Erreur: index de mesure {measurement_index} hors limites")
                return
            
            # Récupérer la matrice DH théorique correspondante
            T_theoretical = dh_matrices[measurement_index]
            
            # Calculer l'écart (delta_T) = T_measured * inv(T_theoretical)
            T_theoretical_inv = np.linalg.inv(T_theoretical)
            delta_T = T_measured @ T_theoretical_inv
            
            # Afficher les écarts dans la table
            self.measurement_widget.display_repere_data(delta_T)
            
        except (ValueError, TypeError, np.linalg.LinAlgError) as e:
            print(f"Erreur lors du calcul des écarts: {e}")
