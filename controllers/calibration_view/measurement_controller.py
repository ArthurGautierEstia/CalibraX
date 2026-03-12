from PyQt6.QtCore import QObject
from PyQt6.QtWidgets import QFileDialog

from models.robot_model import RobotModel
from widgets.calibration_view.measurement_widget import MeasurementWidget

from utils.file_io import FileIOHandler
import utils.math_utils as math_utils
import numpy as np
import os
import csv

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
        self.measurement_widget.apply_parameters_requested.connect(self._on_view_apply_parameters_requested)
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

        # Utiliser le dialogue de sélection de fichier pour CSV ou JSON
        file_path, _ = QFileDialog.getOpenFileName(
            self.measurement_widget,
            "Importer un fichier de mesures",
            configurationDir if os.path.exists(configurationDir) else currentDir,
            "Fichiers supportés (*.csv *.json);;Fichiers CSV (*.csv);;Fichiers JSON (*.json)"
        )
        
        if not file_path:
            return

        try:
            if file_path.endswith('.csv'):
                data = self._parse_csv_measurements(file_path)
            elif file_path.endswith('.json'):
                data, _ = FileIOHandler.select_and_load_json(
                    self.measurement_widget,
                    "Importer un fichier de mesures",
                    configurationDir if os.path.exists(configurationDir) else currentDir
                )
            else:
                return

            if data and isinstance(data, list):
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
                file_name = file_name.split("\\")[-1]  # Support Windows paths
                file_name = file_name.replace(".csv", "").replace(".json", "")
                self.measurement_widget.set_measure_filename(file_name)

                self.robot_model.blockSignals(False)
        except Exception as e:
            print(f"Erreur lors de l'import du fichier: {e}")

    def _parse_csv_measurements(self, file_path: str) -> list:
        """
        Parse un fichier CSV contenant des repères avec le format :
        // Col::Object,X,Y,Z,Rx - Fixed XYZ  (deg),Ry - Fixed XYZ  (deg),Rz - Fixed XYZ  (deg),[Timestamp]
        Mesure::World_Frame,0.0000,0.0000,0.0000,-0.0000,-0.0000,0.0000
        Geo robot::R2,...
        ...
        Retourne une liste de dictionnaires contenant les mesures (World_Frame + R2 à R6)
        """
        measurements = []
        
        try:
            with open(file_path, 'r', encoding='utf-8') as csvfile:
                # Sauter la première ligne (commentaire avec convention)
                next(csvfile)
                
                reader = csv.reader(csvfile)
                
                for row_idx, row in enumerate(reader):
                    if len(row) < 7:
                        continue
                    
                    try:
                        # Parser la ligne
                        # Format : Geo robot::R2,318.9551,350.0166,674.8630,-61.5471,-89.9484,61.5471
                        label = row[0].strip()
                        
                        # Extraire le nom du repère (ex: "R2" de "Geo robot::R2" ou "World_Frame")
                        repere_name = label.split("::")[-1] if "::" in label else label
                        
                        # Extraire les coordonnées et angles
                        x = float(row[1])
                        y = float(row[2])
                        z = float(row[3])
                        rx = float(row[4])  # A - Rotation autour de X (Fixed XYZ)
                        ry = float(row[5])  # B - Rotation autour de Y (Fixed XYZ)
                        rz = float(row[6])  # C - Rotation autour de Z (Fixed XYZ)
                        
                        # Créer le dictionnaire de mesure
                        measurement = {
                            "name": repere_name,
                            "X": x,
                            "Y": y,
                            "Z": z,
                            "A": rx,
                            "B": ry,
                            "C": rz
                        }
                        
                        measurements.append(measurement)
                        
                    except (ValueError, IndexError) as e:
                        print(f"Erreur lors du parsing de la ligne {row_idx}: {e}")
                        continue
        
        except IOError as e:
            print(f"Erreur lors de la lecture du fichier CSV: {e}")
            return []
        
        return measurements

    def _on_view_clear_measurements_requested(self) -> None:
        self.robot_model.clear_measurements()
        self.robot_model.clear_measurement_points()
        self.measurement_widget.clear_measurements()

    def _on_view_apply_parameters_requested(self) -> None:
        # TODO: Implémenter le calcul de corrections
        pass
    
    def _on_view_set_as_reference_requested(self) -> None:
        # Récupérer le repère sélectionné dans l'arbre
        current_item = self.measurement_widget.tree.currentItem()
        if not current_item:
            print("Aucun repère sélectionné")
            return
        
        ref_repere_name = current_item.text(0)
        
        # Chercher la mesure correspondante
        ref_measurement = None
        ref_index = None
        for i, measurement in enumerate(self.measurement_widget.measurements):
            if measurement.get("name") == ref_repere_name:
                ref_measurement = measurement
                ref_index = i
                break
        
        if ref_measurement is None:
            print(f"Repère {ref_repere_name} non trouvé")
            return
        
        try:
            # Créer la matrice homogène du repère de référence
            x_ref = float(ref_measurement.get("X", 0))
            y_ref = float(ref_measurement.get("Y", 0))
            z_ref = float(ref_measurement.get("Z", 0))
            a_ref = float(ref_measurement.get("A", 0))
            b_ref = float(ref_measurement.get("B", 0))
            c_ref = float(ref_measurement.get("C", 0))
            
            R_ref = math_utils.euler_to_rotation_matrix(a_ref, b_ref, c_ref, degrees=True)
            T_ref = np.eye(4)
            T_ref[:3, :3] = R_ref
            T_ref[:3, 3] = [x_ref, y_ref, z_ref]
            
            # Calculer l'inverse du repère de référence
            T_ref_inv = np.linalg.inv(T_ref)
            
            # Recalculer tous les repères par rapport à cette référence
            for measurement in self.measurement_widget.measurements:
                # Créer la matrice homogène du repère actuel
                x = float(measurement.get("X", 0))
                y = float(measurement.get("Y", 0))
                z = float(measurement.get("Z", 0))
                a = float(measurement.get("A", 0))
                b = float(measurement.get("B", 0))
                c = float(measurement.get("C", 0))
                
                R = math_utils.euler_to_rotation_matrix(a, b, c, degrees=True)
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = [x, y, z]
                
                # Calculer la nouvelle matrice relative au repère de référence
                T_new = T_ref_inv @ T
                
                # Extraire les nouvelles coordonnées
                new_x = T_new[0, 3]
                new_y = T_new[1, 3]
                new_z = T_new[2, 3]
                
                # Extraire les nouveaux angles (Euler ZYX en degrés)
                R_new = T_new[:3, :3]
                angles = math_utils.rotation_matrix_to_euler_zyx(R_new)
                new_a, new_b, new_c = float(angles[0]), float(angles[1]), float(angles[2])
                
                # Mettre à jour la mesure
                measurement["X"] = new_x
                measurement["Y"] = new_y
                measurement["Z"] = new_z
                measurement["A"] = new_a
                measurement["B"] = new_b
                measurement["C"] = new_c
            
            # Marquer le repère de référence comme étant la référence
            self.measurement_widget.set_reference_bold(ref_repere_name)
            
            # Réafficher le repère sélectionné actuel
            self._on_view_repere_selected(ref_repere_name)
            
            print(f"Repère '{ref_repere_name}' défini comme référence. Positions recalculées.")
            
        except (ValueError, TypeError, np.linalg.LinAlgError) as e:
            print(f"Erreur lors de la définition de la référence: {e}")
    
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
        """Gère le changement de mode d'affichage (Repères vs Ecarts)"""
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
            
            # Déterminer la matrice théorique correspondante
            repere_name = measurement.get("name", "")
            
            if repere_name == "World_Frame":
                # World_Frame est la base, la matrice théorique est l'identité
                T_theoretical = np.eye(4)
            else:
                # Pour les autres repères (R2, R3, etc.), décaler d'un indice
                # R2 correspond à dh_matrices[0], R3 à dh_matrices[1], etc.
                dh_index = measurement_index - 1
                
                # Récupérer les matrices DH théoriques
                dh_matrices, _, _, _, _ = self.robot_model.compute_fk_joints(self.robot_model.get_joints())
                
                # S'assurer que l'index est valide
                if dh_index < 0 or dh_index >= len(dh_matrices):
                    print(f"Erreur: index DH {dh_index} invalide pour le repère '{repere_name}' (mesure {measurement_index})")
                    print(f"dh_matrices contient {len(dh_matrices)} éléments")
                    return
                
                # Récupérer la matrice DH théorique correspondante
                T_theoretical = dh_matrices[dh_index]
            
            # Calculer l'écart (delta_T) = T_measured * inv(T_theoretical)
            T_theoretical_inv = np.linalg.inv(T_theoretical)
            delta_T = T_measured @ T_theoretical_inv
            
            # Afficher les écarts dans la table
            self.measurement_widget.display_repere_data(delta_T)
            
        except (ValueError, TypeError, np.linalg.LinAlgError) as e:
            print(f"Erreur lors du calcul des écarts: {e}")
