from PyQt5.QtCore import QObject
from utils.file_io import FileIOHandler
import numpy as np
from utils.math_utils import euler_to_rotation_matrix

class MeasurementController(QObject):
    """Contrôleur pour la gestion des mesures"""
    
    def __init__(self, measurement_model, correction_model, measurement_widget, kinematics_engine):
        super().__init__()
        self.measurement_model = measurement_model
        self.correction_model = correction_model
        self.measurement_widget = measurement_widget
        self.kinematics_engine = kinematics_engine
        self.file_io = FileIOHandler()

    def setup_connections(self):
        """Configure les connexions entre la vue et le modèle"""
        # Connecter les signaux
        self.measurement_widget.import_measurements_requested.connect(self.import_measurements)
        self.measurement_widget.set_as_reference_requested.connect(self.set_as_reference)
        self.measurement_widget.calculate_corrections_requested.connect(self.calculate_corrections)
        self.measurement_widget.repere_selected.connect(self.on_repere_selected)
        self.measurement_widget.display_mode_changed.connect(self.on_combo_box_1_changed)
        self.measurement_widget.rotation_type_changed.connect(self.on_combo_box_2_changed)
        self.measurement_widget.clear_measurements_requested.connect(self.measurement_model.clear)

        # Connecter les changements du modèle à la vue
        self.measurement_model.measurements_changed.connect(self.update_view_from_model)
        self.measurement_model.reference_changed.connect(self.measurement_widget.set_reference_bold)
    
    def import_measurements(self):
        """Importe les mesures depuis un fichier JSON"""
        file_name, data = self.file_io.load_json(
            self.measurement_widget,
            "Importer JSON"
        )
        if data:
            self.measurement_model.load_measurements(data)
            print(f"Mesures importées: {file_name}")
    
    def set_as_reference(self):
        """Définit le repère sélectionné comme référence"""
        selected_name = self.measurement_widget.get_current_repere_name()
        if selected_name:
            self.measurement_model.set_reference(selected_name)
            print(f"Repère de référence: {selected_name}")
    
    def calculate_corrections(self):
        """Calcule les corrections basées sur les mesures"""
        self.correction_model.calculate_corrections()
    
    def on_repere_selected(self, repere_name):
        """Callback quand un repère est sélectionné dans l'arbre"""
        measured_repere = self.measurement_model.get_repere_by_name(repere_name)
        T_measured = self.measurement_model.repere_to_matrix(measured_repere)
        print(f"repère mesuré matrice:\n{T_measured}")
        T_dh = self.kinematics_engine.get_matrix_at_joint(int(repere_name.split('_')[-1]))
        print(f"Repère DH: \n{T_dh}")

        # Calcul de la transformation relative
        T_DH_inv = np.linalg.inv(T_dh)
        delta_T = np.dot(T_DH_inv, T_measured)

        # Afficher dans le widget
        self.measurement_widget.display_repere_data(delta_T)
        print("Matrice de différence :\n", delta_T)

    def update_view_from_model(self):
        """Met à jour la vue depuis le modèle"""
        repere_names = self.measurement_model.get_all_repere_names()
        self.measurement_widget.populate_tree(repere_names)
    
    def on_reference_changed(self, ref_name):
        """Callback quand le repère de référence change"""
        self.measurement_widget.set_reference_bold(ref_name)

    def on_combo_box_1_changed(self, display_mode):
        """Callback quand les options d'affichage changent"""
        self.measurement_model.set_display_mode(display_mode)

    def on_combo_box_2_changed(self, rotation_type):
        """Callback quand le type de rotation change"""
        if (rotation_type == "EulerXYZ"):
            
            self.measurement_model.set_rotation_type(rotation_type)