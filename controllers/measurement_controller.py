from PyQt5.QtCore import QObject
from utils.file_io import FileIOHandler

class MeasurementController(QObject):
    """Contrôleur pour la gestion des mesures"""
    
    def __init__(self, measurement_model, correction_model, measurement_widget):
        super().__init__()
        self.measurement_model = measurement_model
        self.correction_model = correction_model
        self.measurement_widget = measurement_widget
        self.file_io = FileIOHandler()
        
        # Connecter les signaux
        self.measurement_widget.import_measurements_requested.connect(self.import_measurements)
        self.measurement_widget.set_as_reference_requested.connect(self.set_as_reference)
        self.measurement_widget.calculate_corrections_requested.connect(self.calculate_corrections)
        self.measurement_widget.repere_selected.connect(self.on_repere_selected)
        
        # Connecter les changements du modèle à la vue
        self.measurement_model.measurements_changed.connect(self.update_view_from_model)
        self.measurement_model.reference_changed.connect(self.on_reference_changed)
    
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
        repere = self.measurement_model.get_repere_by_name(repere_name)
        if repere:
            # Récupérer la matrice de rotation
            rotation_matrix = self.measurement_model.get_rotation_matrix(repere)
            
            # Afficher dans le widget
            self.measurement_widget.display_repere_data(repere, rotation_matrix)
    
    def update_view_from_model(self):
        """Met à jour la vue depuis le modèle"""
        repere_names = self.measurement_model.get_all_repere_names()
        self.measurement_widget.populate_tree(repere_names)
    
    def on_reference_changed(self, ref_name):
        """Callback quand le repère de référence change"""
        self.measurement_widget.set_reference_bold(ref_name)
