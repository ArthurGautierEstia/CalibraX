import json
from PyQt5.QtWidgets import QFileDialog

class FileIOHandler:
    """Gestion des opérations d'import/export de fichiers"""
    
    @staticmethod
    def save_json(parent, title, data):
        """Sauvegarde des données en JSON"""
        file_name, _ = QFileDialog.getSaveFileName(parent, title, "", "JSON Files (*.json)")
        if file_name:
            with open(file_name, "w") as f:
                json.dump(data, f, indent=4)
            return file_name
        return None
    
    @staticmethod
    def load_json(parent, title):
        """Charge des données depuis un JSON"""
        file_name, _ = QFileDialog.getOpenFileName(parent, title, "", "JSON Files (*.json)")
        if file_name:
            try:
                with open(file_name, "r") as f:
                    data = json.load(f)
                return file_name, data
            except Exception as e:
                print(f"Erreur lors du chargement: {e}")
                return None, None
        return None, None
