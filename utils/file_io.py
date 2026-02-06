import json
from PyQt6.QtWidgets import QWidget, QFileDialog
from typing import Any

class FileIOHandler:
    """Gestion des opérations d'import/export de fichiers"""
    
    @staticmethod
    def save_json(parent: QWidget, title: str, data: dict[str, Any], directory: str=None):
        """Sauvegarde des données en JSON"""
        file_name, _ = QFileDialog.getSaveFileName(parent, title, directory, "JSON Files (*.json)")
        if file_name:
            with open(file_name, "w") as f:
                json.dump(data, f, indent=4)
            return file_name
        return None
    
    @staticmethod
    def select_file(parent: QWidget, title: str=None, directory: str=None, filter: str=None):
        """Ouvre une boîte de dialogue pour sélectionner un fichier"""
        file_name, _ = QFileDialog.getOpenFileName(parent, title, directory, filter)
        return file_name

    @staticmethod
    def load_json(file_name: str):
        """Charge des données depuis un JSON"""
        if file_name:
            try:
                with open(file_name, "r") as f:
                    data = json.load(f)
                return file_name, data
            except Exception as e:
                print(f"Erreur lors du chargement: {e}")
                return None, None
        return None, None
    
    @staticmethod
    def select_and_load_json(parent: QWidget, title: str=None, directory: str=None):
        """Sélectionne et charge un fichier JSON"""
        return FileIOHandler.load_json(FileIOHandler.select_file(parent, title, directory, "JSON Files (*.json)"))
