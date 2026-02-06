import sys
import os
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

from models.robot_model import RobotModel
from controllers.main_controller import MainController
from views.main_window import MainWindow

from utils.file_io import FileIOHandler

class MGDApplication:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.load_theme()

        self.robot_model = RobotModel()
        self.main_window = MainWindow()
        self.main_controller = MainController(self.robot_model, self.main_window)

    def load_theme(self):
        try:
            with open("themes/dark_theme.qss", "r") as f:
                self.app.setStyleSheet(f.read())
        except FileNotFoundError:
            print("Fichier dark_theme.qss non trouvé, thème par défaut utilisé")

    def load_data(self, config_file: str):
        config_file, data = FileIOHandler.load_json(config_file)
        if data:
            self.robot_model.load_from_dict(data, config_file)
            self.main_window.viewer3d.load_cad(self.robot_model)
            self.main_window.viewer3d.set_transparency(True)

    def run(self):
        self.main_window.showMaximized()
        if len(sys.argv) > 1:
            config_file = sys.argv[1]
            if os.path.exists(config_file) and config_file.endswith(".json"):
                QTimer.singleShot(100, lambda: self.load_data(config_file))

        sys.exit(self.app.exec())

if __name__ == "__main__":
    app = MGDApplication()
    app.run()