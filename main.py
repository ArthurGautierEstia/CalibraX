import sys
import os
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QIcon, QColor, QPalette

from models.robot_model import RobotModel
from controllers.main_controller import MainController
from views.main_window import MainWindow

from utils.file_io import FileIOHandler

class MGDApplication:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.apply_kuka_accent()

        currentDir = os.getcwd()
        icon_path = os.path.join(currentDir, "appicon.ico")

        self.app.setWindowIcon(QIcon(icon_path))

        #self.load_theme("themes/test.qss")

        self.robot_model = RobotModel()
        self.main_window = MainWindow()

        self.main_controller = MainController(self.robot_model, self.main_window)

    def apply_kuka_accent(self):
        """Force une couleur d'accent orange au lieu de la couleur systeme."""
        accent = QColor("#FF6F00")
        palette = self.app.palette()
        palette.setColor(QPalette.ColorRole.Highlight, accent)
        palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#FFFFFF"))
        palette.setColor(QPalette.ColorRole.Link, accent)
        palette.setColor(QPalette.ColorRole.LinkVisited, QColor("#D65E00"))
        if hasattr(QPalette.ColorRole, "Accent"):
            palette.setColor(QPalette.ColorRole.Accent, accent)
        self.app.setPalette(palette)

    def load_theme(self, file: str = "themes/dark_theme.qss"):
        try:
            with open(file, "r") as f:
                self.app.setStyleSheet(f.read())
        except FileNotFoundError:
            print(f"Fichier {file} non trouvé, thème par défaut utilisé")

    def load_data(self, config_file: str):
        config_file, data = FileIOHandler.load_json(config_file)
        if data and config_file:
            self.robot_model.load_from_dict(data, config_file)
            self.main_window.get_viewer3d().load_cad(self.robot_model)
            self.main_window.get_viewer3d().set_transparency(True)
            self.main_window.get_viewer3d().toogle_base_axis_frames()

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
