import sys
import os
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
from RobotTab.robotmodel import RobotModel
from utils.file_io import FileIOHandler
from views.main_window import MainWindow

class MGDApplication:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.load_theme()

        # ====================================================================
        # RÉGION: Création du modèle
        # ====================================================================
        self.robot_model = RobotModel()

        # ====================================================================
        # RÉGION: Création de la fenêtre principale
        # ====================================================================
        self.main_window = MainWindow(self.robot_model)

    def load_theme(self):
        try:
            with open("themes/dark_theme.qss", "r") as f:
                self.app.setStyleSheet(f.read())
        except FileNotFoundError:
            print("Fichier dark_theme.qss non trouvé, thème par défaut utilisé")

    def run(self):
        self.main_window.showMaximized()
        self.main_window.show()
        # if len(sys.argv) > 1:
        #     print(sys.argv)
        #     config_file = sys.argv[1]
        #     if os.path.exists(config_file) and config_file.endswith(".json"):
        #         print("file found:", config_file)
        #         config_file, data = FileIOHandler.load_json(config_file)
        #         # QTimer.singleShot(100, lambda: self.main_window.get_robot_window().get_robot_controller().load_data_configuration(data))

        sys.exit(self.app.exec_())

if __name__ == "__main__":
    app = MGDApplication()
    app.run()