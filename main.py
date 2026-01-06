import sys
from PyQt5.QtWidgets import QApplication
from views.main_window import MainWindow

class MGDApplication:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.load_theme()

        # ====================================================================
        # RÉGION: Création de la fenêtre principale
        # ====================================================================
        self.main_window = MainWindow()

    def load_theme(self):
        try:
            with open("dark_theme.qss", "r") as f:
                self.app.setStyleSheet(f.read())
        except FileNotFoundError:
            print("Fichier dark_theme.qss non trouvé, thème par défaut utilisé")

    def run(self):
        self.main_window.showMaximized()
        self.main_window.show()
        sys.exit(self.app.exec_())


if __name__ == "__main__":
    app = MGDApplication()
    app.run()