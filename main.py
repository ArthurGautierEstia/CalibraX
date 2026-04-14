import argparse
import os
import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QColor, QIcon, QPalette
from PyQt6.QtWidgets import QApplication

from controllers.main_controller import MainController
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from views.main_window import MainWindow


def parse_startup_options(argv: list[str]) -> dict[str, str]:
    parser = argparse.ArgumentParser(description="CalibraX")
    parser.add_argument("--config", dest="config_path", help="Chemin vers une configuration robot JSON.")
    parser.add_argument("--tool", dest="tool_path", help="Chemin vers un profil tool JSON.")
    parser.add_argument("--workspace", dest="workspace_path", help="Chemin vers un workspace JSON.")
    parser.add_argument(
        "--session",
        dest="session_path",
        default=MainController.DEFAULT_SESSION_FILE,
        help="Chemin vers le fichier de session applicative JSON.",
    )
    args = parser.parse_args(argv)

    return {
        "config": args.config_path or "",
        "tool": args.tool_path or "",
        "workspace": args.workspace_path or "",
        "session": args.session_path or MainController.DEFAULT_SESSION_FILE,
    }


class CalibraxApplication:
    def __init__(self, startup_options: dict[str, str]):
        self.app = QApplication(sys.argv)
        self.apply_kuka_accent()

        current_dir = os.getcwd()
        icon_path = os.path.join(current_dir, "appicon.ico")
        self.app.setWindowIcon(QIcon(icon_path))

        self.robot_model = RobotModel()
        self.tool_model = ToolModel()
        self.workspace_model = WorkspaceModel()
        self.main_window = MainWindow(self.robot_model, self.tool_model, self.workspace_model)
        self.main_controller = MainController(
            self.robot_model,
            self.tool_model,
            self.workspace_model,
            self.main_window,
            startup_options=startup_options,
        )

        self.app.aboutToQuit.connect(self.main_controller.flush_session)

    def apply_kuka_accent(self):
        """Force une couleur d'accent orange au lieu de la couleur système."""
        accent = QColor("#FF6F00")
        palette = self.app.palette()
        palette.setColor(QPalette.ColorRole.Highlight, accent)
        palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#FFFFFF"))
        palette.setColor(QPalette.ColorRole.Link, accent)
        palette.setColor(QPalette.ColorRole.LinkVisited, QColor("#D65E00"))
        if hasattr(QPalette.ColorRole, "Accent"):
            palette.setColor(QPalette.ColorRole.Accent, accent)
        self.app.setPalette(palette)

    def run(self):
        self.main_window.showMaximized()
        QTimer.singleShot(0, self.main_controller.bootstrap_startup)
        sys.exit(self.app.exec())


if __name__ == "__main__":
    startup_options = parse_startup_options(sys.argv[1:])
    app = CalibraxApplication(startup_options)
    app.run()
