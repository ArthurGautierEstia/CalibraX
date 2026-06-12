import argparse
import os
import sys

from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QIcon, QSurfaceFormat
from PyQt6.QtWidgets import QApplication

_fmt = QSurfaceFormat()
_fmt.setSamples(4)
_fmt.setDepthBufferSize(24)
QSurfaceFormat.setDefaultFormat(_fmt)

from controllers.main_controller import MainController
from models.camera_model import CameraModel
from models.external_axes_model import ExternalAxesModel
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.tooling_model import ToolingModel
from models.workspace_model import WorkspaceModel
from models.workpiece_model import WorkpieceModel
from utils.user_data_paths import ensure_user_data_directories
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
        ensure_user_data_directories()

        current_dir = os.getcwd()
        icon_path = os.path.join(current_dir, "appicon.ico")
        self.app.setWindowIcon(QIcon(icon_path))

        self.robot_model = RobotModel()
        self.tool_model = ToolModel()
        self.workspace_model = WorkspaceModel()
        self.camera_model = CameraModel()
        self.external_axes_model = ExternalAxesModel()
        self.workpiece_model = WorkpieceModel()
        self.tooling_model = ToolingModel()
        self.main_window = MainWindow(self.robot_model, self.tool_model, self.workspace_model)
        self.main_controller = MainController(
            self.robot_model,
            self.tool_model,
            self.workspace_model,
            self.camera_model,
            self.external_axes_model,
            self.workpiece_model,
            self.tooling_model,
            self.main_window,
            startup_options=startup_options,
            trajectory_benchmark_verbose=True,
            validity_pool_size=1,
        )

        self.app.aboutToQuit.connect(self.main_controller.shutdown)

    def run(self):
        self.main_window.show_maximized_on_startup()
        QTimer.singleShot(0, self.main_controller.bootstrap_startup)
        sys.exit(self.app.exec())


if __name__ == "__main__":
    startup_options = parse_startup_options(sys.argv[1:])
    app = CalibraxApplication(startup_options)
    app.run()
