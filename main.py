import sys
from PyQt5.QtWidgets import QApplication
from views.main_window import MainWindow
from models.robot_model import RobotModel
from models.kinematics_engine import KinematicsEngine
from models.measurement_model import MeasurementModel
from models.correction_model import CorrectionModel
from controllers.robot_controller import RobotController, JointController, ResultController
from controllers.visualization_controller import VisualizationController
from controllers.measurement_controller import MeasurementController

class MGDApplication:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.load_theme()

        self.window = MainWindow()

        # Modèles
        self.robot_model = RobotModel()
        self.kinematics_engine = KinematicsEngine(self.robot_model)
        self.measurement_model = MeasurementModel()
        self.correction_model = CorrectionModel(
            self.robot_model,
            self.kinematics_engine,
            self.measurement_model,
        )

        # Contrôleurs
        self.visualization_controller = VisualizationController(
            self.kinematics_engine,
            self.robot_model,
            self.window.get_viewer_widget(),
        )

        self.robot_controller = RobotController(
            self.robot_model,
            self.kinematics_engine,
            self.window.get_dh_widget(),
            self.window.get_correction_widget(),
            self.visualization_controller,
        )

        self.joint_controller = JointController(
            self.robot_model,
            self.window.get_joint_widget(),
            self.visualization_controller,
        )

        self.result_controller = ResultController(
            self.kinematics_engine,
            self.window.get_result_widget(),
        )

        self.measurement_controller = MeasurementController(
            self.measurement_model,
            self.correction_model,
            self.window.get_measurement_widget(),
            self.kinematics_engine,
        )

        # Chaque contrôleur se câble lui-même à ses signaux
        self.robot_controller.setup_connections()
        self.visualization_controller.setup_connections()
        self.measurement_controller.setup_connections()

    def load_theme(self):
        try:
            with open("dark_theme.qss", "r") as f:
                self.app.setStyleSheet(f.read())
        except FileNotFoundError:
            print("Fichier dark_theme.qss non trouvé, thème par défaut utilisé")

    def run(self):
        self.window.showMaximized()
        self.window.show()
        sys.exit(self.app.exec_())



if __name__ == "__main__":
    app = MGDApplication()
    app.run()
