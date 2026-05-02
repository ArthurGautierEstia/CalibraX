from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QMainWindow, QSizePolicy, QSplitter, QTabWidget, QVBoxLayout, QWidget

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from views.calibration_view import CalibrationView
from views.cartesian_control_view import CartesianControlView
from views.joint_control_view import JointControlView
from views.robot_view import RobotView
from views.tool_view import ToolView
from views.trajectory_view import TrajectoryView
from views.workspace_view import WorkspaceView
from widgets.viewer_3d_widget import Viewer3DWidget


class MainWindow(QMainWindow):
    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        parent: QWidget = None,
    ):
        super().__init__(parent)
        self.setWindowTitle("Calibrax")

        self.tabs = QTabWidget()
        self.main_splitter: QSplitter | None = None
        self._initial_splitter_sizes_applied = False

        self.robot_view = RobotView()
        self.tool_view = ToolView()
        self.workspace_view = WorkspaceView()
        self.calibration_view = CalibrationView()
        self.joint_control_view = JointControlView()
        self.cartesian_control_view = CartesianControlView()
        self.trajectory_view = TrajectoryView(robot_model, tool_model, workspace_model)

        self.viewer3d = Viewer3DWidget()

        self._setup_ui()

    def _setup_ui(self) -> None:
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        self.tabs.addTab(self.robot_view, "Robot")
        self.tabs.addTab(self.tool_view, "Tool")
        self.tabs.addTab(self.calibration_view, "Calibration")
        self.tabs.addTab(self.workspace_view, "Workspace")
        self.tabs.addTab(self.trajectory_view, "Trajectoire")

        self.robot_view.get_configuration_widget().add_tab(self.cartesian_control_view, "MGI")

        self.main_splitter = QSplitter(Qt.Orientation.Horizontal, central_widget)
        self.main_splitter.setHandleWidth(6)

        self.tabs.setMinimumWidth(0)
        self.tabs.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Expanding)
        self.viewer3d.setMinimumWidth(0)
        self.viewer3d.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.main_splitter.addWidget(self.tabs)
        self.main_splitter.addWidget(self.viewer3d)
        self.main_splitter.setStretchFactor(0, 1)
        self.main_splitter.setStretchFactor(1, 1)
        self.main_splitter.setChildrenCollapsible(False)

        layout = QVBoxLayout(central_widget)
        layout.addWidget(self.main_splitter)

    def showEvent(self, event) -> None:
        super().showEvent(event)
        self._apply_initial_splitter_sizes()

    def _apply_initial_splitter_sizes(self) -> None:
        if self._initial_splitter_sizes_applied or self.main_splitter is None:
            return
        total_width = max(2, self.main_splitter.size().width())
        left_width = total_width // 2
        right_width = total_width - left_width
        self.main_splitter.setSizes([left_width, right_width])
        self._initial_splitter_sizes_applied = True

    ####################
    # VIEW GETTERS
    ####################

    def get_robot_view(self) -> RobotView:
        """Retourne la vue de configuration du robot"""
        return self.robot_view

    def get_calibration_view(self) -> CalibrationView:
        """Retourne la vue de calibration du robot"""
        return self.calibration_view

    def get_tool_view(self) -> ToolView:
        """Retourne la vue de configuration du tool."""
        return self.tool_view

    def get_workspace_view(self) -> WorkspaceView:
        """Retourne la vue workspace."""
        return self.workspace_view

    def get_joint_control_view(self) -> JointControlView:
        """Retourne la vue de controle articulaire"""
        return self.joint_control_view

    def get_cartesian_control_view(self) -> CartesianControlView:
        """Retourne la vue de controle cartesien"""
        return self.cartesian_control_view

    def get_trajectory_view(self) -> TrajectoryView:
        """Retourne la vue de trajectoire"""
        return self.trajectory_view

    def get_viewer3d(self) -> Viewer3DWidget:
        """Retourne la vue du viewer 3D"""
        return self.viewer3d

    #####################
    # Functions
    #####################

    def update_enabled_tabs(self, robot_has_configuration: bool) -> None:
        """Active ou desactive les onglets de controle en fonction de la configuration du robot"""
        for control_view in (
            self.trajectory_view,
        ):
            tab_index = self.tabs.indexOf(control_view)
            if tab_index >= 0:
                self.tabs.setTabEnabled(tab_index, robot_has_configuration)
        self.robot_view.get_configuration_widget().set_tab_enabled("MGI", robot_has_configuration)
