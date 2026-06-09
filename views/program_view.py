from PyQt6.QtWidgets import QScrollArea, QVBoxLayout, QWidget

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.workspace_model import WorkspaceModel
from widgets.program_view.program_actions_widget import ProgramActionsWidget
from widgets.program_view.program_config_widget import ProgramConfigWidget
from widgets.program_view.program_generation_widget import ProgramGenerationWidget
from widgets.program_view.program_graphs_widget import ProgramGraphsWidget
from widgets.program_view.program_keypoints_widget import ProgramKeypointsWidget


class ProgramView(QWidget):
    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        parent: QWidget = None,
    ) -> None:
        super().__init__(parent)
        self.header_widget = ProgramConfigWidget()
        self.config_widget = ProgramKeypointsWidget(robot_model, tool_model, workspace_model)
        self.actions_widget = ProgramActionsWidget()
        self.generation_widget = ProgramGenerationWidget()
        self.graphs_widget = ProgramGraphsWidget()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        scroll_area = QScrollArea(self)
        scroll_area.setWidgetResizable(True)

        content = QWidget(scroll_area)
        content_layout = QVBoxLayout(content)
        content_layout.addWidget(self.header_widget)
        content_layout.addWidget(self.config_widget)
        content_layout.addWidget(self.actions_widget)
        content_layout.addWidget(self.generation_widget)
        content_layout.addWidget(self.graphs_widget)
        content_layout.addStretch()

        scroll_area.setWidget(content)
        layout.addWidget(scroll_area)

    def get_header_widget(self) -> ProgramConfigWidget:
        return self.header_widget

    def get_config_widget(self) -> ProgramKeypointsWidget:
        return self.config_widget

    def get_actions_widget(self) -> ProgramActionsWidget:
        return self.actions_widget

    def get_generation_widget(self) -> ProgramGenerationWidget:
        return self.generation_widget

    def get_graphs_widget(self) -> ProgramGraphsWidget:
        return self.graphs_widget
