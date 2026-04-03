from PyQt6.QtWidgets import QWidget, QVBoxLayout, QScrollArea

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from widgets.trajectory_view.trajectory_config_widget import TrajectoryConfigWidget
from widgets.trajectory_view.trajectory_actions_widget import TrajectoryActionsWidget
from widgets.trajectory_view.trajectory_graphs_widget import TrajectoryGraphsWidget


class TrajectoryView(QWidget):
    """Main view for trajectory creation, playback, and graphs."""

    def __init__(self, robot_model: RobotModel, tool_model: ToolModel, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.config_widget = TrajectoryConfigWidget(robot_model, tool_model)
        self.actions_widget = TrajectoryActionsWidget()
        self.graphs_widget = TrajectoryGraphsWidget()

        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)

        scroll_area = QScrollArea(self)
        scroll_area.setWidgetResizable(True)

        content = QWidget(scroll_area)
        content_layout = QVBoxLayout(content)
        content_layout.addWidget(self.config_widget)
        content_layout.addWidget(self.actions_widget)
        content_layout.addWidget(self.graphs_widget)
        content_layout.addStretch()

        scroll_area.setWidget(content)
        layout.addWidget(scroll_area)

    def get_config_widget(self) -> TrajectoryConfigWidget:
        return self.config_widget

    def get_actions_widget(self) -> TrajectoryActionsWidget:
        return self.actions_widget

    def get_graphs_widget(self) -> TrajectoryGraphsWidget:
        return self.graphs_widget
