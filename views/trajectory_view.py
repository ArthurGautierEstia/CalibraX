from PyQt6.QtWidgets import QWidget, QVBoxLayout

from models.robot_model import RobotModel
from widgets.trajectory_view.trajectory_config_widget import TrajectoryConfigWidget
from widgets.trajectory_view.trajectory_actions_widget import TrajectoryActionsWidget
from widgets.trajectory_view.trajectory_graphs_widget import TrajectoryGraphsWidget


class TrajectoryView(QWidget):
    """Main view for trajectory creation, playback, and graphs."""

    def __init__(self, robot_model: RobotModel, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.config_widget = TrajectoryConfigWidget(robot_model)
        self.actions_widget = TrajectoryActionsWidget()
        self.graphs_widget = TrajectoryGraphsWidget()

        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.addWidget(self.config_widget)
        layout.addWidget(self.actions_widget)
        layout.addWidget(self.graphs_widget)
        layout.addStretch()

    def get_config_widget(self) -> TrajectoryConfigWidget:
        return self.config_widget

    def get_actions_widget(self) -> TrajectoryActionsWidget:
        return self.actions_widget

    def get_graphs_widget(self) -> TrajectoryGraphsWidget:
        return self.graphs_widget
