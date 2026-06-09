from PyQt6.QtWidgets import QVBoxLayout, QWidget

from widgets.workspace_view.workspace_configuration_widget import WorkspaceConfigurationWidget


class WorkspaceView(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.configuration_widget = WorkspaceConfigurationWidget()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.addWidget(self.configuration_widget)

    def get_configuration_widget(self) -> WorkspaceConfigurationWidget:
        return self.configuration_widget
