from PyQt6.QtWidgets import QVBoxLayout, QWidget

from widgets.tool_view.tool_configuration_widget import ToolConfigurationWidget


class ToolView(QWidget):
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self.configuration_widget = ToolConfigurationWidget()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        layout.addWidget(self.configuration_widget)

    def get_configuration_widget(self) -> ToolConfigurationWidget:
        return self.configuration_widget
