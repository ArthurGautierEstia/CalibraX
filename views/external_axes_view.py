from PyQt6.QtWidgets import QVBoxLayout, QWidget

from widgets.external_axes_view.external_axes_panel_widget import ExternalAxesPanelWidget


class ExternalAxesView(QWidget):
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self.panel_widget = ExternalAxesPanelWidget()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.panel_widget)

    def get_panel_widget(self) -> ExternalAxesPanelWidget:
        return self.panel_widget
