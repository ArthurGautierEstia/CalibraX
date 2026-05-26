from PyQt6.QtWidgets import QVBoxLayout, QWidget

from widgets.external_axes_view.external_axes_panel_widget import ExternalAxesPanelWidget


class ExternalAxesView(QWidget):
    def __init__(self, parent: QWidget = None):
        super().__init__(parent)
        self.panel_widget = ExternalAxesPanelWidget()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.panel_widget)

    def get_panel_widget(self) -> ExternalAxesPanelWidget:
        return self.panel_widget
