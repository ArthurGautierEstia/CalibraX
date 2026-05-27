from PyQt6.QtWidgets import QVBoxLayout, QWidget

from widgets.workpiece_view.workpiece_config_widget import WorkpieceConfigWidget


class WorkpieceView(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.config_widget = WorkpieceConfigWidget()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.config_widget)

    def get_config_widget(self) -> WorkpieceConfigWidget:
        return self.config_widget
