from PyQt6.QtWidgets import QVBoxLayout, QWidget

from widgets.camera_view.camera_configuration_widget import CameraConfigurationWidget


class CameraView(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.configuration_widget = CameraConfigurationWidget()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.configuration_widget, 1)

    def get_configuration_widget(self) -> CameraConfigurationWidget:
        return self.configuration_widget
