from PyQt6.QtWidgets import QVBoxLayout, QWidget

from widgets.cartesian_control_view.cartesian_control_widget import CartesianControlWidget


class CartesianControlView(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.cartesian_control_widget = CartesianControlWidget()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.addWidget(self.cartesian_control_widget)

    def get_cartesian_control_widget(self) -> CartesianControlWidget:
        return self.cartesian_control_widget
