from PyQt6.QtWidgets import QVBoxLayout, QWidget

from widgets.cartesian_control_view.mgi_solutions_widget import MgiSolutionsWidget


class MgiView(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.mgi_solutions_widget = MgiSolutionsWidget()
        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.addWidget(self.mgi_solutions_widget)

    def get_mgi_solutions_widget(self) -> MgiSolutionsWidget:
        return self.mgi_solutions_widget
