from __future__ import annotations

from PyQt6.QtWidgets import QScrollArea, QVBoxLayout, QWidget

from widgets.machining_view.machining_actions_widget import MachiningActionsWidget
from widgets.machining_view.machining_graphs_widget import MachiningGraphsWidget
from widgets.machining_view.machining_params_widget import MachiningParamsWidget


class MachiningView(QWidget):
    """Vue de simulation d'usinage robotisé (efforts de coupe, couples, déformation TCP)."""

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.params_widget = MachiningParamsWidget()
        self.actions_widget = MachiningActionsWidget()
        self.graphs_widget = MachiningGraphsWidget()

        self._setup_ui()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)

        content = QWidget(scroll)
        content_layout = QVBoxLayout(content)
        content_layout.addWidget(self.params_widget)
        content_layout.addWidget(self.actions_widget)
        content_layout.addWidget(self.graphs_widget)
        content_layout.addStretch()

        scroll.setWidget(content)
        layout.addWidget(scroll)

    def get_params_widget(self) -> MachiningParamsWidget:
        return self.params_widget

    def get_actions_widget(self) -> MachiningActionsWidget:
        return self.actions_widget

    def get_graphs_widget(self) -> MachiningGraphsWidget:
        return self.graphs_widget
