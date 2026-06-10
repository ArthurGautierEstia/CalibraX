from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QDialog, QVBoxLayout, QWidget

from widgets.program_view.program_generation_widget import ProgramGenerationWidget


class ProgramSettingsDialog(QDialog):
    """Dialog non-modale : réglages génération programme + header KRL + preview KRL."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Paramètres programme")
        self.setWindowFlags(
            Qt.WindowType.Window
            | Qt.WindowType.WindowCloseButtonHint
            | Qt.WindowType.WindowMinimizeButtonHint
        )
        self.setModal(False)
        self.setMinimumWidth(500)

        self._generation_widget = ProgramGenerationWidget()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(self._generation_widget)

    def get_generation_widget(self) -> ProgramGenerationWidget:
        return self._generation_widget
