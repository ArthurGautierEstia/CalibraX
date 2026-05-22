from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class MachiningActionsWidget(QWidget):
    """Bouton de déclenchement de la simulation d'usinage et zone de messages."""

    simulate_requested = pyqtSignal()

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self.btn_simulate = QPushButton("Simuler les efforts d'usinage")
        self._issue_label = QLabel()
        self._warning_label = QLabel()
        self._status_label = QLabel()

        self._issue_label.setWordWrap(True)
        self._warning_label.setWordWrap(True)
        self._status_label.setWordWrap(True)

        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        btn_row = QHBoxLayout()
        btn_row.addWidget(self.btn_simulate)
        btn_row.addStretch()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.addLayout(btn_row)
        layout.addWidget(self._status_label)
        layout.addWidget(self._issue_label)
        layout.addWidget(self._warning_label)

    def _setup_connections(self) -> None:
        self.btn_simulate.clicked.connect(self.simulate_requested)

    # ------------------------------------------------------------------
    # API publique
    # ------------------------------------------------------------------

    def set_simulate_enabled(self, enabled: bool) -> None:
        self.btn_simulate.setEnabled(enabled)

    def set_issue_messages(self, messages: list[str]) -> None:
        if messages:
            self._issue_label.setText("\n".join(messages))
            self._issue_label.setStyleSheet("color: #d9534f; font-weight: bold;")
        else:
            self._issue_label.clear()
            self._issue_label.setStyleSheet("")

    def set_warning_messages(self, messages: list[str]) -> None:
        if messages:
            self._warning_label.setText("\n".join(messages))
            self._warning_label.setStyleSheet("color: #f0ad4e; font-weight: bold;")
        else:
            self._warning_label.clear()
            self._warning_label.setStyleSheet("")

    def set_status_text(self, text: str) -> None:
        self._status_label.setText(text)
        self._status_label.setStyleSheet("")
