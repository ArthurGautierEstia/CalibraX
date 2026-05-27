from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QGroupBox, QHBoxLayout, QPushButton, QScrollArea,
    QSplitter, QVBoxLayout, QWidget,
)
from PyQt6.QtCore import Qt

from widgets.workpiece_view.tooling_panel_widget import ToolingPanelWidget
from widgets.workpiece_view.workpiece_config_widget import WorkpieceConfigWidget


class WorkpieceView(QWidget):
    """Onglet Pièce : outillage + pièce + sauvegarde."""

    save_requested = pyqtSignal()
    load_requested = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setSpacing(6)
        outer.setContentsMargins(4, 4, 4, 4)

        # ── Boutons sauvegarde ────────────────────────────────────────
        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(6)

        btn_save = QPushButton("💾 Sauvegarder config pièce")
        btn_save.setToolTip("Exporte la configuration outillage + pièce vers un fichier JSON")
        btn_save.clicked.connect(self.save_requested)
        btn_load = QPushButton("📂 Charger config pièce")
        btn_load.setToolTip("Importe une configuration outillage + pièce depuis un fichier JSON")
        btn_load.clicked.connect(self.load_requested)
        btn_layout.addWidget(btn_save)
        btn_layout.addWidget(btn_load)
        btn_layout.addStretch()
        outer.addWidget(btn_row)

        # ── Splitter vertical : outillage en haut, pièce en bas ───────
        splitter = QSplitter(Qt.Orientation.Vertical)

        # Section outillage
        tooling_group = QGroupBox("Outillage")
        tooling_group_layout = QVBoxLayout(tooling_group)
        tooling_group_layout.setContentsMargins(4, 4, 4, 4)
        self.tooling_panel = ToolingPanelWidget()
        tooling_group_layout.addWidget(self.tooling_panel)
        splitter.addWidget(tooling_group)

        # Section pièce (scrollable)
        piece_group = QGroupBox("Pièce")
        piece_group_layout = QVBoxLayout(piece_group)
        piece_group_layout.setContentsMargins(4, 4, 4, 4)

        piece_scroll = QScrollArea()
        piece_scroll.setWidgetResizable(True)
        piece_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        piece_inner = QWidget()
        piece_inner_layout = QVBoxLayout(piece_inner)
        piece_inner_layout.setContentsMargins(0, 0, 0, 0)
        self.piece_config = WorkpieceConfigWidget()
        piece_inner_layout.addWidget(self.piece_config)
        piece_inner_layout.addStretch()
        piece_scroll.setWidget(piece_inner)
        piece_group_layout.addWidget(piece_scroll)

        splitter.addWidget(piece_group)
        splitter.setSizes([400, 300])

        outer.addWidget(splitter)

    # ------------------------------------------------------------------
    # Accesseurs
    # ------------------------------------------------------------------

    def get_tooling_panel(self) -> ToolingPanelWidget:
        return self.tooling_panel

    def get_piece_config(self) -> WorkpieceConfigWidget:
        return self.piece_config
