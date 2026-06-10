from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QPalette
from PyQt6.QtWidgets import (
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton, QScrollArea,
    QSplitter, QVBoxLayout, QWidget,
)

from utils.config_action_icons import (
    CONFIG_ACTION_BUTTON_SIZE,
    CONFIG_ACTION_ICON_SIZE,
    build_new_icon,
    build_save_icon,
)
from utils.status_badge import apply_status_badge
from widgets.workpiece_view.tooling_panel_widget import ToolingPanelWidget
from widgets.workpiece_view.workpiece_config_widget import WorkpieceConfigWidget


class WorkpieceView(QWidget):
    """Onglet Pièce : outillage + pièce + sauvegarde."""

    save_requested = pyqtSignal()
    save_as_requested = pyqtSignal()
    load_requested = pyqtSignal()
    new_requested = pyqtSignal()
    clear_piece_requested = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.current_config_label: QLabel | None = None
        self.status_label: QLabel | None = None
        self._setup_ui()

    def _setup_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setSpacing(6)
        outer.setContentsMargins(4, 4, 4, 4)

        outer.addLayout(self._build_header())

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

        btn_clear = QPushButton("🗑 Effacer la pièce")
        btn_clear.setToolTip("Supprime la CAO et remet la pose à zéro")
        btn_clear.clicked.connect(self.clear_piece_requested)
        piece_group_layout.addWidget(btn_clear)

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

    def _build_header(self) -> QVBoxLayout:
        header_layout = QVBoxLayout()
        header_layout.setSpacing(6)

        title_row = QHBoxLayout()
        title_label = QLabel("Configuration pièce")
        title_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        title_row.addWidget(title_label)
        title_row.addStretch()

        title_row.addWidget(QLabel("Statut :"))
        self.status_label = QLabel()
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        apply_status_badge(self.status_label, "Configuration non chargée", "#808080")
        title_row.addWidget(self.status_label)
        header_layout.addLayout(title_row)

        fields_layout = QGridLayout()
        fields_layout.setHorizontalSpacing(8)
        fields_layout.setVerticalSpacing(6)

        current_config_title_label = QLabel("Configuration courante :")
        current_config_title_label.setMinimumWidth(150)
        fields_layout.addWidget(current_config_title_label, 0, 0)

        self.current_config_label = QLabel("Aucune configuration")
        self.current_config_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        self.current_config_label.setMinimumWidth(220)
        self._apply_current_config_label_style()
        self.current_config_label.setFixedHeight(self.current_config_label.sizeHint().height())
        fields_layout.addWidget(self.current_config_label, 0, 1, Qt.AlignmentFlag.AlignVCenter)
        fields_layout.setColumnStretch(0, 0)
        fields_layout.setColumnStretch(1, 1)

        action_row = QHBoxLayout()
        action_row.addStretch()

        load_button = QPushButton("...")
        load_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        load_button.setToolTip("Charger une configuration pièce")
        load_button.clicked.connect(self.load_requested.emit)
        action_row.addWidget(load_button)

        new_button = QPushButton()
        new_button.setIcon(build_new_icon(self.palette()))
        new_button.setIconSize(CONFIG_ACTION_ICON_SIZE)
        new_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        new_button.setToolTip("Créer une nouvelle configuration pièce")
        new_button.clicked.connect(self.new_requested.emit)
        action_row.addWidget(new_button)

        save_button = QPushButton()
        save_button.setIcon(build_save_icon(self.palette()))
        save_button.setIconSize(CONFIG_ACTION_ICON_SIZE)
        save_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        save_button.setToolTip("Enregistrer la configuration pièce courante")
        save_button.clicked.connect(self.save_requested.emit)
        action_row.addWidget(save_button)

        save_as_button = QPushButton()
        save_as_button.setIcon(build_save_icon(self.palette(), include_pencil=True))
        save_as_button.setIconSize(CONFIG_ACTION_ICON_SIZE)
        save_as_button.setFixedSize(CONFIG_ACTION_BUTTON_SIZE, CONFIG_ACTION_BUTTON_SIZE)
        save_as_button.setToolTip("Enregistrer la configuration pièce dans un nouveau fichier JSON")
        save_as_button.clicked.connect(self.save_as_requested.emit)
        action_row.addWidget(save_as_button)

        fields_layout.addLayout(action_row, 1, 0, 1, 2)
        header_layout.addLayout(fields_layout)
        return header_layout

    def set_current_configuration_name(self, configuration_name: str) -> None:
        if self.current_config_label is None:
            return
        name = str(configuration_name or "").strip()
        self.current_config_label.setText(name or "Aucune configuration")

    def set_configuration_status(self, text: str, color: str = "#808080") -> None:
        if self.status_label is None:
            return
        apply_status_badge(self.status_label, text, color)

    def changeEvent(self, event) -> None:  # type: ignore[override]
        super().changeEvent(event)
        if event.type() == event.Type.PaletteChange:
            self._apply_current_config_label_style()

    def _apply_current_config_label_style(self) -> None:
        if self.current_config_label is None:
            return
        accent = self.palette().color(QPalette.ColorRole.Highlight).name()
        self.current_config_label.setStyleSheet(
            f"border: 1px solid #555; padding: 2px; background-color: #2a2a2a; color: {accent};"
        )
