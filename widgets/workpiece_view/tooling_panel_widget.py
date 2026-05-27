"""Panneau de configuration de l'outillage (liste d'éléments + éditeur)."""
from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox, QFormLayout, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QPushButton, QSplitter, QVBoxLayout, QWidget,
)

from models.tooling_element import ToolingElement
from models.tooling_model import ToolingModel
from widgets.workpiece_view.tooling_element_editor import ToolingElementEditorWidget


class ToolingPanelWidget(QWidget):
    """Panneau outillage : repère parent + liste d'éléments + éditeur sélectionné."""

    tooling_changed = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._elements: list[ToolingElement] = []
        self._updating_ui = False
        self._setup_ui()

    # ------------------------------------------------------------------
    # Construction UI
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setSpacing(6)
        outer.setContentsMargins(4, 4, 4, 4)

        # Repère parent de l'outillage
        parent_form = QFormLayout()
        parent_form.setSpacing(4)
        self._parent_combo = QComboBox()
        self._parent_combo.addItem("Monde", "")
        self._parent_combo.currentIndexChanged.connect(self._emit)
        parent_form.addRow("Repère parent :", self._parent_combo)
        outer.addLayout(parent_form)

        # Splitter horizontal : liste à gauche, éditeur à droite
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # ── Gauche : liste ─────────────────────────────────────────
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setSpacing(4)
        left_layout.setContentsMargins(0, 0, 0, 0)

        left_layout.addWidget(QLabel("<b>Éléments de l'outillage</b>"))

        self._list = QListWidget()
        self._list.currentRowChanged.connect(self._on_selection_changed)
        left_layout.addWidget(self._list)

        btn_row = QWidget()
        btn_layout = QHBoxLayout(btn_row)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(4)

        btn_add = QPushButton("+ Ajouter")
        btn_add.clicked.connect(self._add_element)
        btn_remove = QPushButton("− Supprimer")
        btn_remove.clicked.connect(self._remove_selected)
        btn_layout.addWidget(btn_add)
        btn_layout.addWidget(btn_remove)
        left_layout.addWidget(btn_row)

        move_row = QWidget()
        move_layout = QHBoxLayout(move_row)
        move_layout.setContentsMargins(0, 0, 0, 0)
        move_layout.setSpacing(4)
        btn_up = QPushButton("↑")
        btn_up.setFixedWidth(36)
        btn_up.clicked.connect(self._move_up)
        btn_down = QPushButton("↓")
        btn_down.setFixedWidth(36)
        btn_down.clicked.connect(self._move_down)
        move_layout.addWidget(btn_up)
        move_layout.addWidget(btn_down)
        move_layout.addStretch()
        left_layout.addWidget(move_row)

        splitter.addWidget(left)

        # ── Droite : éditeur ───────────────────────────────────────
        self._editor = ToolingElementEditorWidget()
        self._editor.changed.connect(self._on_editor_changed)
        splitter.addWidget(self._editor)

        splitter.setSizes([180, 300])
        splitter.setChildrenCollapsible(False)

        outer.addWidget(splitter)

        # Indicateur repère outillage
        self._tooling_frame_label = QLabel("Repère outillage : (aucun élément)")
        self._tooling_frame_label.setStyleSheet("color: gray; font-size: 11px;")
        outer.addWidget(self._tooling_frame_label)

    # ------------------------------------------------------------------
    # Gestion liste
    # ------------------------------------------------------------------

    def _add_element(self) -> None:
        elem = ToolingElement(name=f"Élément {len(self._elements) + 1}")
        self._elements.append(elem)
        self._rebuild_list()
        self._list.setCurrentRow(len(self._elements) - 1)
        self._emit()

    def _remove_selected(self) -> None:
        row = self._list.currentRow()
        if row < 0 or row >= len(self._elements):
            return
        self._elements.pop(row)
        self._rebuild_list()
        if self._elements:
            self._list.setCurrentRow(min(row, len(self._elements) - 1))
        else:
            self._editor.clear()
        self._emit()

    def _move_up(self) -> None:
        row = self._list.currentRow()
        if row <= 0:
            return
        self._elements[row - 1], self._elements[row] = self._elements[row], self._elements[row - 1]
        self._rebuild_list()
        self._list.setCurrentRow(row - 1)
        self._emit()

    def _move_down(self) -> None:
        row = self._list.currentRow()
        if row < 0 or row >= len(self._elements) - 1:
            return
        self._elements[row], self._elements[row + 1] = self._elements[row + 1], self._elements[row]
        self._rebuild_list()
        self._list.setCurrentRow(row + 1)
        self._emit()

    def _rebuild_list(self) -> None:
        self._updating_ui = True
        self._list.clear()
        for i, elem in enumerate(self._elements):
            label = f"[{i}] {elem.name}"
            if i == len(self._elements) - 1 and self._elements:
                label += "  ← repère outillage"
            self._list.addItem(QListWidgetItem(label))
        self._update_tooling_frame_label()
        self._updating_ui = False

    def _update_tooling_frame_label(self) -> None:
        if self._elements:
            last = self._elements[-1]
            self._tooling_frame_label.setText(
                f"Repère outillage = repère de « {last.name} »"
            )
        else:
            self._tooling_frame_label.setText("Repère outillage : (aucun élément)")

    def _on_selection_changed(self, row: int) -> None:
        if self._updating_ui:
            return
        if row < 0 or row >= len(self._elements):
            self._editor.clear()
            return
        self._editor.set_element(self._elements[row], is_first=(row == 0))

    def _on_editor_changed(self) -> None:
        if self._updating_ui:
            return
        row = self._list.currentRow()
        if row < 0 or row >= len(self._elements):
            return
        updated = self._editor.get_element()
        if updated is not None:
            self._elements[row] = updated
            self._rebuild_list()
            self._list.setCurrentRow(row)
        self._emit()

    def _emit(self, *_) -> None:
        self.tooling_changed.emit()

    # ------------------------------------------------------------------
    # Mise à jour dropdown repère parent
    # ------------------------------------------------------------------

    def update_parent_frames(
        self,
        ext_axes: list[tuple[str, str]],
        ws_elements: list[str],
    ) -> None:
        current_id = self._parent_combo.currentData()
        self._parent_combo.blockSignals(True)
        self._parent_combo.clear()
        self._parent_combo.addItem("Monde", "")
        self._parent_combo.addItem("Robot (base)", "robot")
        for axis_id, axis_name in ext_axes:
            self._parent_combo.addItem(f"Axe ext. : {axis_name}", f"ext:{axis_id}")
        for elem_name in ws_elements:
            self._parent_combo.addItem(f"Workspace : {elem_name}", f"ws:{elem_name}")
        restored = False
        for i in range(self._parent_combo.count()):
            if self._parent_combo.itemData(i) == current_id:
                self._parent_combo.setCurrentIndex(i)
                restored = True
                break
        if not restored:
            self._parent_combo.setCurrentIndex(0)
        self._parent_combo.blockSignals(False)

    # ------------------------------------------------------------------
    # Lecture / écriture depuis le modèle
    # ------------------------------------------------------------------

    def get_parent_frame_id(self) -> str:
        return self._parent_combo.currentData() or ""

    def get_elements(self) -> list[ToolingElement]:
        return [e.copy() for e in self._elements]

    def has_elements(self) -> bool:
        return bool(self._elements)

    def set_from_model(self, model: ToolingModel) -> None:
        self._updating_ui = True
        self._elements = model.get_elements()

        target_id = model.get_parent_frame_id()
        for i in range(self._parent_combo.count()):
            if self._parent_combo.itemData(i) == target_id:
                self._parent_combo.setCurrentIndex(i)
                break

        self._rebuild_list()
        self._editor.clear()
        if self._elements:
            self._list.setCurrentRow(0)
            self._editor.set_element(self._elements[0], is_first=True)

        self._updating_ui = False
