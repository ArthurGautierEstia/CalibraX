"""Éditeur d'un élément d'outillage."""
from __future__ import annotations

import os
from pathlib import Path

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QColorDialog, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QScrollArea,
    QVBoxLayout, QWidget,
)

from models.tooling_element import ToolingElement
from models.types.pose6 import Pose6
from widgets.external_axes_view.pose6_editor_widget import Pose6EditorWidget

_STL_DEFAULT = str(Path(__file__).parent.parent.parent / "default_data" / "workspaces_stl")
if not Path(_STL_DEFAULT).exists():
    _STL_DEFAULT = str(Path.home())


def _normalize_project_path(path: str) -> str:
    absolute_path = os.path.abspath(path)
    project_root = os.path.abspath(os.getcwd())
    try:
        common_path = os.path.commonpath([project_root, absolute_path])
    except ValueError:
        return absolute_path
    if common_path != project_root:
        return absolute_path
    try:
        relative_path = os.path.relpath(absolute_path, project_root)
    except ValueError:
        return absolute_path
    relative_path = relative_path.replace("\\", "/")
    if relative_path == ".":
        return "./"
    return f"./{relative_path}" if not relative_path.startswith(".") else relative_path


def _color_to_style(rgba: tuple) -> str:
    r, g, b = int(rgba[0] * 255), int(rgba[1] * 255), int(rgba[2] * 255)
    return f"background-color: rgb({r},{g},{b}); border: 1px solid #666; border-radius: 3px;"


class ToolingElementEditorWidget(QScrollArea):
    """Éditeur pour un ToolingElement."""

    changed = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self._building = False
        self._current_id: str | None = None

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(8)
        self._setup_ui(layout)
        self.setWidget(container)
        self.setEnabled(False)

    def _setup_ui(self, layout: QVBoxLayout) -> None:
        # ── Identité ──────────────────────────────────────────────────
        id_form = QFormLayout()
        id_form.setSpacing(4)
        self._name_edit = QLineEdit()
        self._name_edit.setPlaceholderText("Nom de l'élément")
        self._name_edit.editingFinished.connect(self._emit)
        id_form.addRow("Nom :", self._name_edit)
        layout.addLayout(id_form)

        # ── CAO ───────────────────────────────────────────────────────
        cao_box = QGroupBox("CAO")
        cao_layout = QVBoxLayout(cao_box)
        cao_layout.setSpacing(4)

        stl_row = QWidget()
        stl_layout = QHBoxLayout(stl_row)
        stl_layout.setContentsMargins(0, 0, 0, 0)
        self._cad_line = QLineEdit()
        self._cad_line.setReadOnly(True)
        self._cad_line.setPlaceholderText("Aucun fichier STL")
        btn_browse = QPushButton("…")
        btn_browse.setFixedWidth(30)
        btn_browse.clicked.connect(self._browse_cad)
        self._color_btn = QPushButton()
        self._color_btn.setFixedSize(28, 28)
        self._color_btn.setToolTip("Couleur")
        self._color_btn.clicked.connect(self._pick_color)
        self._set_color(ToolingElement.DEFAULT_COLOR)
        stl_layout.addWidget(QLabel("STL :"))
        stl_layout.addWidget(self._cad_line)
        stl_layout.addWidget(btn_browse)
        stl_layout.addWidget(self._color_btn)
        cao_layout.addWidget(stl_row)
        layout.addWidget(cao_box)

        # ── Pose dans repère précédent ─────────────────────────────────
        self._pose_label = QLabel("Pour le 1er élément : repère parent de l'outillage")
        self._pose_label.setStyleSheet("color: gray; font-size: 11px;")
        layout.addWidget(self._pose_label)

        self._pose_editor = Pose6EditorWidget("Pose dans le repère précédent")
        self._pose_editor.pose_changed.connect(self._emit)
        layout.addWidget(self._pose_editor)

        # ── Repère de l'élément ────────────────────────────────────────
        frame_box = QGroupBox("Repère de l'élément (dans la CAO)")
        frame_layout = QVBoxLayout(frame_box)
        frame_layout.setSpacing(4)

        info = QLabel("Définit la sortie cinématique de cet élément.\nL'élément suivant s'y accrochera.")
        info.setStyleSheet("color: gray; font-size: 11px;")
        frame_layout.addWidget(info)

        self._frame_editor = Pose6EditorWidget("Repère dans la CAO")
        self._frame_editor.pose_changed.connect(self._emit)
        frame_layout.addWidget(self._frame_editor)
        layout.addWidget(frame_box)

        layout.addStretch()

    # ------------------------------------------------------------------

    def _browse_cad(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Sélectionner STL outillage", _STL_DEFAULT,
            "STL files (*.stl);;All files (*)"
        )
        if path:
            self._cad_line.setText(_normalize_project_path(path))
            self._emit()

    def _pick_color(self) -> None:
        cur = self._color_btn.property("color_rgba") or list(ToolingElement.DEFAULT_COLOR)
        qc = QColor(int(cur[0] * 255), int(cur[1] * 255), int(cur[2] * 255), int(cur[3] * 255))
        new_qc = QColorDialog.getColor(qc, self, "Couleur élément",
                                       options=QColorDialog.ColorDialogOption.ShowAlphaChannel)
        if new_qc.isValid():
            self._set_color((new_qc.redF(), new_qc.greenF(), new_qc.blueF(), new_qc.alphaF()))
            self._emit()

    def _set_color(self, rgba: tuple) -> None:
        self._color_btn.setStyleSheet(_color_to_style(rgba))
        self._color_btn.setProperty("color_rgba", list(rgba))

    def _emit(self, *_) -> None:
        if not self._building:
            self.changed.emit()

    # ------------------------------------------------------------------

    def get_element(self) -> ToolingElement | None:
        if self._current_id is None:
            return None
        color = tuple(self._color_btn.property("color_rgba") or list(ToolingElement.DEFAULT_COLOR))
        return ToolingElement(
            name=self._name_edit.text().strip() or "Élément",
            element_id=self._current_id,
            cad_model=self._cad_line.text(),
            cad_color=color,
            pose_in_prev=self._pose_editor.get_pose(),
            element_frame_pose=self._frame_editor.get_pose(),
        )

    def set_element(self, element: ToolingElement, is_first: bool = False) -> None:
        self._building = True
        self._current_id = element.id
        self._name_edit.setText(element.name)
        self._cad_line.setText(element.cad_model)
        self._set_color(element.cad_color)
        self._pose_editor.set_pose(element.pose_in_prev)
        self._frame_editor.set_pose(element.element_frame_pose)
        label_text = (
            "Pour le 1er élément : repère parent de l'outillage"
            if is_first
            else "Repère de l'élément précédent"
        )
        self._pose_label.setText(label_text)
        self.setEnabled(True)
        self._building = False

    def clear(self) -> None:
        self._current_id = None
        self.setEnabled(False)
