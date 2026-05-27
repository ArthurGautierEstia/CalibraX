"""Widget de configuration de la section Pièce (sans l'outillage)."""
from __future__ import annotations

import os
from pathlib import Path

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QColorDialog, QComboBox, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QScrollArea,
    QVBoxLayout, QWidget,
)

from models.types.pose6 import Pose6
from models.workpiece_model import WorkpieceModel
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


class WorkpieceConfigWidget(QWidget):
    """Section configuration de la pièce (CAO, repère parent, pose, repère pièce)."""

    config_changed = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._building = False
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        layout.setContentsMargins(0, 0, 0, 0)
        self._setup_ui(layout)

    def _setup_ui(self, layout: QVBoxLayout) -> None:
        # ── Positionnement ────────────────────────────────────────────
        pos_form = QFormLayout()
        pos_form.setSpacing(4)
        self._parent_combo = QComboBox()
        self._parent_combo.addItem("Monde", "")
        self._parent_combo.currentIndexChanged.connect(self._emit)
        pos_form.addRow("Repère parent :", self._parent_combo)
        layout.addLayout(pos_form)

        # ── CAO ───────────────────────────────────────────────────────
        cao_box = QGroupBox("CAO Pièce")
        cao_layout = QVBoxLayout(cao_box)
        cao_layout.setSpacing(4)

        stl_row = QWidget()
        stl_layout = QHBoxLayout(stl_row)
        stl_layout.setContentsMargins(0, 0, 0, 0)
        self._cad_line = QLineEdit()
        self._cad_line.setReadOnly(True)
        self._cad_line.setPlaceholderText("Aucun fichier STL sélectionné")
        btn_browse = QPushButton("…")
        btn_browse.setFixedWidth(30)
        btn_browse.clicked.connect(self._browse_cad)
        self._color_btn = QPushButton()
        self._color_btn.setFixedSize(28, 28)
        self._color_btn.setToolTip("Couleur de la pièce")
        self._color_btn.clicked.connect(self._pick_color)
        self._set_color((0.8, 0.6, 0.2, 1.0))
        stl_layout.addWidget(QLabel("STL :"))
        stl_layout.addWidget(self._cad_line)
        stl_layout.addWidget(btn_browse)
        stl_layout.addWidget(self._color_btn)
        cao_layout.addWidget(stl_row)
        layout.addWidget(cao_box)

        # ── Pose ──────────────────────────────────────────────────────
        self._pose_editor = Pose6EditorWidget("Pose dans le repère parent")
        self._pose_editor.pose_changed.connect(self._emit)
        layout.addWidget(self._pose_editor)

        # ── Repère pièce ──────────────────────────────────────────────
        frame_box = QGroupBox("Repère pièce")
        frame_layout = QVBoxLayout(frame_box)
        frame_layout.setSpacing(4)

        info = QLabel("Origine du repère pièce dans la CAO (pour l'import de programmes).")
        info.setStyleSheet("color: gray; font-size: 11px;")
        frame_layout.addWidget(info)

        self._frame_editor = Pose6EditorWidget("Repère pièce dans la CAO")
        self._frame_editor.pose_changed.connect(self._emit)
        frame_layout.addWidget(self._frame_editor)
        layout.addWidget(frame_box)

    # ------------------------------------------------------------------

    def _browse_cad(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Sélectionner STL pièce", _STL_DEFAULT,
            "STL files (*.stl);;All files (*)"
        )
        if path:
            self._cad_line.setText(_normalize_project_path(path))
            self._emit()

    def _pick_color(self) -> None:
        cur = self._color_btn.property("color_rgba") or [0.8, 0.6, 0.2, 1.0]
        qc = QColor(int(cur[0] * 255), int(cur[1] * 255), int(cur[2] * 255), int(cur[3] * 255))
        new_qc = QColorDialog.getColor(qc, self, "Couleur pièce",
                                       options=QColorDialog.ColorDialogOption.ShowAlphaChannel)
        if new_qc.isValid():
            self._set_color((new_qc.redF(), new_qc.greenF(), new_qc.blueF(), new_qc.alphaF()))
            self._emit()

    def _set_color(self, rgba: tuple) -> None:
        self._color_btn.setStyleSheet(_color_to_style(rgba))
        self._color_btn.setProperty("color_rgba", list(rgba))

    def _emit(self, *_) -> None:
        if not self._building:
            self.config_changed.emit()

    # ------------------------------------------------------------------
    # Mise à jour du dropdown repère parent
    # ------------------------------------------------------------------

    def update_parent_frames(
        self,
        ext_axes: list[tuple[str, str]],
        ws_elements: list[str],
        has_tooling: bool = False,
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
        if has_tooling:
            self._parent_combo.addItem("Outillage (repère final)", WorkpieceModel.FRAME_TOOLING)

        restored = False
        for i in range(self._parent_combo.count()):
            if self._parent_combo.itemData(i) == current_id:
                self._parent_combo.setCurrentIndex(i)
                restored = True
                break
        if not restored:
            if has_tooling:
                for i in range(self._parent_combo.count()):
                    if self._parent_combo.itemData(i) == WorkpieceModel.FRAME_TOOLING:
                        self._parent_combo.setCurrentIndex(i)
                        break
            else:
                self._parent_combo.setCurrentIndex(0)
        self._parent_combo.blockSignals(False)

    # ------------------------------------------------------------------
    # Lecture / écriture
    # ------------------------------------------------------------------

    def get_data(self) -> dict:
        color = tuple(self._color_btn.property("color_rgba") or [0.8, 0.6, 0.2, 1.0])
        return {
            "cad_model": self._cad_line.text(),
            "cad_color": list(color),
            "parent_frame_id": self._parent_combo.currentData() or "",
            "pose_in_parent": self._pose_editor.get_pose().to_list(),
            "workpiece_frame_pose": self._frame_editor.get_pose().to_list(),
        }

    def set_data(self, model: WorkpieceModel) -> None:
        self._building = True
        self._cad_line.setText(model.get_cad_model())
        self._set_color(model.get_cad_color())
        self._pose_editor.set_pose(model.get_pose_in_parent())
        self._frame_editor.set_pose(model.get_workpiece_frame_pose())

        target_id = model.get_parent_frame_id()
        for i in range(self._parent_combo.count()):
            if self._parent_combo.itemData(i) == target_id:
                self._parent_combo.setCurrentIndex(i)
                break
        self._building = False
