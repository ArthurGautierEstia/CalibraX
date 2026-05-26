"""Widget de configuration d'un axe externe (panneau droit de l'onglet)."""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QFileDialog, QFormLayout, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QScrollArea, QVBoxLayout, QWidget,
)

from models.external_axis import ExternalAxis
from models.external_axis_joint import ExternalAxisJoint
from models.types.external_axis_joint_type import ExternalAxisJointType
from models.types.external_axis_mount_mode import ExternalAxisMountMode
from models.types.pose6 import Pose6
from widgets.external_axes_view.joint_editor_widget import JointEditorWidget
from widgets.external_axes_view.pose6_editor_widget import Pose6EditorWidget


class ExternalAxisConfigWidget(QScrollArea):
    """Panneau d'édition complet d'un ExternalAxis."""

    axis_changed = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self._building = False
        self._joint_editors: list[JointEditorWidget] = []

        container = QWidget()
        self._main_layout = QVBoxLayout(container)
        self._main_layout.setSpacing(8)
        self._setup_ui()
        self.setWidget(container)

    # ------------------------------------------------------------------
    # Construction UI
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        # ── Section Identité ──────────────────────────────────────────
        id_box = QGroupBox("Identité")
        id_form = QFormLayout(id_box)
        id_form.setSpacing(4)

        self._name_edit = QLineEdit()
        self._name_edit.setPlaceholderText("Nom de l'axe")
        self._name_edit.editingFinished.connect(self._emit)
        id_form.addRow("Nom :", self._name_edit)

        self._mode_combo = QComboBox()
        self._mode_combo.addItem("Axes positionnés", ExternalAxisMountMode.POSITIONED)
        self._mode_combo.addItem("Synchronisé (futur)", ExternalAxisMountMode.SYNCHRONIZED)
        self._mode_combo.model().item(1).setEnabled(False)
        self._mode_combo.currentIndexChanged.connect(self._emit)
        id_form.addRow("Mode :", self._mode_combo)

        self._parent_combo = QComboBox()
        self._parent_combo.addItem("Monde (libre)", None)
        self._parent_combo.currentIndexChanged.connect(self._emit)
        id_form.addRow("Monté sur :", self._parent_combo)

        self._main_layout.addWidget(id_box)

        # ── Section Base (CAO fixe) ───────────────────────────────────
        base_box = QGroupBox("Base (élément fixe)")
        base_layout = QVBoxLayout(base_box)

        cad_row = QWidget()
        cad_layout = QHBoxLayout(cad_row)
        cad_layout.setContentsMargins(0, 0, 0, 0)
        self._base_cad_line = QLineEdit()
        self._base_cad_line.setReadOnly(True)
        self._base_cad_line.setPlaceholderText("Aucun fichier STL")
        btn = QPushButton("…")
        btn.setFixedWidth(30)
        btn.clicked.connect(self._browse_base_cad)
        cad_layout.addWidget(QLabel("CAO (STL) :"))
        cad_layout.addWidget(self._base_cad_line)
        cad_layout.addWidget(btn)
        base_layout.addWidget(cad_row)

        self._base_pose_editor = Pose6EditorWidget("Position dans parent")
        self._base_pose_editor.pose_changed.connect(self._emit)
        base_layout.addWidget(self._base_pose_editor)

        self._axis_frame_editor = Pose6EditorWidget("Repère mathématique de l'axe")
        self._axis_frame_editor.pose_changed.connect(self._emit)
        base_layout.addWidget(self._axis_frame_editor)

        self._main_layout.addWidget(base_box)

        # ── Section Joints ────────────────────────────────────────────
        joints_box = QGroupBox("Articulations")
        self._joints_layout = QVBoxLayout(joints_box)

        btn_add_joint = QPushButton("+ Ajouter une articulation")
        btn_add_joint.clicked.connect(self._add_joint)
        self._joints_layout.addWidget(btn_add_joint)

        self._main_layout.addWidget(joints_box)

        # ── Section Sortie ────────────────────────────────────────────
        end_box = QGroupBox("Sortie / Point de montage")
        end_layout = QVBoxLayout(end_box)
        self._mount_pose_editor = Pose6EditorWidget("Pose montage (dans dernier joint)")
        self._mount_pose_editor.pose_changed.connect(self._emit)
        end_layout.addWidget(self._mount_pose_editor)
        self._main_layout.addWidget(end_box)

        self._main_layout.addStretch()

    # ------------------------------------------------------------------
    # Joints dynamiques
    # ------------------------------------------------------------------

    def _add_joint(self, joint: ExternalAxisJoint | None = None) -> None:
        idx = len(self._joint_editors)
        editor = JointEditorWidget(idx)
        editor.changed.connect(self._emit)
        if joint is not None:
            editor.set_joint(joint)

        btn_remove = QPushButton(f"- Supprimer Joint {idx + 1}")
        btn_remove.clicked.connect(lambda: self._remove_joint(idx))

        # Insérer avant le bouton "+" (dernier item du joints_layout est le stretch)
        insert_pos = self._joints_layout.count() - 1
        self._joints_layout.insertWidget(insert_pos, editor)
        self._joints_layout.insertWidget(insert_pos + 1, btn_remove)
        self._joint_editors.append(editor)
        self._emit()

    def _remove_joint(self, idx: int) -> None:
        if idx >= len(self._joint_editors):
            return
        # Rebuild: retire tous les joints et les recrée sans celui-ci
        old_joints = [ed.get_joint() for ed in self._joint_editors]
        old_joints.pop(idx)
        self._clear_joints_ui()
        for j in old_joints:
            self._add_joint(j)
        self._rebuild_joint_labels()
        self._emit()

    def _clear_joints_ui(self) -> None:
        self._joint_editors.clear()
        while self._joints_layout.count() > 1:  # garder le bouton +
            item = self._joints_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    def _rebuild_joint_labels(self) -> None:
        for i, ed in enumerate(self._joint_editors):
            ed.setTitle(f"Joint {i + 1}")

    # ------------------------------------------------------------------
    # Accès données
    # ------------------------------------------------------------------

    def get_axis(self, current_id: str) -> ExternalAxis:
        parent_id = self._parent_combo.currentData()
        return ExternalAxis(
            name=self._name_edit.text().strip() or "Axe externe",
            axis_id=current_id,
            mount_parent_id=parent_id if parent_id != "None" and parent_id else None,
            mount_mode=self._mode_combo.currentData(),
            base_cad_model=self._base_cad_line.text(),
            base_pose_in_parent=self._base_pose_editor.get_pose(),
            axis_frame_in_base=self._axis_frame_editor.get_pose(),
            joints=[ed.get_joint() for ed in self._joint_editors],
            mount_pose_in_end=self._mount_pose_editor.get_pose(),
        )

    def set_axis(self, axis: ExternalAxis, other_axes: list[ExternalAxis]) -> None:
        self._building = True
        self._name_edit.setText(axis.name)

        idx = 0 if axis.mount_mode == ExternalAxisMountMode.POSITIONED else 1
        self._mode_combo.setCurrentIndex(idx)

        # Remplir le combo parent avec les autres axes (pour éviter les cycles simples)
        self._parent_combo.blockSignals(True)
        self._parent_combo.clear()
        self._parent_combo.addItem("Monde (libre)", None)
        for other in other_axes:
            if other.id != axis.id:
                self._parent_combo.addItem(other.name, other.id)
        selected_parent = axis.mount_parent_id
        for i in range(self._parent_combo.count()):
            if self._parent_combo.itemData(i) == selected_parent:
                self._parent_combo.setCurrentIndex(i)
                break
        self._parent_combo.blockSignals(False)

        self._base_cad_line.setText(axis.base_cad_model)
        self._base_pose_editor.set_pose(axis.base_pose_in_parent)
        self._axis_frame_editor.set_pose(axis.axis_frame_in_base)

        self._clear_joints_ui()
        for joint in axis.joints:
            self._add_joint(joint)

        self._mount_pose_editor.set_pose(axis.mount_pose_in_end)
        self._building = False

    def refresh_parent_combo(self, current_axis_id: str, all_axes: list[ExternalAxis]) -> None:
        current_parent = self._parent_combo.currentData()
        self._parent_combo.blockSignals(True)
        self._parent_combo.clear()
        self._parent_combo.addItem("Monde (libre)", None)
        for a in all_axes:
            if a.id != current_axis_id:
                self._parent_combo.addItem(a.name, a.id)
        for i in range(self._parent_combo.count()):
            if self._parent_combo.itemData(i) == current_parent:
                self._parent_combo.setCurrentIndex(i)
                break
        self._parent_combo.blockSignals(False)

    # ------------------------------------------------------------------

    def _browse_base_cad(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Sélectionner STL base", "", "STL files (*.stl);;All files (*)")
        if path:
            self._base_cad_line.setText(path)
            self._emit()

    def _emit(self, *_) -> None:
        if not self._building:
            self.axis_changed.emit()
