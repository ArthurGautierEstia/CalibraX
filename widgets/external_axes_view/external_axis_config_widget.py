"""Widget de configuration d'un axe externe – interface simplifiée."""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFileDialog, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QScrollArea, QSlider,
    QVBoxLayout, QWidget,
)

from models.external_axis import ExternalAxis
from models.external_axis_joint import ExternalAxisJoint
from models.types.external_axis_joint_type import ExternalAxisJointType
from models.types.external_axis_mount_mode import ExternalAxisMountMode
from models.types.pose6 import Pose6
from widgets.external_axes_view.pose6_editor_widget import Pose6EditorWidget

# Dossier par défaut pour l'explorateur STL
_STL_DEFAULT = str(
    Path(__file__).parent.parent.parent / "default_data" / "external_axes_stl"
)
if not Path(_STL_DEFAULT).exists():
    _STL_DEFAULT = str(Path.home())

# Axes disponibles dans la combo (label, vecteur unitaire)
_AXIS_CHOICES = [
    ("X",  (1.0,  0.0,  0.0)),
    ("Y",  (0.0,  1.0,  0.0)),
    ("Z",  (0.0,  0.0,  1.0)),
    ("-X", (-1.0, 0.0,  0.0)),
    ("-Y", (0.0, -1.0,  0.0)),
    ("-Z", (0.0,  0.0, -1.0)),
]

_SLIDER_STEPS = 1000


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


def _axis_to_combo_index(axis: tuple[float, float, float]) -> int:
    best, best_dot = 0, -2.0
    for i, (_, v) in enumerate(_AXIS_CHOICES):
        d = float(np.dot(axis, v))
        if d > best_dot:
            best_dot, best = d, i
    return best


def _color_to_style(rgba: tuple) -> str:
    r, g, b = int(rgba[0] * 255), int(rgba[1] * 255), int(rgba[2] * 255)
    return f"background-color: rgb({r},{g},{b}); border: 1px solid #666; border-radius: 3px;"


def _make_color_button(rgba: tuple) -> QPushButton:
    btn = QPushButton()
    btn.setFixedSize(28, 28)
    btn.setToolTip("Choisir la couleur")
    btn.setStyleSheet(_color_to_style(rgba))
    btn.setProperty("color_rgba", list(rgba))
    return btn


def _pick_color(btn: QPushButton, parent: QWidget) -> bool:
    """Ouvre le sélecteur de couleur et met à jour le bouton. Retourne True si changé."""
    from PyQt6.QtWidgets import QColorDialog
    cur = btn.property("color_rgba") or [0.5, 0.5, 0.5, 1.0]
    qc = QColor(int(cur[0]*255), int(cur[1]*255), int(cur[2]*255), int(cur[3]*255))
    new_qc = QColorDialog.getColor(
        qc, parent, "Couleur",
        options=QColorDialog.ColorDialogOption.ShowAlphaChannel,
    )
    if not new_qc.isValid():
        return False
    rgba = (new_qc.redF(), new_qc.greenF(), new_qc.blueF(), new_qc.alphaF())
    btn.setStyleSheet(_color_to_style(rgba))
    btn.setProperty("color_rgba", list(rgba))
    return True


# ---------------------------------------------------------------------------
# Widget d'une section articulaire (1 ou 2 joints)
# ---------------------------------------------------------------------------

class _JointSectionWidget(QGroupBox):
    """Éditeur d'un degré de liberté dans l'interface simplifiée.

    Pour joint_index == 0 :
        - « Repère d'axe » = axis_frame_in_base (passé via set_data)
    Pour joint_index > 0 :
        - « Repère d'axe N » = link_pose_in_prev du joint
    """

    changed = pyqtSignal()          # changement structurel
    value_changed = pyqtSignal(float)  # slider bougé

    def __init__(self, joint_index: int, parent: QWidget | None = None) -> None:
        label = f"Axe {joint_index + 1}"
        super().__init__(label, parent)
        self._joint_index = joint_index
        self._building = False
        self._q_min = -1000.0
        self._q_max = 1000.0
        self._setup_ui()

    # ------------------------------------------------------------------
    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setSpacing(6)

        # ── Repère d'axe ──────────────────────────────────────────────
        frame_label = (
            "Repère d'axe dans la base"
            if self._joint_index == 0
            else f"Repère d'axe {self._joint_index + 1} dans l'axe {self._joint_index}"
        )
        self._frame_editor = Pose6EditorWidget(frame_label)
        self._frame_editor.pose_changed.connect(self._emit)
        layout.addWidget(self._frame_editor)

        # ── Type + Direction ──────────────────────────────────────────
        type_dir_widget = QWidget()
        type_dir_layout = QFormLayout(type_dir_widget)
        type_dir_layout.setSpacing(4)

        self._type_combo = QComboBox()
        self._type_combo.addItem("Prismatique (translation)", ExternalAxisJointType.LINEAR)
        self._type_combo.addItem("Rotoïde (rotation)", ExternalAxisJointType.ROTARY)
        self._type_combo.currentIndexChanged.connect(self._on_type_changed)
        type_dir_layout.addRow("Type :", self._type_combo)

        self._dir_combo = QComboBox()
        for label, _ in _AXIS_CHOICES:
            self._dir_combo.addItem(label)
        self._dir_combo.currentIndexChanged.connect(self._emit)
        type_dir_layout.addRow("Direction :", self._dir_combo)

        layout.addWidget(type_dir_widget)

        # ── Limites ───────────────────────────────────────────────────
        limits_box = QGroupBox("Limites")
        limits_layout = QFormLayout(limits_box)
        limits_layout.setSpacing(4)

        self._min_spin = self._make_limit_spin()
        self._max_spin = self._make_limit_spin()
        self._offset_spin = self._make_limit_spin()
        self._min_spin.setValue(-1000.0)
        self._max_spin.setValue(1000.0)
        self._min_spin.editingFinished.connect(self._on_limits_changed)
        self._max_spin.editingFinished.connect(self._on_limits_changed)
        self._offset_spin.editingFinished.connect(self._emit)

        lim_row = QWidget()
        lim_layout = QHBoxLayout(lim_row)
        lim_layout.setContentsMargins(0, 0, 0, 0)
        lim_layout.addWidget(QLabel("Min"))
        lim_layout.addWidget(self._min_spin)
        lim_layout.addWidget(QLabel("Max"))
        lim_layout.addWidget(self._max_spin)
        limits_layout.addRow("Limites :", lim_row)
        limits_layout.addRow("Offset (zéro) :", self._offset_spin)
        layout.addWidget(limits_box)

        # ── Slider de test ────────────────────────────────────────────
        test_box = QGroupBox("Test (jog)")
        test_layout = QVBoxLayout(test_box)
        test_layout.setSpacing(4)

        slider_row = QWidget()
        slider_row_layout = QHBoxLayout(slider_row)
        slider_row_layout.setContentsMargins(0, 0, 0, 0)
        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setRange(0, _SLIDER_STEPS)
        self._slider.setValue(0)
        self._slider.valueChanged.connect(self._on_slider_moved)
        self._value_label = QLabel("0.000")
        self._value_label.setFixedWidth(70)
        self._unit_label = QLabel("mm")
        slider_row_layout.addWidget(self._slider)
        slider_row_layout.addWidget(self._value_label)
        slider_row_layout.addWidget(self._unit_label)
        test_layout.addWidget(slider_row)
        layout.addWidget(test_box)

        # ── CAO pièce mobile ──────────────────────────────────────────
        cad_box = QGroupBox("Pièce mobile (CAO)")
        cad_layout = QVBoxLayout(cad_box)
        cad_layout.setSpacing(4)

        cad_file_row = QWidget()
        cad_file_layout = QHBoxLayout(cad_file_row)
        cad_file_layout.setContentsMargins(0, 0, 0, 0)
        self._cad_line = QLineEdit()
        self._cad_line.setReadOnly(True)
        self._cad_line.setPlaceholderText("Aucun fichier STL")
        btn_cad = QPushButton("…")
        btn_cad.setFixedWidth(30)
        btn_cad.clicked.connect(self._browse_cad)
        self._cad_color_btn = _make_color_button((0.25, 0.45, 0.65, 1.0))
        self._cad_color_btn.clicked.connect(self._pick_cad_color)
        cad_file_layout.addWidget(QLabel("STL :"))
        cad_file_layout.addWidget(self._cad_line)
        cad_file_layout.addWidget(btn_cad)
        cad_file_layout.addWidget(self._cad_color_btn)
        cad_layout.addWidget(cad_file_row)

        self._mobile_pose_editor = Pose6EditorWidget("Position dans le repère d'axe")
        self._mobile_pose_editor.pose_changed.connect(self._emit)
        cad_layout.addWidget(self._mobile_pose_editor)

        layout.addWidget(cad_box)

    # ------------------------------------------------------------------
    def _make_limit_spin(self) -> QDoubleSpinBox:
        sb = QDoubleSpinBox()
        sb.setRange(-99999.0, 99999.0)
        sb.setDecimals(2)
        sb.setSingleStep(10.0)
        return sb

    def _on_type_changed(self) -> None:
        jtype = self._type_combo.currentData()
        unit = "mm" if jtype == ExternalAxisJointType.LINEAR else "°"
        self._unit_label.setText(unit)
        self._emit()

    def _on_limits_changed(self) -> None:
        self._q_min = self._min_spin.value()
        self._q_max = self._max_spin.value()
        self._refresh_slider_label()
        self._emit()

    def _on_slider_moved(self, slider_val: int) -> None:
        v = self._slider_to_value(slider_val)
        self._value_label.setText(f"{v:.3f}")
        if not self._building:
            self.value_changed.emit(v)

    def _slider_to_value(self, slider_val: int) -> float:
        if self._q_max <= self._q_min:
            return self._q_min
        ratio = slider_val / _SLIDER_STEPS
        return self._q_min + ratio * (self._q_max - self._q_min)

    def _value_to_slider(self, value: float) -> int:
        if self._q_max <= self._q_min:
            return 0
        ratio = (value - self._q_min) / (self._q_max - self._q_min)
        return int(max(0, min(_SLIDER_STEPS, ratio * _SLIDER_STEPS)))

    def _refresh_slider_label(self) -> None:
        v = self._slider_to_value(self._slider.value())
        self._value_label.setText(f"{v:.3f}")

    def _browse_cad(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Sélectionner STL mobile", _STL_DEFAULT,
            "STL files (*.stl);;All files (*)"
        )
        if path:
            self._cad_line.setText(_normalize_project_path(path))
            self._emit()

    def _pick_cad_color(self) -> None:
        if _pick_color(self._cad_color_btn, self):
            self._emit()

    def _emit(self, *_) -> None:
        if not self._building:
            self.changed.emit()

    # ------------------------------------------------------------------
    # Accès données
    # ------------------------------------------------------------------

    def get_axis_frame(self) -> Pose6:
        """Retourne la pose du repère d'axe (axis_frame_in_base pour joint 0,
        link_pose_in_prev pour joint i>0)."""
        return self._frame_editor.get_pose()

    def get_joint(self) -> ExternalAxisJoint:
        dir_vec = _AXIS_CHOICES[self._dir_combo.currentIndex()][1]
        cad_color = tuple(self._cad_color_btn.property("color_rgba") or [0.25, 0.45, 0.65, 1.0])
        link_pose = (
            Pose6.zeros()
            if self._joint_index == 0
            else self._frame_editor.get_pose()
        )
        return ExternalAxisJoint(
            joint_type=self._type_combo.currentData(),
            axis=dir_vec,
            link_pose_in_prev=link_pose,
            cad_model=self._cad_line.text(),
            cad_color=cad_color,
            cad_offset_in_joint=self._mobile_pose_editor.get_pose(),
            q_min=self._min_spin.value(),
            q_max=self._max_spin.value(),
            offset=self._offset_spin.value(),
            value=self._slider_to_value(self._slider.value()),
        )

    def get_slider_value(self) -> float:
        return self._slider_to_value(self._slider.value())

    def set_data(
        self,
        joint: ExternalAxisJoint,
        axis_frame: Pose6 | None = None,
    ) -> None:
        """Remplit les champs depuis un ExternalAxisJoint.

        axis_frame : Pose6 pour le repère d'axe (axis_frame_in_base) si joint_index==0,
                     sinon None (link_pose_in_prev utilisé directement).
        """
        self._building = True
        if self._joint_index == 0:
            self._frame_editor.set_pose(axis_frame or Pose6.zeros())
        else:
            self._frame_editor.set_pose(joint.link_pose_in_prev)

        idx = 0 if joint.joint_type == ExternalAxisJointType.LINEAR else 1
        self._type_combo.setCurrentIndex(idx)
        unit = "mm" if joint.joint_type == ExternalAxisJointType.LINEAR else "°"
        self._unit_label.setText(unit)

        self._dir_combo.setCurrentIndex(_axis_to_combo_index(joint.axis))

        self._min_spin.setValue(joint.q_min)
        self._max_spin.setValue(joint.q_max)
        self._offset_spin.setValue(joint.offset)
        self._q_min = joint.q_min
        self._q_max = joint.q_max

        slider_pos = self._value_to_slider(joint.value)
        self._slider.setValue(slider_pos)
        self._value_label.setText(f"{joint.value:.3f}")

        self._cad_line.setText(joint.cad_model)
        rgba = list(joint.cad_color)
        self._cad_color_btn.setStyleSheet(_color_to_style(joint.cad_color))
        self._cad_color_btn.setProperty("color_rgba", rgba)

        self._mobile_pose_editor.set_pose(joint.cad_offset_in_joint)
        self._building = False


# ---------------------------------------------------------------------------
# Widget de configuration principal
# ---------------------------------------------------------------------------

class ExternalAxisConfigWidget(QScrollArea):
    """Panneau d'édition complet d'un ExternalAxis (interface simplifiée)."""

    axis_changed = pyqtSignal()
    joint_value_changed = pyqtSignal(int, float)   # (joint_index, value)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWidgetResizable(True)
        self._building = False
        self._joint_sections: list[_JointSectionWidget] = []

        container = QWidget()
        self._main_layout = QVBoxLayout(container)
        self._main_layout.setSpacing(8)
        self._setup_static_ui()
        self.setWidget(container)

    # ------------------------------------------------------------------
    # Construction UI statique (sections toujours présentes)
    # ------------------------------------------------------------------

    def _setup_static_ui(self) -> None:
        # ── Identité ──────────────────────────────────────────────────
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

        # ── Base fixe ─────────────────────────────────────────────────
        base_box = QGroupBox("Base (élément fixe)")
        base_layout = QVBoxLayout(base_box)
        base_layout.setSpacing(6)

        cad_row = QWidget()
        cad_layout = QHBoxLayout(cad_row)
        cad_layout.setContentsMargins(0, 0, 0, 0)
        self._base_cad_line = QLineEdit()
        self._base_cad_line.setReadOnly(True)
        self._base_cad_line.setPlaceholderText("Aucun fichier STL")
        btn_base = QPushButton("…")
        btn_base.setFixedWidth(30)
        btn_base.clicked.connect(self._browse_base_cad)
        self._base_color_btn = _make_color_button((0.3, 0.3, 0.35, 1.0))
        self._base_color_btn.clicked.connect(self._pick_base_color)
        cad_layout.addWidget(QLabel("STL :"))
        cad_layout.addWidget(self._base_cad_line)
        cad_layout.addWidget(btn_base)
        cad_layout.addWidget(self._base_color_btn)
        base_layout.addWidget(cad_row)

        self._base_pose_editor = Pose6EditorWidget("Position dans le parent")
        self._base_pose_editor.pose_changed.connect(self._emit)
        base_layout.addWidget(self._base_pose_editor)

        self._main_layout.addWidget(base_box)

        # ── Zone dynamique pour les sections articulaires ─────────────
        self._joints_container = QWidget()
        self._joints_layout = QVBoxLayout(self._joints_container)
        self._joints_layout.setSpacing(6)
        self._joints_layout.setContentsMargins(0, 0, 0, 0)
        self._main_layout.addWidget(self._joints_container)

        # Bouton ajout 2ème axe
        self._btn_add_joint = QPushButton("+ Ajouter un 2ème degré de liberté")
        self._btn_add_joint.clicked.connect(self._add_second_joint)
        self._main_layout.addWidget(self._btn_add_joint)

        self._main_layout.addStretch()

    # ------------------------------------------------------------------
    # Gestion dynamique des sections articulaires
    # ------------------------------------------------------------------

    def _rebuild_joint_sections(self, joints: list[ExternalAxisJoint], axis_frame: Pose6) -> None:
        """Recrée les sections articulaires à partir de la liste de joints."""
        self._clear_joint_sections()
        for i, joint in enumerate(joints[:2]):  # max 2 joints dans l'UI simplifiée
            self._append_joint_section(i, joint, axis_frame if i == 0 else None)
        self._refresh_add_button()

    def _append_joint_section(
        self,
        index: int,
        joint: ExternalAxisJoint,
        axis_frame: Pose6 | None,
    ) -> None:
        section = _JointSectionWidget(index)
        section.changed.connect(self._emit)
        section.value_changed.connect(lambda v, i=index: self._on_joint_value_changed(i, v))
        section.set_data(joint, axis_frame)

        # Bouton supprimer uniquement pour joint 1 (index > 0)
        if index > 0:
            btn_del = QPushButton(f"- Supprimer le {index + 1}ème degré de liberté")
            btn_del.clicked.connect(lambda: self._remove_second_joint())
            self._joints_layout.addWidget(btn_del)
            section.setProperty("del_btn", btn_del)

        self._joints_layout.addWidget(section)
        self._joint_sections.append(section)

    def _clear_joint_sections(self) -> None:
        self._joint_sections.clear()
        while self._joints_layout.count():
            item = self._joints_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

    def _refresh_add_button(self) -> None:
        self._btn_add_joint.setVisible(len(self._joint_sections) < 2)

    def _add_second_joint(self) -> None:
        if len(self._joint_sections) >= 2:
            return
        from models.external_axis_joint import ExternalAxisJoint as EAJ
        new_joint = EAJ(joint_type=ExternalAxisJointType.ROTARY, axis=(0, 0, 1),
                        q_min=-180.0, q_max=180.0)
        self._append_joint_section(1, new_joint, None)
        self._refresh_add_button()
        self._emit()

    def _remove_second_joint(self) -> None:
        if len(self._joint_sections) < 2:
            return
        section = self._joint_sections.pop()
        # Disposition dans le layout : [..., btn_del, section]
        # On retire les 2 derniers items
        for _ in range(2):
            count = self._joints_layout.count()
            if count == 0:
                break
            item = self._joints_layout.takeAt(count - 1)
            if item and item.widget():
                item.widget().deleteLater()
        del section  # forcer la suppression
        self._refresh_add_button()
        self._emit()

    # ------------------------------------------------------------------
    # Signaux
    # ------------------------------------------------------------------

    def _on_joint_value_changed(self, joint_index: int, value: float) -> None:
        if not self._building:
            self.joint_value_changed.emit(joint_index, value)

    def _emit(self, *_) -> None:
        if not self._building:
            self.axis_changed.emit()

    # ------------------------------------------------------------------
    # Browsing CAO
    # ------------------------------------------------------------------

    def _browse_base_cad(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Sélectionner STL base", _STL_DEFAULT,
            "STL files (*.stl);;All files (*)"
        )
        if path:
            self._base_cad_line.setText(_normalize_project_path(path))
            self._emit()

    def _pick_base_color(self) -> None:
        if _pick_color(self._base_color_btn, self):
            self._emit()

    # ------------------------------------------------------------------
    # Accès données
    # ------------------------------------------------------------------

    def get_axis(self, current_id: str) -> ExternalAxis:
        parent_id = self._parent_combo.currentData()
        axis_frame = (
            self._joint_sections[0].get_axis_frame()
            if self._joint_sections
            else Pose6.zeros()
        )
        joints = [s.get_joint() for s in self._joint_sections]
        base_color = tuple(
            self._base_color_btn.property("color_rgba") or [0.3, 0.3, 0.35, 1.0]
        )
        return ExternalAxis(
            name=self._name_edit.text().strip() or "Axe externe",
            axis_id=current_id,
            mount_parent_id=parent_id if parent_id != "None" and parent_id else None,
            mount_mode=self._mode_combo.currentData(),
            base_cad_model=self._base_cad_line.text(),
            base_cad_color=base_color,
            base_pose_in_parent=self._base_pose_editor.get_pose(),
            axis_frame_in_base=axis_frame,
            joints=joints,
        )

    def set_axis(self, axis: ExternalAxis, other_axes: list[ExternalAxis]) -> None:
        self._building = True

        self._name_edit.setText(axis.name)

        idx = 0 if axis.mount_mode == ExternalAxisMountMode.POSITIONED else 1
        self._mode_combo.setCurrentIndex(idx)

        self._parent_combo.blockSignals(True)
        self._parent_combo.clear()
        self._parent_combo.addItem("Monde (libre)", None)
        for other in other_axes:
            if other.id != axis.id:
                self._parent_combo.addItem(other.name, other.id)
        for i in range(self._parent_combo.count()):
            if self._parent_combo.itemData(i) == axis.mount_parent_id:
                self._parent_combo.setCurrentIndex(i)
                break
        self._parent_combo.blockSignals(False)

        self._base_cad_line.setText(axis.base_cad_model)
        rgba = list(axis.base_cad_color)
        self._base_color_btn.setStyleSheet(_color_to_style(axis.base_cad_color))
        self._base_color_btn.setProperty("color_rgba", rgba)
        self._base_pose_editor.set_pose(axis.base_pose_in_parent)

        self._rebuild_joint_sections(axis.joints, axis.axis_frame_in_base)

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
