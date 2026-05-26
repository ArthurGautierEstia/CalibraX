"""Widget d'édition d'une articulation ExternalAxisJoint."""
from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox,
    QHBoxLayout, QLabel, QLineEdit, QPushButton, QWidget,
)

from models.external_axis_joint import ExternalAxisJoint
from models.types.external_axis_joint_type import ExternalAxisJointType
from models.types.pose6 import Pose6
from utils.math_utils import normalize3
from widgets.external_axes_view.pose6_editor_widget import Pose6EditorWidget


class JointEditorWidget(QGroupBox):
    """Éditeur pour un ExternalAxisJoint (type, axe, pose, limites, CAO)."""
    changed = pyqtSignal()

    def __init__(self, joint_index: int, parent: QWidget | None = None) -> None:
        super().__init__(f"Joint {joint_index + 1}", parent)
        self._building = False
        self._setup_ui()

    def _setup_ui(self) -> None:
        from PyQt6.QtWidgets import QFileDialog
        form = QFormLayout(self)
        form.setSpacing(4)

        # Type
        self._type_combo = QComboBox()
        self._type_combo.addItem("Prismatique (linéaire)", ExternalAxisJointType.LINEAR)
        self._type_combo.addItem("Rotoïde (rotatif)", ExternalAxisJointType.ROTARY)
        self._type_combo.currentIndexChanged.connect(self._emit)
        form.addRow("Type :", self._type_combo)

        # Axe (vecteur unitaire)
        axis_widget = QWidget()
        axis_layout = QHBoxLayout(axis_widget)
        axis_layout.setContentsMargins(0, 0, 0, 0)
        self._axis_x = self._make_axis_spin()
        self._axis_y = self._make_axis_spin()
        self._axis_z = self._make_axis_spin()
        self._axis_z.setValue(1.0)
        for sb in (self._axis_x, self._axis_y, self._axis_z):
            axis_layout.addWidget(sb)
        btn_norm = QPushButton("⊥ Norm.")
        btn_norm.setFixedWidth(60)
        btn_norm.clicked.connect(self._normalize_axis)
        axis_layout.addWidget(btn_norm)
        form.addRow("Axe (XYZ) :", axis_widget)

        # Pose lien dans le joint précédent
        self._pose_editor = Pose6EditorWidget("Pose lien dans joint précédent")
        self._pose_editor.pose_changed.connect(self._emit)
        form.addRow(self._pose_editor)

        # CAO
        cad_row = QWidget()
        cad_layout = QHBoxLayout(cad_row)
        cad_layout.setContentsMargins(0, 0, 0, 0)
        self._cad_line = QLineEdit()
        self._cad_line.setReadOnly(True)
        self._cad_line.setPlaceholderText("Aucun fichier STL")
        btn_cad = QPushButton("…")
        btn_cad.setFixedWidth(30)
        btn_cad.clicked.connect(self._browse_cad)
        cad_layout.addWidget(self._cad_line)
        cad_layout.addWidget(btn_cad)
        form.addRow("CAO (STL) :", cad_row)

        # Limites
        limits_widget = QWidget()
        limits_layout = QHBoxLayout(limits_widget)
        limits_layout.setContentsMargins(0, 0, 0, 0)
        self._q_min = self._make_limit_spin(-99999)
        self._q_max = self._make_limit_spin(99999)
        self._q_min.setValue(-1000.0)
        self._q_max.setValue(1000.0)
        self._q_min.valueChanged.connect(self._emit)
        self._q_max.valueChanged.connect(self._emit)
        limits_layout.addWidget(QLabel("Min"))
        limits_layout.addWidget(self._q_min)
        limits_layout.addWidget(QLabel("Max"))
        limits_layout.addWidget(self._q_max)
        form.addRow("Limites :", limits_widget)

        # Offset zéro
        self._offset_spin = self._make_limit_spin(-99999)
        self._offset_spin.valueChanged.connect(self._emit)
        form.addRow("Offset (zéro) :", self._offset_spin)

    def _make_axis_spin(self) -> QDoubleSpinBox:
        sb = QDoubleSpinBox()
        sb.setRange(-1.0, 1.0)
        sb.setDecimals(3)
        sb.setSingleStep(0.1)
        sb.valueChanged.connect(self._emit)
        return sb

    def _make_limit_spin(self, limit: float) -> QDoubleSpinBox:
        sb = QDoubleSpinBox()
        sb.setRange(-abs(limit), abs(limit))
        sb.setDecimals(2)
        sb.setSingleStep(1.0)
        return sb

    def _normalize_axis(self) -> None:
        v = normalize3([self._axis_x.value(), self._axis_y.value(), self._axis_z.value()])
        self._building = True
        self._axis_x.setValue(v[0])
        self._axis_y.setValue(v[1])
        self._axis_z.setValue(v[2])
        self._building = False
        self._emit()

    def _browse_cad(self) -> None:
        from PyQt6.QtWidgets import QFileDialog
        path, _ = QFileDialog.getOpenFileName(self, "Sélectionner STL", "", "STL files (*.stl);;All files (*)")
        if path:
            self._cad_line.setText(path)
            self._emit()

    def _emit(self, *_) -> None:
        if not self._building:
            self.changed.emit()

    # ------------------------------------------------------------------

    def get_joint(self) -> ExternalAxisJoint:
        return ExternalAxisJoint(
            joint_type=self._type_combo.currentData(),
            axis=(self._axis_x.value(), self._axis_y.value(), self._axis_z.value()),
            link_pose_in_prev=self._pose_editor.get_pose(),
            cad_model=self._cad_line.text(),
            q_min=self._q_min.value(),
            q_max=self._q_max.value(),
            offset=self._offset_spin.value(),
            value=0.0,
        )

    def set_joint(self, joint: ExternalAxisJoint) -> None:
        self._building = True
        idx = 0 if joint.joint_type == ExternalAxisJointType.LINEAR else 1
        self._type_combo.setCurrentIndex(idx)
        self._axis_x.setValue(joint.axis[0])
        self._axis_y.setValue(joint.axis[1])
        self._axis_z.setValue(joint.axis[2])
        self._pose_editor.set_pose(joint.link_pose_in_prev)
        self._cad_line.setText(joint.cad_model)
        self._q_min.setValue(joint.q_min)
        self._q_max.setValue(joint.q_max)
        self._offset_spin.setValue(joint.offset)
        self._building = False
