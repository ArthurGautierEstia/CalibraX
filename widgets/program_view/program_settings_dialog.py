from __future__ import annotations

from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import (
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)

from models.robot_program import ProgramBaseSource
from models.types import Pose6
from widgets.program_view.program_generation_widget import ProgramGenerationWidget

_AXIS_LABELS = ("X", "Y", "Z", "A", "B", "C")
_ORIENT_LABELS = ("A", "B", "C")
_EXT_AXIS_PREFIX = "EXT:"


class _PoseEditRow(QWidget):
    """Ligne compacte de 6 spinboxes XYZABC. Commit sur Entrée / perte de focus / flèches
    (keyboardTracking désactivé : pas d'émission pendant la frappe)."""

    committed = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(0)
        self._spinboxes: list[QDoubleSpinBox] = []
        for index, label in enumerate(_AXIS_LABELS):
            grid.addWidget(QLabel(label), 0, index, alignment=Qt.AlignmentFlag.AlignHCenter)
            spinbox = QDoubleSpinBox(self)
            if index < 3:
                spinbox.setRange(-100000.0, 100000.0)
                spinbox.setSingleStep(1.0)
            else:
                spinbox.setRange(-360.0, 360.0)
                spinbox.setSingleStep(0.1)
            spinbox.setDecimals(3)
            spinbox.setKeyboardTracking(False)
            spinbox.valueChanged.connect(self._on_value_committed)
            grid.addWidget(spinbox, 1, index)
            self._spinboxes.append(spinbox)

    def _on_value_committed(self, _value: float) -> None:
        self.committed.emit()

    def get_pose(self) -> Pose6:
        return Pose6.from_values([float(s.value()) for s in self._spinboxes])

    def set_pose(self, pose: Pose6) -> None:
        for spinbox, value in zip(self._spinboxes, pose.to_list()):
            spinbox.blockSignals(True)
            spinbox.setValue(float(value))
            spinbox.blockSignals(False)

    def set_enabled_all(self, enabled: bool) -> None:
        for spinbox in self._spinboxes:
            spinbox.setEnabled(enabled)


class _OrientationEditRow(QWidget):
    """Ligne compacte de 3 spinboxes ABC (deg)."""

    committed = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(0)
        self._spinboxes: list[QDoubleSpinBox] = []
        for index, label in enumerate(_ORIENT_LABELS):
            grid.addWidget(QLabel(label), 0, index, alignment=Qt.AlignmentFlag.AlignHCenter)
            spinbox = QDoubleSpinBox(self)
            spinbox.setRange(-360.0, 360.0)
            spinbox.setSingleStep(0.1)
            spinbox.setDecimals(3)
            spinbox.setKeyboardTracking(False)
            spinbox.valueChanged.connect(self._on_value_committed)
            grid.addWidget(spinbox, 1, index)
            self._spinboxes.append(spinbox)

    def _on_value_committed(self, _value: float) -> None:
        self.committed.emit()

    def get_orientation(self) -> tuple[float, float, float]:
        return (float(self._spinboxes[0].value()), float(self._spinboxes[1].value()), float(self._spinboxes[2].value()))

    def set_orientation(self, a: float, b: float, c: float) -> None:
        for spinbox, value in zip(self._spinboxes, (a, b, c)):
            spinbox.blockSignals(True)
            spinbox.setValue(value)
            spinbox.blockSignals(False)


class ProgramBaseSectionWidget(QGroupBox):
    """Section base programme : repère de référence + offsets XYZABC.

    L'offset est appliqué par T_source @ T_offset (rotations composées avant translations).
    """

    baseConfigChanged = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("Base programme", parent)
        self._updating = False

        layout = QVBoxLayout(self)
        form = QFormLayout()
        self._source_combo = QComboBox(self)
        self._rebuild_source_items([])
        form.addRow("Repère de référence :", self._source_combo)
        layout.addLayout(form)

        self._manual_label = QLabel("Base manuelle (repère robot) :")
        self._manual_row = _PoseEditRow(self)
        layout.addWidget(self._manual_label)
        layout.addWidget(self._manual_row)

        layout.addWidget(QLabel("Offset par rapport au repère de référence :"))
        self._offset_row = _PoseEditRow(self)
        layout.addWidget(self._offset_row)

        self._source_combo.currentIndexChanged.connect(self._on_source_changed)
        self._manual_row.committed.connect(self._emit_changed)
        self._offset_row.committed.connect(self._emit_changed)
        self._update_manual_visibility()

    def _rebuild_source_items(self, external_axes: list[tuple[str, str]]) -> None:
        current_data = self._source_combo.currentData() if self._source_combo.count() else None
        self._source_combo.blockSignals(True)
        self._source_combo.clear()
        self._source_combo.addItem("Repère monde", ProgramBaseSource.WORLD.value)
        self._source_combo.addItem("Repère robot", ProgramBaseSource.ROBOT.value)
        self._source_combo.addItem("Repère du fichier programme", ProgramBaseSource.PROGRAM_FILE.value)
        self._source_combo.addItem("Repère pièce", ProgramBaseSource.WORKPIECE.value)
        self._source_combo.addItem("Manuel", ProgramBaseSource.MANUAL.value)
        for axis_id, axis_name in external_axes:
            self._source_combo.addItem(f"Axe externe : {axis_name}", f"{_EXT_AXIS_PREFIX}{axis_id}")
        if current_data is not None:
            index = self._source_combo.findData(current_data)
            if index >= 0:
                self._source_combo.setCurrentIndex(index)
        self._source_combo.blockSignals(False)

    def set_external_axes(self, external_axes: list[tuple[str, str]]) -> None:
        self._rebuild_source_items(external_axes)

    def _on_source_changed(self, _index: int) -> None:
        self._update_manual_visibility()
        self._emit_changed()

    def _update_manual_visibility(self) -> None:
        is_manual = self._source_combo.currentData() == ProgramBaseSource.MANUAL.value
        self._manual_label.setVisible(is_manual)
        self._manual_row.setVisible(is_manual)

    def _emit_changed(self) -> None:
        if not self._updating:
            self.baseConfigChanged.emit()

    def get_base_config(self) -> tuple[ProgramBaseSource, str | None, Pose6, Pose6]:
        """Retourne (source, axe_externe_id, base_manuelle, offset)."""
        data = str(self._source_combo.currentData())
        if data.startswith(_EXT_AXIS_PREFIX):
            return (
                ProgramBaseSource.EXTERNAL_AXIS,
                data[len(_EXT_AXIS_PREFIX):],
                self._manual_row.get_pose(),
                self._offset_row.get_pose(),
            )
        return (
            ProgramBaseSource(data),
            None,
            self._manual_row.get_pose(),
            self._offset_row.get_pose(),
        )

    def set_base_config(
        self,
        source: ProgramBaseSource,
        ext_axis_id: str | None,
        manual_base: Pose6,
        offset: Pose6,
    ) -> None:
        self._updating = True
        try:
            data = (
                f"{_EXT_AXIS_PREFIX}{ext_axis_id}"
                if source == ProgramBaseSource.EXTERNAL_AXIS and ext_axis_id
                else source.value
            )
            index = self._source_combo.findData(data)
            if index >= 0:
                self._source_combo.setCurrentIndex(index)
            self._manual_row.set_pose(manual_base)
            self._offset_row.set_pose(offset)
            self._update_manual_visibility()
        finally:
            self._updating = False


class ProgramToolSectionWidget(QGroupBox):
    """Section outil : outil courant de la config, outil lu dans le programme,
    ou outil spécifique XYZABC (par rapport au flange). Avec option d'override
    d'orientation pour toutes les cibles cartésiennes."""

    toolConfigChanged = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__("Outil ($TOOL)", parent)
        self._updating = False

        layout = QVBoxLayout(self)
        form = QFormLayout()
        self._source_combo = QComboBox(self)
        self._source_combo.addItem("Outil courant (config)", "CURRENT")
        self._source_combo.addItem("Outil du programme", "PROGRAM")
        self._source_combo.addItem("Outil personnalisé", "CUSTOM")
        form.addRow("Source :", self._source_combo)
        layout.addLayout(form)

        self._custom_label = QLabel("Outil personnalisé (flange → TCP) :")
        self._custom_row = _PoseEditRow(self)
        self._custom_label.setVisible(False)
        self._custom_row.setVisible(False)
        layout.addWidget(self._custom_label)
        layout.addWidget(self._custom_row)

        # QGroupBox checkable : le titre-checkbox est séparé physiquement des spinboxes,
        # ce qui évite les toggles accidentels quand l'utilisateur clique à côté d'une spinbox.
        self._orientation_group = QGroupBox("Définir l'orientation outil")
        self._orientation_group.setCheckable(True)
        self._orientation_group.setChecked(False)
        orient_layout = QVBoxLayout(self._orientation_group)
        self._orientation_label = QLabel("Orientation appliquée aux cibles (A, B, C, deg) :")
        self._orientation_row = _OrientationEditRow(self._orientation_group)
        self._orientation_label.setVisible(False)
        self._orientation_row.setVisible(False)
        orient_layout.addWidget(self._orientation_label)
        orient_layout.addWidget(self._orientation_row)
        layout.addWidget(self._orientation_group)

        self._source_combo.currentIndexChanged.connect(self._on_source_changed)
        self._custom_row.committed.connect(self._emit_changed)
        self._orientation_group.toggled.connect(self._on_orientation_toggled)
        self._orientation_row.committed.connect(self._emit_changed)

    def _on_source_changed(self, _index: int) -> None:
        is_custom = self._source_combo.currentData() == "CUSTOM"
        self._custom_label.setVisible(is_custom)
        self._custom_row.setVisible(is_custom)
        self._emit_changed()

    def _on_orientation_toggled(self, checked: bool) -> None:
        self._orientation_label.setVisible(checked)
        self._orientation_row.setVisible(checked)
        self._emit_changed()

    def _emit_changed(self) -> None:
        if not self._updating:
            self.toolConfigChanged.emit()

    def get_tool_config(self) -> tuple[str, Pose6, Pose6 | None]:
        """Retourne (source, pose_outil_personnalisé, orientation_override_ou_None)."""
        source = str(self._source_combo.currentData())
        custom_pose = self._custom_row.get_pose()
        if self._orientation_group.isChecked():
            a, b, c = self._orientation_row.get_orientation()
            orientation_override: Pose6 | None = Pose6(x=0.0, y=0.0, z=0.0, a=a, b=b, c=c)
        else:
            orientation_override = None
        return source, custom_pose, orientation_override

    def set_tool_config(self, source: str, custom_pose: Pose6, orientation_override: Pose6 | None = None) -> None:
        self._updating = True
        try:
            index = self._source_combo.findData(source)
            if index >= 0:
                self._source_combo.setCurrentIndex(index)
            self._custom_row.set_pose(custom_pose)
            is_custom = source == "CUSTOM"
            self._custom_label.setVisible(is_custom)
            self._custom_row.setVisible(is_custom)

            has_orientation = orientation_override is not None
            self._orientation_group.blockSignals(True)
            self._orientation_group.setChecked(has_orientation)
            self._orientation_group.blockSignals(False)
            if has_orientation and orientation_override is not None:
                self._orientation_row.set_orientation(orientation_override.a, orientation_override.b, orientation_override.c)
            self._orientation_label.setVisible(has_orientation)
            self._orientation_row.setVisible(has_orientation)
        finally:
            self._updating = False

    def set_program_tool_available(self, available: bool) -> None:
        index = self._source_combo.findData("PROGRAM")
        if index >= 0:
            item = self._source_combo.model().item(index)
            if item is not None:
                item.setEnabled(available)


class ProgramSettingsDialog(QDialog):
    """Dialog NON bloquant : base programme, outil, vitesse, approximation,
    approche/retrait, header KRL et preview KRL.

    Le viewer reste interactif pendant l'édition (cibles mises à jour en live).
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("Paramètres programme")
        self.setWindowFlags(
            Qt.WindowType.Window
            | Qt.WindowType.WindowCloseButtonHint
            | Qt.WindowType.WindowMinimizeButtonHint
        )
        self.setModal(False)
        self.setMinimumWidth(640)
        self.setMinimumHeight(560)

        self.base_section = ProgramBaseSectionWidget()
        self.tool_section = ProgramToolSectionWidget()
        self._generation_widget = ProgramGenerationWidget()

        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(8, 8, 8, 8)
        content_layout.setSpacing(8)
        content_layout.addWidget(self.base_section)
        content_layout.addWidget(self.tool_section)
        content_layout.addWidget(self._generation_widget)
        content_layout.addStretch()

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(scroll)

    def get_generation_widget(self) -> ProgramGenerationWidget:
        return self._generation_widget
