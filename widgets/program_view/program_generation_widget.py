from __future__ import annotations

from functools import partial

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from models.program_generation_settings import ProgramGenerationSettings
from models.types.approach_retract import ApproachAxisRef, ApproachRetractConfig, ApproachRetractStep
from models.types.motion_approximation import ApproximationMode, MotionApproximation


_AXIS_REF_LABELS: dict[ApproachAxisRef, str] = {
    ApproachAxisRef.TOOL_Z: "Z outil",
    ApproachAxisRef.PIECE_X: "X pièce",
    ApproachAxisRef.PIECE_Y: "Y pièce",
    ApproachAxisRef.PIECE_Z: "Z pièce",
}

_APPROX_MODE_LABELS: dict[ApproximationMode, str] = {
    ApproximationMode.NONE: "Aucune",
    ApproximationMode.C_DIS: "C_DIS (mm)",
    ApproximationMode.C_VEL: "C_VEL (%)",
}

_DEFAULT_STEP = ApproachRetractStep(ApproachAxisRef.TOOL_Z, 50.0, 0.2)


class _StepRow(QWidget):
    """Ligne représentant un pas d'approche/retrait : axe, distance, vitesse, bouton suppr."""

    changed = pyqtSignal()
    removed = pyqtSignal()

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        self._combo = QComboBox()
        for ref in ApproachAxisRef:
            self._combo.addItem(_AXIS_REF_LABELS[ref], ref.value)

        self._spin_dist = QDoubleSpinBox()
        self._spin_dist.setRange(0.0, 2000.0)
        self._spin_dist.setDecimals(1)
        self._spin_dist.setSuffix(" mm")
        self._spin_dist.setValue(50.0)

        self._spin_speed = QDoubleSpinBox()
        self._spin_speed.setRange(0.001, 5.0)
        self._spin_speed.setDecimals(3)
        self._spin_speed.setSuffix(" m/s")
        self._spin_speed.setValue(0.2)

        self._btn_invert = QPushButton("⇄")
        self._btn_invert.setCheckable(True)
        self._btn_invert.setFixedWidth(30)
        self._btn_invert.setToolTip("Inverser le sens : actif = sens +axe, inactif = sens -axe (défaut)")

        btn_remove = QPushButton("×")
        btn_remove.setFixedWidth(24)
        btn_remove.setToolTip("Supprimer ce pas")

        layout.addWidget(self._combo, stretch=2)
        layout.addWidget(self._spin_dist, stretch=2)
        layout.addWidget(self._spin_speed, stretch=2)
        layout.addWidget(self._btn_invert)
        layout.addWidget(btn_remove)

        self._combo.currentIndexChanged.connect(self.changed)
        self._spin_dist.editingFinished.connect(self.changed)
        self._spin_speed.editingFinished.connect(self.changed)
        self._btn_invert.toggled.connect(self.changed)
        btn_remove.clicked.connect(self.removed)

    def get_step(self) -> ApproachRetractStep:
        return ApproachRetractStep(
            axis_ref=ApproachAxisRef(self._combo.currentData()),
            distance_mm=self._spin_dist.value(),
            speed_mps=self._spin_speed.value(),
            inverted=self._btn_invert.isChecked(),
        )

    def set_step(self, step: ApproachRetractStep) -> None:
        idx = self._combo.findData(step.axis_ref.value)
        if idx >= 0:
            self._combo.setCurrentIndex(idx)
        self._spin_dist.setValue(step.distance_mm)
        self._spin_speed.setValue(step.speed_mps)
        self._btn_invert.setChecked(step.inverted)


class _ApproachRetractSectionWidget(QWidget):
    """Section approche ou retrait : en-tête repliable, liste de pas, bouton ajout."""

    changed = pyqtSignal()

    def __init__(self, label: str, parent: QWidget = None) -> None:
        super().__init__(parent)
        self._updating = False
        self._step_rows: list[_StepRow] = []

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # En-tête : bouton déplier + label + checkbox activé
        header = QWidget()
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(4)

        self._btn_expand = QPushButton("▶")
        self._btn_expand.setCheckable(True)
        self._btn_expand.setFixedWidth(26)
        self._btn_expand.setFlat(True)

        lbl = QLabel(label)
        font = lbl.font()
        font.setBold(True)
        lbl.setFont(font)

        self._cb_enabled = QCheckBox("Activé(e)")

        header_layout.addWidget(self._btn_expand)
        header_layout.addWidget(lbl)
        header_layout.addStretch()
        header_layout.addWidget(self._cb_enabled)
        layout.addWidget(header)

        # Corps repliable
        self._body = QWidget()
        body_layout = QVBoxLayout(self._body)
        body_layout.setContentsMargins(14, 2, 0, 4)
        body_layout.setSpacing(2)

        self._steps_layout = QVBoxLayout()
        self._steps_layout.setSpacing(2)
        body_layout.addLayout(self._steps_layout)

        self._btn_add = QPushButton("+ Ajouter un pas")
        body_layout.addWidget(self._btn_add)

        layout.addWidget(self._body)
        self._body.setVisible(False)

        self._btn_expand.toggled.connect(self._on_expand_toggled)
        self._cb_enabled.stateChanged.connect(self._emit_changed)
        self._btn_add.clicked.connect(lambda: self._add_step())

    def _on_expand_toggled(self, checked: bool) -> None:
        self._btn_expand.setText("▼" if checked else "▶")
        self._body.setVisible(checked)

    def _emit_changed(self) -> None:
        if not self._updating:
            self.changed.emit()

    def _add_step(self, step: ApproachRetractStep | None = None) -> None:
        row = _StepRow(self)
        if step is not None:
            row.set_step(step)
        row.changed.connect(self._emit_changed)
        row.removed.connect(partial(self._remove_step, row))
        self._steps_layout.addWidget(row)
        self._step_rows.append(row)
        self._emit_changed()

    def _remove_step(self, row: _StepRow) -> None:
        if len(self._step_rows) <= 1:
            return  # Toujours au moins un pas
        self._steps_layout.removeWidget(row)
        row.deleteLater()
        self._step_rows.remove(row)
        self._emit_changed()

    def get_config(self) -> ApproachRetractConfig:
        return ApproachRetractConfig(
            enabled=self._cb_enabled.isChecked(),
            steps=tuple(r.get_step() for r in self._step_rows),
        )

    def set_config(self, cfg: ApproachRetractConfig) -> None:
        self._updating = True
        try:
            self._cb_enabled.setChecked(cfg.enabled)
            for row in self._step_rows:
                self._steps_layout.removeWidget(row)
                row.deleteLater()
            self._step_rows.clear()
            steps = cfg.steps if cfg.steps else (_DEFAULT_STEP,)
            for step in steps:
                self._add_step(step)
        finally:
            self._updating = False


class ProgramGenerationWidget(QWidget):
    """Panneau réglages génération : HOME, approche, retrait, approx par défaut, header, preview KRL."""

    generationSettingsChanged = pyqtSignal(dict)
    saveHeaderRequested = pyqtSignal(str)
    resetHeaderRequested = pyqtSignal()

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)
        self._updating = False
        self._setup_ui()
        self._setup_connections()

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)
        layout.setSpacing(6)

        # Groupe réglages
        self._group_settings = QGroupBox("Réglages génération programme")
        group_layout = QVBoxLayout(self._group_settings)
        group_layout.setSpacing(4)

        self._cb_home = QCheckBox("Activer HOME (position de départ)")
        self._cb_home.setChecked(True)
        group_layout.addWidget(self._cb_home)

        self._approach_section = _ApproachRetractSectionWidget("Approche")
        self._retract_section = _ApproachRetractSectionWidget("Retrait")
        group_layout.addWidget(self._approach_section)
        group_layout.addWidget(self._retract_section)

        approx_box = QGroupBox("Approximation par défaut")
        approx_form = QFormLayout(approx_box)
        approx_form.setContentsMargins(6, 4, 6, 4)
        self._combo_approx_mode = QComboBox()
        for mode in ApproximationMode:
            self._combo_approx_mode.addItem(_APPROX_MODE_LABELS[mode], mode.value)
        self._spin_approx_value = QDoubleSpinBox()
        self._spin_approx_value.setRange(0.0, 1000.0)
        self._spin_approx_value.setDecimals(3)
        self._spin_approx_value.setValue(0.0)
        approx_form.addRow("Mode :", self._combo_approx_mode)
        approx_form.addRow("Valeur :", self._spin_approx_value)
        group_layout.addWidget(approx_box)

        layout.addWidget(self._group_settings)

        # Zone header KRL
        self._btn_toggle_header = QPushButton("Afficher header KRL")
        self._btn_toggle_header.setCheckable(True)
        layout.addWidget(self._btn_toggle_header)

        self._header_zone = QWidget()
        header_layout = QVBoxLayout(self._header_zone)
        header_layout.setContentsMargins(0, 0, 0, 0)
        self._header_edit = QPlainTextEdit()
        self._header_edit.setMinimumHeight(120)
        self._header_edit.setMaximumHeight(200)
        self._header_edit.setPlaceholderText("Template header KRL…")
        mono = QFont("Courier New", 9)
        self._header_edit.setFont(mono)
        header_layout.addWidget(self._header_edit)
        header_btn_row = QHBoxLayout()
        self._btn_save_header = QPushButton("Enregistrer header")
        self._btn_reset_header = QPushButton("Réinitialiser")
        header_btn_row.addWidget(self._btn_save_header)
        header_btn_row.addWidget(self._btn_reset_header)
        header_btn_row.addStretch()
        header_layout.addLayout(header_btn_row)
        self._header_zone.setVisible(False)
        layout.addWidget(self._header_zone)

        # Preview KRL
        self._btn_toggle_preview = QPushButton("Afficher preview KRL")
        self._btn_toggle_preview.setCheckable(True)
        layout.addWidget(self._btn_toggle_preview)

        self._preview_edit = QPlainTextEdit()
        self._preview_edit.setReadOnly(True)
        self._preview_edit.setMinimumHeight(200)
        self._preview_edit.setMaximumHeight(400)
        self._preview_edit.setPlaceholderText("Simulez le programme pour voir la preview KRL…")
        self._preview_edit.setFont(mono)
        self._preview_edit.setVisible(False)
        layout.addWidget(self._preview_edit)

    def _setup_connections(self) -> None:
        self._btn_toggle_header.toggled.connect(self._on_header_toggled)
        self._btn_toggle_preview.toggled.connect(self._on_preview_toggled)
        self._btn_save_header.clicked.connect(self._on_save_header)
        self._btn_reset_header.clicked.connect(self.resetHeaderRequested.emit)

        self._cb_home.stateChanged.connect(self._emit_settings_changed)
        self._combo_approx_mode.currentIndexChanged.connect(self._emit_settings_changed)
        self._spin_approx_value.editingFinished.connect(self._emit_settings_changed)
        self._approach_section.changed.connect(self._emit_settings_changed)
        self._retract_section.changed.connect(self._emit_settings_changed)

    def _on_header_toggled(self, checked: bool) -> None:
        self._header_zone.setVisible(checked)
        self._btn_toggle_header.setText(
            "Masquer header KRL" if checked else "Afficher header KRL"
        )

    def _on_preview_toggled(self, checked: bool) -> None:
        self._preview_edit.setVisible(checked)
        self._btn_toggle_preview.setText(
            "Masquer preview KRL" if checked else "Afficher preview KRL"
        )

    def _on_save_header(self) -> None:
        self.saveHeaderRequested.emit(self._header_edit.toPlainText())

    def _emit_settings_changed(self) -> None:
        if self._updating:
            return
        self.generationSettingsChanged.emit(self.get_settings().to_dict())

    # ── Public API ────────────────────────────────────────────────────────────

    def get_settings(self) -> ProgramGenerationSettings:
        return ProgramGenerationSettings(
            home_enabled=self._cb_home.isChecked(),
            approach=self._approach_section.get_config(),
            retract=self._retract_section.get_config(),
            header_text=self._header_edit.toPlainText(),
            default_approximation=MotionApproximation(
                mode=ApproximationMode(self._combo_approx_mode.currentData()),
                value=self._spin_approx_value.value(),
            ),
        )

    def set_settings(self, settings: ProgramGenerationSettings) -> None:
        self._updating = True
        try:
            self._cb_home.setChecked(settings.home_enabled)
            self._approach_section.set_config(settings.approach)
            self._retract_section.set_config(settings.retract)
            _set_combo(self._combo_approx_mode, settings.default_approximation.mode.value)
            self._spin_approx_value.setValue(settings.default_approximation.value)
        finally:
            self._updating = False

    def set_header_text(self, text: str) -> None:
        self._header_edit.setPlainText(text)

    def get_header_text(self) -> str:
        return self._header_edit.toPlainText()

    def set_krl_preview_text(self, text: str) -> None:
        self._preview_edit.setPlainText(text)


def _set_combo(combo: QComboBox, data_value: str) -> None:
    idx = combo.findData(data_value)
    if idx >= 0:
        combo.setCurrentIndex(idx)
