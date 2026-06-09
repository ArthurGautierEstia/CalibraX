from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from models.program_generation_settings import ProgramGenerationSettings
from models.types.approach_retract import ApproachAxisRef, ApproachRetractConfig
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

        # ── Settings group ──────────────────────────────────────────────────
        self._group_settings = QGroupBox("Réglages génération programme")
        group_layout = QVBoxLayout(self._group_settings)
        group_layout.setSpacing(4)

        self._cb_home = QCheckBox("Activer HOME (position de départ / fin)")
        self._cb_home.setChecked(True)
        group_layout.addWidget(self._cb_home)

        approach_box = self._build_motion_box("Approche")
        self._cb_approach, self._combo_approach_ref, self._spin_approach_dist, self._spin_approach_speed = (
            approach_box[0], approach_box[1], approach_box[2], approach_box[3]
        )
        group_layout.addWidget(approach_box[4])

        retract_box = self._build_motion_box("Retrait")
        self._cb_retract, self._combo_retract_ref, self._spin_retract_dist, self._spin_retract_speed = (
            retract_box[0], retract_box[1], retract_box[2], retract_box[3]
        )
        group_layout.addWidget(retract_box[4])

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

        # ── Header zone ─────────────────────────────────────────────────────
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

        # ── KRL Preview ──────────────────────────────────────────────────────
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

    @staticmethod
    def _build_motion_box(
        label: str,
    ) -> tuple[QCheckBox, QComboBox, QDoubleSpinBox, QDoubleSpinBox, QGroupBox]:
        box = QGroupBox(label)
        form = QFormLayout(box)
        form.setContentsMargins(6, 4, 6, 4)
        cb = QCheckBox("Activé(e)")
        combo = QComboBox()
        for ref in ApproachAxisRef:
            combo.addItem(_AXIS_REF_LABELS[ref], ref.value)
        spin_dist = QDoubleSpinBox()
        spin_dist.setRange(0.0, 2000.0)
        spin_dist.setDecimals(1)
        spin_dist.setSuffix(" mm")
        spin_dist.setValue(50.0)
        spin_speed = QDoubleSpinBox()
        spin_speed.setRange(0.001, 5.0)
        spin_speed.setDecimals(3)
        spin_speed.setSuffix(" m/s")
        spin_speed.setValue(0.2)
        form.addRow(cb)
        form.addRow("Axe :", combo)
        form.addRow("Distance :", spin_dist)
        form.addRow("Vitesse :", spin_speed)
        return cb, combo, spin_dist, spin_speed, box

    def _setup_connections(self) -> None:
        self._btn_toggle_header.toggled.connect(self._on_header_toggled)
        self._btn_toggle_preview.toggled.connect(self._on_preview_toggled)
        self._btn_save_header.clicked.connect(self._on_save_header)
        self._btn_reset_header.clicked.connect(self.resetHeaderRequested.emit)

        for cb in (self._cb_home, self._cb_approach, self._cb_retract):
            cb.stateChanged.connect(self._emit_settings_changed)
        for combo in (self._combo_approach_ref, self._combo_retract_ref, self._combo_approx_mode):
            combo.currentIndexChanged.connect(self._emit_settings_changed)
        for spin in (
            self._spin_approach_dist, self._spin_approach_speed,
            self._spin_retract_dist, self._spin_retract_speed,
            self._spin_approx_value,
        ):
            spin.valueChanged.connect(self._emit_settings_changed)

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

    # ── Public API ──────────────────────────────────────────────────────────

    def get_settings(self) -> ProgramGenerationSettings:
        return ProgramGenerationSettings(
            home_enabled=self._cb_home.isChecked(),
            approach=ApproachRetractConfig(
                enabled=self._cb_approach.isChecked(),
                axis_ref=ApproachAxisRef(self._combo_approach_ref.currentData()),
                distance_mm=self._spin_approach_dist.value(),
                speed_mps=self._spin_approach_speed.value(),
            ),
            retract=ApproachRetractConfig(
                enabled=self._cb_retract.isChecked(),
                axis_ref=ApproachAxisRef(self._combo_retract_ref.currentData()),
                distance_mm=self._spin_retract_dist.value(),
                speed_mps=self._spin_retract_speed.value(),
            ),
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
            _set_combo(self._combo_approach_ref, settings.approach.axis_ref.value)
            self._cb_approach.setChecked(settings.approach.enabled)
            self._spin_approach_dist.setValue(settings.approach.distance_mm)
            self._spin_approach_speed.setValue(settings.approach.speed_mps)
            _set_combo(self._combo_retract_ref, settings.retract.axis_ref.value)
            self._cb_retract.setChecked(settings.retract.enabled)
            self._spin_retract_dist.setValue(settings.retract.distance_mm)
            self._spin_retract_speed.setValue(settings.retract.speed_mps)
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
