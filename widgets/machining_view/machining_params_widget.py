from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QAbstractSpinBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from models.types.machining_params import (
    CuttingParams,
    MachiningSimulationParams,
    MATERIAL_PRESETS,
    RobotMechanicalParams,
    ROBOT_PRESETS,
)


class MachiningParamsWidget(QWidget):
    """Saisie des paramètres de simulation d'usinage (conditions de coupe + mécanique robot)."""

    params_changed = pyqtSignal()

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        self._updating_from_preset = False

        # ------------------------------------------------------------------
        # Conditions de coupe — paramètres principaux
        # ------------------------------------------------------------------
        self._ap_spin = self._make_double(0.01, 100.0, 2, 0.5, " mm", 2.0)
        self._ae_spin = self._make_double(0.01, 500.0, 2, 1.0, " mm", 5.0)
        self._diam_spin = self._make_double(1.0, 500.0, 2, 5.0, " mm", 20.0)
        self._z_spin = self._make_int(1, 20, 4)
        self._n_spin = self._make_double(1.0, 100_000.0, 0, 100.0, " tr/min", 3000.0)
        self._vf_spin = self._make_double(1.0, 100_000.0, 1, 100.0, " mm/min", 1200.0)
        self._fz_label = QLabel()
        self._fz_label.setStyleSheet("color: gray; font-style: italic;")

        # ------------------------------------------------------------------
        # Conditions de coupe — détails coefficients Altintas
        # ------------------------------------------------------------------
        self._ktc_spin = self._make_double(1.0, 5000.0, 1, 50.0, " MPa", 550.0)
        self._krc_spin = self._make_double(1.0, 5000.0, 1, 50.0, " MPa", 200.0)
        self._kac_spin = self._make_double(1.0, 5000.0, 1, 50.0, " MPa", 150.0)
        self._kte_spin = self._make_double(0.0, 1000.0, 2, 1.0, " N/mm", 0.0)
        self._kre_spin = self._make_double(0.0, 1000.0, 2, 1.0, " N/mm", 0.0)
        self._kae_spin = self._make_double(0.0, 1000.0, 2, 1.0, " N/mm", 0.0)

        # ------------------------------------------------------------------
        # Mécanique robot — détails raideurs + τ_max
        # ------------------------------------------------------------------
        default_preset = ROBOT_PRESETS[0]
        self._k_spins: list[QDoubleSpinBox] = []
        self._tau_spins: list[QDoubleSpinBox] = []
        self._k_source_labels: list[QLabel] = []
        for i in range(6):
            self._k_spins.append(
                self._make_double(1.0, 1e8, 0, 1e5, " N·m/rad", default_preset.stiffness_Nm_per_rad[i])
            )
            self._tau_spins.append(
                self._make_double(1.0, 50_000.0, 0, 100.0, " N·m", default_preset.torque_max_Nm[i])
            )
            lbl = QLabel(default_preset.stiffness_sources[i])
            lbl.setStyleSheet("color: gray; font-size: 9px;")
            self._k_source_labels.append(lbl)

        # Liste unifiée de tous les spinboxes (pour blockSignals global)
        self._all_spins: list[QAbstractSpinBox] = [
            self._ap_spin, self._ae_spin, self._diam_spin, self._z_spin,
            self._n_spin, self._vf_spin,
            self._ktc_spin, self._krc_spin, self._kac_spin,
            self._kte_spin, self._kre_spin, self._kae_spin,
        ] + self._k_spins + self._tau_spins

        self._setup_ui()
        self._setup_connections()
        self._update_fz_label()

    # ------------------------------------------------------------------
    # Construction de l'interface
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        row = QHBoxLayout()
        row.addWidget(self._build_cutting_group())
        row.addWidget(self._build_mechanical_group())
        layout.addLayout(row)

    def _build_cutting_group(self) -> QGroupBox:
        group = QGroupBox("Conditions de coupe")
        layout = QVBoxLayout(group)

        # Sélecteur de matériau
        self._material_combo = QComboBox()
        for p in MATERIAL_PRESETS:
            self._material_combo.addItem(p.name)
        self._material_combo.addItem("Personnalisé")

        self._material_details_btn = QPushButton("Détails ▾")
        self._material_details_btn.setCheckable(True)
        self._material_details_btn.setFixedWidth(90)

        combo_row = QHBoxLayout()
        combo_row.addWidget(QLabel("Matériau :"))
        combo_row.addWidget(self._material_combo, 1)
        combo_row.addWidget(self._material_details_btn)
        layout.addLayout(combo_row)

        # Séparateur
        layout.addWidget(self._make_separator())

        # Paramètres principaux
        grid = QGridLayout()
        grid.setColumnStretch(1, 1)
        main_rows = [
            ("Profondeur axiale a_p",  self._ap_spin),
            ("Engagement radial a_e",  self._ae_spin),
            ("Diamètre outil D",        self._diam_spin),
            ("Nombre de dents z",       self._z_spin),
            ("Vitesse broche N",        self._n_spin),
            ("Vitesse d'avance v_f",   self._vf_spin),
        ]
        for r, (label, widget) in enumerate(main_rows):
            grid.addWidget(QLabel(label), r, 0)
            grid.addWidget(widget, r, 1)
        # Ligne f_z calculée (lecture seule)
        grid.addWidget(QLabel("→ Avance/dent f_z"), len(main_rows), 0)
        grid.addWidget(self._fz_label, len(main_rows), 1)
        layout.addLayout(grid)

        # Zone détails coefficients (repliée par défaut)
        self._material_details_widget = QWidget()
        details_grid = QGridLayout(self._material_details_widget)
        details_grid.setContentsMargins(0, 4, 0, 0)
        k_rows = [
            ("K_tc (cisaillement tang.)", self._ktc_spin),
            ("K_rc (cisaillement rad.)",  self._krc_spin),
            ("K_ac (cisaillement axial)", self._kac_spin),
            ("K_te (arête tang.)",        self._kte_spin),
            ("K_re (arête rad.)",         self._kre_spin),
            ("K_ae (arête axiale)",       self._kae_spin),
        ]
        for r, (label, widget) in enumerate(k_rows):
            details_grid.addWidget(QLabel(label), r, 0)
            details_grid.addWidget(widget, r, 1)

        layout.addWidget(self._make_separator())
        layout.addWidget(self._material_details_widget)
        self._material_details_widget.setVisible(False)

        return group

    def _build_mechanical_group(self) -> QGroupBox:
        group = QGroupBox("Mécanique robot")
        layout = QVBoxLayout(group)

        # Sélecteur de modèle robot
        self._robot_combo = QComboBox()
        for p in ROBOT_PRESETS:
            self._robot_combo.addItem(p.name)
        self._robot_combo.addItem("Personnalisé")

        self._robot_details_btn = QPushButton("Détails ▾")
        self._robot_details_btn.setCheckable(True)
        self._robot_details_btn.setFixedWidth(90)

        combo_row = QHBoxLayout()
        combo_row.addWidget(QLabel("Modèle :"))
        combo_row.addWidget(self._robot_combo, 1)
        combo_row.addWidget(self._robot_details_btn)
        layout.addLayout(combo_row)

        # Zone détails raideurs (repliée par défaut)
        self._robot_details_widget = QWidget()
        details_grid = QGridLayout(self._robot_details_widget)
        details_grid.setContentsMargins(0, 4, 0, 0)
        details_grid.addWidget(QLabel("Axe"),            0, 0)
        details_grid.addWidget(QLabel("Raideur k"),      0, 1)
        details_grid.addWidget(QLabel("τ_max"),          0, 2)
        details_grid.addWidget(QLabel("Source"),         0, 3)
        for i in range(6):
            details_grid.addWidget(QLabel(f"J{i + 1}"),       i + 1, 0)
            details_grid.addWidget(self._k_spins[i],           i + 1, 1)
            details_grid.addWidget(self._tau_spins[i],         i + 1, 2)
            details_grid.addWidget(self._k_source_labels[i],   i + 1, 3)

        layout.addWidget(self._make_separator())
        layout.addWidget(self._robot_details_widget)
        self._robot_details_widget.setVisible(False)
        layout.addStretch()

        return group

    # ------------------------------------------------------------------
    # Connexions
    # ------------------------------------------------------------------

    def _setup_connections(self) -> None:
        self._material_details_btn.toggled.connect(self._on_material_details_toggled)
        self._robot_details_btn.toggled.connect(self._on_robot_details_toggled)

        self._material_combo.currentIndexChanged.connect(self._on_material_preset_changed)
        self._robot_combo.currentIndexChanged.connect(self._on_robot_preset_changed)

        # Paramètres principaux → recalcul f_z + signal
        for spin in [self._ap_spin, self._ae_spin, self._diam_spin,
                     self._n_spin, self._vf_spin]:
            spin.valueChanged.connect(self._on_cutting_param_changed)
        self._z_spin.valueChanged.connect(self._on_cutting_param_changed)

        # Détails K → combo "Personnalisé" + signal
        for spin in [self._ktc_spin, self._krc_spin, self._kac_spin,
                     self._kte_spin, self._kre_spin, self._kae_spin]:
            spin.valueChanged.connect(self._on_k_detail_changed)

        # Détails raideurs robot → combo "Personnalisé" + signal
        for spin in self._k_spins + self._tau_spins:
            spin.valueChanged.connect(self._on_robot_detail_changed)

    # ------------------------------------------------------------------
    # Slots internes
    # ------------------------------------------------------------------

    def _on_material_details_toggled(self, checked: bool) -> None:
        self._material_details_widget.setVisible(checked)
        self._material_details_btn.setText("Détails ▴" if checked else "Détails ▾")

    def _on_robot_details_toggled(self, checked: bool) -> None:
        self._robot_details_widget.setVisible(checked)
        self._robot_details_btn.setText("Détails ▴" if checked else "Détails ▾")

    def _on_material_preset_changed(self, index: int) -> None:
        if self._updating_from_preset:
            return
        if index >= len(MATERIAL_PRESETS):  # "Personnalisé"
            return
        preset = MATERIAL_PRESETS[index]
        self._updating_from_preset = True
        self._ktc_spin.setValue(preset.K_tc)
        self._krc_spin.setValue(preset.K_rc)
        self._kac_spin.setValue(preset.K_ac)
        self._kte_spin.setValue(preset.K_te)
        self._kre_spin.setValue(preset.K_re)
        self._kae_spin.setValue(preset.K_ae)
        self._updating_from_preset = False
        self.params_changed.emit()

    def _on_robot_preset_changed(self, index: int) -> None:
        if self._updating_from_preset:
            return
        if index >= len(ROBOT_PRESETS):  # "Personnalisé"
            return
        preset = ROBOT_PRESETS[index]
        self._updating_from_preset = True
        for i in range(6):
            self._k_spins[i].setValue(preset.stiffness_Nm_per_rad[i])
            self._tau_spins[i].setValue(preset.torque_max_Nm[i])
            self._k_source_labels[i].setText(preset.stiffness_sources[i])
        self._updating_from_preset = False
        self.params_changed.emit()

    def _on_cutting_param_changed(self) -> None:
        self._update_fz_label()
        self.params_changed.emit()

    def _on_k_detail_changed(self) -> None:
        if not self._updating_from_preset:
            self._set_combo_to_custom(self._material_combo, len(MATERIAL_PRESETS))
        self.params_changed.emit()

    def _on_robot_detail_changed(self) -> None:
        if not self._updating_from_preset:
            self._set_combo_to_custom(self._robot_combo, len(ROBOT_PRESETS))
        self.params_changed.emit()

    def _update_fz_label(self) -> None:
        n = self._n_spin.value()
        vf = self._vf_spin.value()
        z = self._z_spin.value()
        if n > 0.0 and z > 0:
            fz = vf / (n * z)
            self._fz_label.setText(f"{fz:.4f} mm/dent")
        else:
            self._fz_label.setText("—")

    def _set_combo_to_custom(self, combo: QComboBox, custom_index: int) -> None:
        """Bascule le combo sur 'Personnalisé' sans déclencher de signal de preset."""
        self._updating_from_preset = True
        combo.setCurrentIndex(custom_index)
        self._updating_from_preset = False

    # ------------------------------------------------------------------
    # API publique
    # ------------------------------------------------------------------

    def get_params(self) -> MachiningSimulationParams:
        """Retourne les paramètres actuels saisis par l'utilisateur."""
        cutting = CuttingParams(
            a_p=self._ap_spin.value(),
            a_e=self._ae_spin.value(),
            diameter=self._diam_spin.value(),
            z_teeth=self._z_spin.value(),
            spindle_speed_rpm=self._n_spin.value(),
            feed_rate_mm_min=self._vf_spin.value(),
            K_tc=self._ktc_spin.value(),
            K_rc=self._krc_spin.value(),
            K_ac=self._kac_spin.value(),
            K_te=self._kte_spin.value(),
            K_re=self._kre_spin.value(),
            K_ae=self._kae_spin.value(),
        )
        mechanical = RobotMechanicalParams(
            joint_stiffness_Nm_per_rad=[s.value() for s in self._k_spins],
            joint_torque_max_Nm=[s.value() for s in self._tau_spins],
        )
        return MachiningSimulationParams(cutting=cutting, mechanical=mechanical)

    def set_params(self, params: MachiningSimulationParams) -> None:
        """Pré-remplit l'interface avec les paramètres fournis."""
        c = params.cutting
        m = params.mechanical

        for spin in self._all_spins:
            spin.blockSignals(True)

        self._ap_spin.setValue(c.a_p)
        self._ae_spin.setValue(c.a_e)
        self._diam_spin.setValue(c.diameter)
        self._z_spin.setValue(c.z_teeth)
        self._n_spin.setValue(c.spindle_speed_rpm)
        self._vf_spin.setValue(c.feed_rate_mm_min)
        self._ktc_spin.setValue(c.K_tc)
        self._krc_spin.setValue(c.K_rc)
        self._kac_spin.setValue(c.K_ac)
        self._kte_spin.setValue(c.K_te)
        self._kre_spin.setValue(c.K_re)
        self._kae_spin.setValue(c.K_ae)
        for i, spin in enumerate(self._k_spins):
            spin.setValue(m.joint_stiffness_Nm_per_rad[i])
        for i, spin in enumerate(self._tau_spins):
            spin.setValue(m.joint_torque_max_Nm[i])

        for spin in self._all_spins:
            spin.blockSignals(False)

        self._update_fz_label()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_double(
        min_val: float, max_val: float, decimals: int,
        step: float, suffix: str, default: float,
    ) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(min_val, max_val)
        spin.setDecimals(decimals)
        spin.setSingleStep(step)
        spin.setSuffix(suffix)
        spin.setValue(default)
        spin.setKeyboardTracking(False)
        return spin

    @staticmethod
    def _make_int(min_val: int, max_val: int, default: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(default)
        spin.setKeyboardTracking(False)
        return spin

    @staticmethod
    def _make_separator() -> QFrame:
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setFrameShadow(QFrame.Shadow.Sunken)
        return sep
