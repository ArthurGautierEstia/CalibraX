from __future__ import annotations

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from models.types.machining_params import CuttingParams, MachiningSimulationParams, RobotMechanicalParams


class MachiningParamsWidget(QWidget):
    """Saisie des paramètres de simulation d'usinage (conditions de coupe + mécanique robot)."""

    params_changed = pyqtSignal()

    def __init__(self, parent: QWidget = None) -> None:
        super().__init__(parent)

        # --- Groupe Matière & outil ---
        self._ap_spin = self._make_double_spin(0.01, 100.0, 2, 0.5, " mm", 2.0)
        self._ae_spin = self._make_double_spin(0.01, 500.0, 2, 1.0, " mm", 5.0)
        self._fz_spin = self._make_double_spin(0.001, 10.0, 3, 0.01, " mm/dent", 0.1)
        self._diam_spin = self._make_double_spin(1.0, 500.0, 2, 5.0, " mm", 20.0)
        self._z_spin = self._make_int_spin(1, 20, 4)
        self._ktc_spin = self._make_double_spin(1.0, 5000.0, 1, 50.0, " MPa", 550.0)
        self._krc_spin = self._make_double_spin(1.0, 5000.0, 1, 50.0, " MPa", 200.0)
        self._kac_spin = self._make_double_spin(1.0, 5000.0, 1, 50.0, " MPa", 150.0)
        self._kte_spin = self._make_double_spin(0.0, 1000.0, 1, 1.0, " N/mm", 0.0)
        self._kre_spin = self._make_double_spin(0.0, 1000.0, 1, 1.0, " N/mm", 0.0)
        self._kae_spin = self._make_double_spin(0.0, 1000.0, 1, 1.0, " N/mm", 0.0)

        # --- Groupe Mécanique robot (raideurs + couples max) ---
        # k_i (N·m/rad) :
        #   k1, k2 identifiés sur KR500 par Jubien et al., ICINCO 2014 (Méthode 2, capteur effort)
        #   k3..k6 : valeurs indicatives — non identifiées, à confirmer expérimentalement
        k_defaults = [6.93e6, 7.81e6, 2.0e6, 1.0e6, 0.6e6, 0.3e6]
        # τ_max_i (N·m) — à confirmer fiche constructeur KR500-3
        tau_defaults = [5000.0, 5000.0, 3000.0, 1500.0, 1000.0, 600.0]

        self._k_spins: list[QDoubleSpinBox] = []
        self._tau_spins: list[QDoubleSpinBox] = []
        for i in range(6):
            self._k_spins.append(
                self._make_double_spin(1.0, 1e8, 0, 1e5, " N·m/rad", k_defaults[i])
            )
            self._tau_spins.append(
                self._make_double_spin(1.0, 50000.0, 0, 100.0, " N·m", tau_defaults[i])
            )

        self._setup_ui()
        self._setup_connections()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _setup_ui(self) -> None:
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)

        row = QHBoxLayout()
        row.addWidget(self._build_cutting_group())
        row.addWidget(self._build_mechanical_group())
        main_layout.addLayout(row)

    def _build_cutting_group(self) -> QGroupBox:
        group = QGroupBox("Conditions de coupe")
        layout = QGridLayout(group)
        layout.setColumnStretch(1, 1)

        rows = [
            ("Profondeur axiale a_p", self._ap_spin),
            ("Engagement radial a_e", self._ae_spin),
            ("Avance par dent f_z", self._fz_spin),
            ("Diamètre outil D", self._diam_spin),
            ("Nombre de dents z", self._z_spin),
            ("K_tc (cisaillement tang.)", self._ktc_spin),
            ("K_rc (cisaillement rad.)", self._krc_spin),
            ("K_ac (cisaillement axial)", self._kac_spin),
            ("K_te (arête tang.)", self._kte_spin),
            ("K_re (arête rad.)", self._kre_spin),
            ("K_ae (arête axiale)", self._kae_spin),
        ]
        for r, (label, widget) in enumerate(rows):
            layout.addWidget(QLabel(label), r, 0)
            layout.addWidget(widget, r, 1)
        return group

    def _build_mechanical_group(self) -> QGroupBox:
        group = QGroupBox("Mécanique robot (KR500-3)")
        layout = QGridLayout(group)
        layout.addWidget(QLabel("Axe"), 0, 0)
        layout.addWidget(QLabel("Raideur k (N·m/rad)"), 0, 1)
        layout.addWidget(QLabel("τ_max (N·m)"), 0, 2)
        for i in range(6):
            layout.addWidget(QLabel(f"J{i + 1}"), i + 1, 0)
            layout.addWidget(self._k_spins[i], i + 1, 1)
            layout.addWidget(self._tau_spins[i], i + 1, 2)
        return group

    # ------------------------------------------------------------------
    # Connexions
    # ------------------------------------------------------------------

    def _setup_connections(self) -> None:
        for spin in [
            self._ap_spin, self._ae_spin, self._fz_spin, self._diam_spin,
            self._ktc_spin, self._krc_spin, self._kac_spin,
            self._kte_spin, self._kre_spin, self._kae_spin,
        ]:
            spin.valueChanged.connect(self.params_changed)
        self._z_spin.valueChanged.connect(self.params_changed)
        for spin in self._k_spins + self._tau_spins:
            spin.valueChanged.connect(self.params_changed)

    # ------------------------------------------------------------------
    # API publique
    # ------------------------------------------------------------------

    def get_params(self) -> MachiningSimulationParams:
        """Retourne les paramètres actuels saisis par l'utilisateur."""
        cutting = CuttingParams(
            a_p=self._ap_spin.value(),
            a_e=self._ae_spin.value(),
            f_z=self._fz_spin.value(),
            diameter=self._diam_spin.value(),
            z_teeth=self._z_spin.value(),
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

        for spin in [
            self._ap_spin, self._ae_spin, self._fz_spin, self._diam_spin,
            self._ktc_spin, self._krc_spin, self._kac_spin,
            self._kte_spin, self._kre_spin, self._kae_spin,
        ] + self._k_spins + self._tau_spins:
            spin.blockSignals(True)

        self._ap_spin.setValue(c.a_p)
        self._ae_spin.setValue(c.a_e)
        self._fz_spin.setValue(c.f_z)
        self._diam_spin.setValue(c.diameter)
        self._z_spin.setValue(c.z_teeth)
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

        for spin in [
            self._ap_spin, self._ae_spin, self._fz_spin, self._diam_spin,
            self._ktc_spin, self._krc_spin, self._kac_spin,
            self._kte_spin, self._kre_spin, self._kae_spin,
        ] + self._k_spins + self._tau_spins:
            spin.blockSignals(False)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _make_double_spin(
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
    def _make_int_spin(min_val: int, max_val: int, default: int) -> QSpinBox:
        spin = QSpinBox()
        spin.setRange(min_val, max_val)
        spin.setValue(default)
        spin.setKeyboardTracking(False)
        return spin
