from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QCheckBox, QLabel
)
from PyQt6.QtCore import pyqtSignal

from mgi import MgiConfigKey


class MgiConfigurationSelectorWidget(QWidget):
    """
    Widget permettant de sélectionner les configurations MGI autorisées
    """

    # Émis à chaque modification
    # dict[MgiConfigKey, bool]
    configurations_changed = pyqtSignal(dict)

    def __init__(self, config_states: dict[MgiConfigKey, bool], parent=None):
        super().__init__(parent)

        self._config_states: dict[MgiConfigKey, bool] = dict(config_states)
        self._checkboxes: dict[MgiConfigKey, QCheckBox] = {}

        self._init_ui()

    # ---------------------------------------------------------
    # UI
    # ---------------------------------------------------------

    def _init_ui(self):
        layout = QVBoxLayout(self)

        title = QLabel("Configurations MGI autorisées")
        title.setStyleSheet("font-weight: bold;")
        layout.addWidget(title)

        for key in MgiConfigKey:
            cb = QCheckBox(self._config_label(key))
            cb.setChecked(self._config_states.get(key, False))
            cb.stateChanged.connect(
                lambda state, k=key: self._on_checkbox_changed(k, state)
            )

            self._checkboxes[key] = cb
            layout.addWidget(cb)

        layout.addStretch()

    # ---------------------------------------------------------
    # Internals
    # ---------------------------------------------------------

    def _on_checkbox_changed(self, key: MgiConfigKey, state: int):
        checked = state == 2  # Qt Checked
        self._config_states[key] = checked
        self.configurations_changed.emit(dict(self._config_states))

    @staticmethod
    def _config_label(key: MgiConfigKey) -> str:
        """
        Label lisible pour l'utilisateur
        """
        return f"{key.name}  ({MgiConfigurationSelectorWidget._config_hint(key)})"

    @staticmethod
    def _config_hint(key: MgiConfigKey) -> str:
        hints = {
            MgiConfigKey.FUN: "Front / Up / No Flip",
            MgiConfigKey.FUF: "Front / Up / Flip",
            MgiConfigKey.FDN: "Front / Down / No Flip",
            MgiConfigKey.FDF: "Front / Down / Flip",
            MgiConfigKey.BUN: "Back / Up / No Flip",
            MgiConfigKey.BUF: "Back / Up / Flip",
            MgiConfigKey.BDN: "Back / Down / No Flip",
            MgiConfigKey.BDF: "Back / Down / Flip",
        }
        return hints.get(key, "")
    
    # ---------------------------------------------------------
    # Public API
    # ---------------------------------------------------------

    def get_states(self) -> dict[MgiConfigKey, bool]:
        """Récupère l'état de toutes les configurations"""
        return dict(self._config_states)

    def get_allowed_configurations(self) -> set[MgiConfigKey]:
        """Récupère uniquement les configurations autorisées"""
        return {k for k, v in self._config_states.items() if v}

    def set_states(self, states: dict[MgiConfigKey, bool]) -> None:
        """Définit l'état de toutes les configurations (mise à jour externe)"""
        self._config_states = dict(states)
        for key, cb in self._checkboxes.items():
            cb.blockSignals(True)
            cb.setChecked(self._config_states.get(key, False))
            cb.blockSignals(False)
    
    def set_allowed_configurations(self, allowed: set[MgiConfigKey]) -> None:
        """Définit les configurations autorisées (mise à jour externe)"""
        states = {key: key in allowed for key in MgiConfigKey}
        self.set_states(states)
