from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QVBoxLayout, QWidget

from utils.mgi import MgiConfigKey
from widgets.cartesian_control_view.mgi_configuration_selector_widget import MgiConfigurationSelectorWidget
from widgets.cartesian_control_view.mgi_joint_weights_widget import MgiJointWeightsWidget


class RobotMgiConfigurationWidget(QWidget):
    allowed_configs_changed = pyqtSignal()
    weights_changed = pyqtSignal(list)

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self.config_selector = MgiConfigurationSelectorWidget({key: True for key in MgiConfigKey})
        self.weights_selector = MgiJointWeightsWidget([1.0] * 6)

        layout = QVBoxLayout(self)
        layout.addWidget(self.config_selector)
        layout.addWidget(self.weights_selector)

        self.config_selector.configurations_changed.connect(self._on_allowed_configs_changed)
        self.weights_selector.weights_changed.connect(self.weights_changed.emit)

    def get_allowed_configs(self) -> set[MgiConfigKey]:
        return self.config_selector.get_allowed_configurations()

    def set_allowed_configurations(self, allowed: set[MgiConfigKey]) -> None:
        self.config_selector.set_allowed_configurations(allowed)

    def set_weights(self, weights: list[float]) -> None:
        self.weights_selector.set_weights(weights)

    def _on_allowed_configs_changed(self, _states: dict[MgiConfigKey, bool]) -> None:
        self.allowed_configs_changed.emit()
