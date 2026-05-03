from PyQt6.QtCore import QObject

from models.robot_model import RobotModel
from widgets.robot_view.robot_mgi_configuration_widget import RobotMgiConfigurationWidget


class RobotMgiConfigurationController(QObject):
    def __init__(
        self,
        robot_model: RobotModel,
        robot_mgi_configuration_widget: RobotMgiConfigurationWidget,
        parent: QObject | None = None,
    ):
        super().__init__(parent)
        self.robot_model = robot_model
        self.robot_mgi_configuration_widget = robot_mgi_configuration_widget

        self._setup_connections()
        self._refresh_view()

    def _setup_connections(self) -> None:
        self.robot_mgi_configuration_widget.allowed_configs_changed.connect(self._on_view_allowed_configs_changed)
        self.robot_mgi_configuration_widget.weights_changed.connect(self._on_view_weights_changed)

        self.robot_model.allowed_config_changed.connect(self._on_model_allowed_configs_changed)
        self.robot_model.joint_weights_changed.connect(self._on_model_joint_weights_changed)
        self.robot_model.configuration_changed.connect(self._refresh_view)

    def _on_view_allowed_configs_changed(self) -> None:
        self.robot_model.set_allowed_configurations(self.robot_mgi_configuration_widget.get_allowed_configs())

    def _on_view_weights_changed(self, weights: list[float]) -> None:
        self.robot_model.set_joint_weights(weights)

    def _on_model_allowed_configs_changed(self) -> None:
        self.robot_mgi_configuration_widget.set_allowed_configurations(self.robot_model.get_allowed_configurations())

    def _on_model_joint_weights_changed(self) -> None:
        self.robot_mgi_configuration_widget.set_weights(self.robot_model.get_joint_weights())

    def _refresh_view(self) -> None:
        self._on_model_allowed_configs_changed()
        self._on_model_joint_weights_changed()
