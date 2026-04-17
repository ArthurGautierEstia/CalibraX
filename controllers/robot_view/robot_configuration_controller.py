from PyQt6.QtCore import QObject
from PyQt6.QtCore import pyqtSignal
import os

from models.robot_model import RobotModel
from models.robot_configuration_file import RobotConfigurationFile
from utils.file_io import FileIOHandler
from utils.popup import show_error_popup
from utils.str_utils import str_to_float
from widgets.robot_view.robot_configuration_widget import RobotConfigurationWidget

class RobotConfigurationController(QObject):
    DEFAULT_ROBOT_CONFIG_DIRECTORY = os.path.join("user_data", "configurations")

    configuration_loaded = pyqtSignal()

    def __init__(self, robot_model: RobotModel, robot_configuration_widget: RobotConfigurationWidget, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.robot_configuration_widget = robot_configuration_widget
        self._setup_connections()
    
    def _setup_connections(self) -> None:
        # Signals from Robot Model
        self.robot_model.configuration_changed.connect(self._on_robot_configuration_changed)
        self.robot_model.robot_name_changed.connect(self._on_robot_name_changed)
        self.robot_model.dh_params_changed.connect(self._on_robot_dh_table_changed)
        self.robot_model.measured_dh_params_changed.connect(self._on_robot_measured_dh_table_changed)
        self.robot_model.measured_dh_enabled_changed.connect(self._on_robot_measured_dh_enabled_changed)
        self.robot_model.axis_limits_changed.connect(self._on_robot_axis_config_changed)
        self.robot_model.axis_speed_limits_changed.connect(self._on_robot_axis_config_changed)
        self.robot_model.axis_jerk_limits_changed.connect(self._on_robot_axis_config_changed)
        self.robot_model.axis_reversed_changed.connect(self._on_robot_axis_config_changed)
        self.robot_model.robot_cad_models_changed.connect(self._on_robot_cad_models_changed)
        # Signals from View
        self.robot_configuration_widget.text_changed_requested.connect(self._on_view_name_changed)
        self.robot_configuration_widget.dh_value_changed.connect(self._on_view_dh_value_changed)
        self.robot_configuration_widget.measured_dh_enabled_changed.connect(self._on_view_measured_dh_enabled_changed)
        self.robot_configuration_widget.axis_config_changed.connect(self._on_view_axis_config_changed)
        self.robot_configuration_widget.axis_colliders_config_changed.connect(self._on_view_axis_colliders_config_changed)
        self.robot_configuration_widget.positions_config_changed.connect(self._on_view_positions_config_changed)
        self.robot_configuration_widget.position_zero_requested.connect(self._on_view_position_zero_requested)
        self.robot_configuration_widget.position_transport_requested.connect(self._on_view_position_transport_requested)
        self.robot_configuration_widget.home_position_requested.connect(self._on_view_home_position_requested)
        self.robot_configuration_widget.robot_cad_models_changed.connect(self._on_view_robot_cad_models_changed)
        self.robot_configuration_widget.load_config_requested.connect(self._on_view_load_config_requested)
        self.robot_configuration_widget.export_config_requested.connect(self._on_view_export_config_requested)

    # ======
    # Connection callbacks
    # ======

    def _on_robot_configuration_changed(self) -> None:
        self.update_robot_name_view()
        self.update_dh_table_view()
        self.update_measured_dh_table_view()
        self.update_axis_config_view()
        self.update_axis_colliders_view()
        self.update_positions_config_view()
        self.update_cad_view()

    def _on_robot_name_changed(self) -> None:
        self.update_robot_name_view()

    def _on_robot_dh_table_changed(self) -> None:
        self.update_dh_table_view()

    def _on_robot_measured_dh_table_changed(self) -> None:
        self.update_measured_dh_table_view()

    def _on_robot_measured_dh_enabled_changed(self) -> None:
        self.robot_model.compute_fk_tcp()

    def _on_robot_axis_config_changed(self) -> None:
        self.update_axis_config_view()

    def _on_robot_cad_models_changed(self) -> None:
        self.update_cad_view()

    def _on_view_name_changed(self) -> None:
        self.robot_model.set_robot_name(self.robot_configuration_widget.get_robot_name())

    def _on_view_dh_value_changed(self, row: int, col: int, value: str) -> None:
        fval = str_to_float(value)
        self.robot_model.set_dh_param(row, col, fval)

    def _on_view_measured_dh_enabled_changed(self, enabled: bool) -> None:
        self.robot_model.set_measured_dh_enabled(enabled)
    
    def _on_view_axis_config_changed(
        self,
        axis_limits: list[tuple[float, float]],
        cartesian_slider_limits_xyz: list[tuple[float, float]],
        axis_speed_limits: list[float],
        axis_jerk_limits: list[float],
        axis_reversed: list[int],
    ) -> None:
        self.robot_model.inhibit_auto_compute_fk_tcp(True)
        self.robot_model.set_cartesian_slider_limits_xyz(cartesian_slider_limits_xyz)
        self.robot_model.set_axis_speed_limits(axis_speed_limits)
        self.robot_model.set_axis_jerk_limits(axis_jerk_limits)
        self.robot_model.set_axis_limits(axis_limits)
        self.robot_model.set_axis_reversed(axis_reversed)
        self.robot_model.inhibit_auto_compute_fk_tcp(False)
        self.robot_model.compute_fk_tcp()

    def _on_view_axis_colliders_config_changed(self, axis_colliders: list[dict]) -> None:
        self.robot_model.set_axis_colliders(axis_colliders)

    def _on_view_positions_config_changed(
        self,
        home_position: list[float],
        position_zero: list[float],
        position_transport: list[float],
    ) -> None:
        self.robot_model.set_position_zero(position_zero)
        self.robot_model.set_position_transport(position_transport)
        self.robot_model.set_home_position(home_position)

    def _sync_positions_from_view(self) -> None:
        self._on_view_positions_config_changed(
            self.robot_configuration_widget.get_home_position(),
            self.robot_configuration_widget.get_position_zero(),
            self.robot_configuration_widget.get_position_transport(),
        )

    def _on_view_position_zero_requested(self) -> None:
        self._sync_positions_from_view()
        self.robot_model.go_to_position_zero()

    def _on_view_position_transport_requested(self) -> None:
        self._sync_positions_from_view()
        self.robot_model.go_to_position_transport()

    def _on_view_home_position_requested(self) -> None:
        self._sync_positions_from_view()
        self.robot_model.go_to_home_position()

    def _on_view_robot_cad_models_changed(self, robot_cad_models: list[str]) -> None:
        self.robot_model.set_robot_cad_models(robot_cad_models)

    def _on_view_load_config_requested(self) -> None:
        self.load_configuration()

    def _on_view_export_config_requested(self) -> None:
        self.export_configuration()

    # ======
    # Methods
    # ======

    def update_robot_name_view(self) -> None:
        self.robot_configuration_widget.set_robot_name(self.robot_model.get_robot_name())
    
    def update_dh_table_view(self) -> None:
        self.robot_configuration_widget.set_dh_params(self.robot_model.get_dh_params())

    def update_measured_dh_table_view(self) -> None:
        measured_params = self.robot_model.get_measured_dh_params()
        has_measured_params = self.robot_model.get_measured_dh_enabled() or any(
            any(abs(value) > 1e-9 for value in row) for row in measured_params
        )
        self.robot_configuration_widget.set_measured_dh_params(measured_params)
        self.robot_configuration_widget.measured_dh_toggle.setEnabled(has_measured_params)
        self.robot_configuration_widget.set_measured_dh_table_enabled(self.robot_model.get_measured_dh_enabled())

    def update_axis_config_view(self) -> None:
        self.robot_configuration_widget.set_axis_config(
            self.robot_model.get_axis_limits(),
            self.robot_model.get_cartesian_slider_limits_xyz(),
            self.robot_model.get_axis_speed_limits(),
            self.robot_model.get_axis_jerk_limits(),
            self.robot_model.get_axis_reversed(),
        )

    def update_axis_colliders_view(self) -> None:
        self.robot_configuration_widget.set_axis_colliders(self.robot_model.get_axis_colliders())

    def update_positions_config_view(self) -> None:
        self.robot_configuration_widget.set_positions_config(
            self.robot_model.get_home_position(),
            self.robot_model.get_position_zero(),
            self.robot_model.get_position_transport(),
        )

    def update_cad_view(self) -> None:
        self.robot_configuration_widget.set_robot_cad_models(self.robot_model.get_robot_cad_models())

    def load_configuration(self):
        """Charger une configuration depuis un fichier json"""
        configuration_dir = self._robot_configuration_directory()

        file_path, data = FileIOHandler.select_and_load_json(
            self.robot_configuration_widget,
            "Charger une configuration robot",
            configuration_dir,
        )

        if data:
            if not isinstance(data, dict):
                show_error_popup("Erreur d'importation", "Le fichier de configuration n'est pas au format adapté. Veuillez vérifier le contenu.")
                return
            
            self.load_configuration_from_path(file_path)

    def export_configuration(self):
        """Exporter la configuration actuelle"""
        configuration_dir = self._robot_configuration_directory()

        config = RobotConfigurationFile.from_robot_model(self.robot_model)
        FileIOHandler.save_json(
            self.robot_configuration_widget,
            "Exporter/Sauvegarder une configuration robot",
            config.to_dict(),
            configuration_dir,
        )

    def load_configuration_from_path(self, file_path: str, show_errors: bool = True) -> bool:
        _, data = FileIOHandler.load_json(file_path)
        if not isinstance(data, dict):
            if show_errors:
                show_error_popup(
                    "Erreur d'importation",
                    "Le fichier de configuration n'est pas au format adapte. Veuillez verifier le contenu.",
                )
            return False

        config = RobotConfigurationFile.from_dict(data)
        self.robot_model.load_from_configuration_file(config, file_path)
        self.configuration_loaded.emit()
        return True

    @staticmethod
    def _robot_configuration_directory() -> str:
        root_dir = os.getcwd()
        configuration_dir = os.path.abspath(
            os.path.join(root_dir, RobotConfigurationController.DEFAULT_ROBOT_CONFIG_DIRECTORY)
        )
        os.makedirs(configuration_dir, exist_ok=True)
        return configuration_dir
