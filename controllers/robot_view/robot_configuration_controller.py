from PyQt6.QtCore import QObject
from PyQt6.QtCore import pyqtSignal
import json
import os

from models.primitive_collider_models import AxisDirection
from models.primitive_collider_models import RobotAxisColliderData
from models.robot_model import RobotModel
from models.robot_configuration_file import RobotConfigurationFile
from models.types import XYZ3
from utils.file_io import FileIOHandler
from utils.popup import show_error_popup
from widgets.robot_view.robot_configuration_widget import RobotConfigurationWidget


class RobotConfigurationController(QObject):
    DEFAULT_ROBOT_CONFIG_DIRECTORY = os.path.join("user_data", "configurations")
    STATUS_UNSAVED = "Configuration non enregistrée"
    STATUS_MODIFIED = "Modifications non enregistrées"
    STATUS_SAVED = "Configuration enregistrée"
    STATUS_LOADED = "Configuration chargée"
    STATUS_UP_TO_DATE = "Configuration à jour"

    configuration_loaded = pyqtSignal()

    def __init__(self, robot_model: RobotModel, robot_configuration_widget: RobotConfigurationWidget, parent: QObject = None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.robot_configuration_widget = robot_configuration_widget
        self._saved_snapshot: str | None = None
        self._has_saved_reference = False
        self._clean_status_text = RobotConfigurationController.STATUS_UNSAVED
        self._was_dirty_since_reference = False
        self._setup_connections()
        self._on_robot_configuration_changed()
        self._mark_as_unsaved_reference()

    def _setup_connections(self) -> None:
        self.robot_model.configuration_changed.connect(self._on_robot_configuration_changed)
        self.robot_model.robot_name_changed.connect(self._on_robot_name_changed)
        self.robot_model.dh_params_changed.connect(self._on_robot_dh_table_changed)
        self.robot_model.measured_dh_params_changed.connect(self._on_robot_measured_dh_table_changed)
        self.robot_model.measured_dh_enabled_changed.connect(self._on_robot_measured_dh_enabled_changed)
        self.robot_model.axis_limits_changed.connect(self._on_robot_axis_config_changed)
        self.robot_model.axis_speed_limits_changed.connect(self._on_robot_axis_config_changed)
        self.robot_model.axis_accel_limits_changed.connect(self._on_robot_axis_config_changed)
        self.robot_model.axis_jerk_limits_changed.connect(self._on_robot_axis_config_changed)
        self.robot_model.axis_reversed_changed.connect(self._on_robot_axis_config_changed)
        self.robot_model.allowed_config_changed.connect(self._on_robot_persistence_state_changed)
        self.robot_model.robot_cad_models_changed.connect(self._on_robot_cad_models_changed)
        self.robot_model.cartesian_slider_limits_changed.connect(self._on_robot_persistence_state_changed)
        self.robot_model.axis_colliders_changed.connect(self._on_robot_persistence_state_changed)
        self.robot_model.joint_weights_changed.connect(self._on_robot_persistence_state_changed)
        self.robot_model.corrections_changed.connect(self._on_robot_persistence_state_changed)

        self.robot_configuration_widget.text_changed_requested.connect(self._on_view_name_changed)
        self.robot_configuration_widget.dh_value_changed.connect(self._on_view_dh_value_changed)
        self.robot_configuration_widget.measured_dh_enabled_changed.connect(self._on_view_measured_dh_enabled_changed)
        self.robot_configuration_widget.axis_config_changed.connect(self._on_view_axis_config_changed)
        self.robot_configuration_widget.axis_colliders_config_changed.connect(self._on_view_axis_colliders_config_changed)
        self.robot_configuration_widget.positions_config_changed.connect(self._on_view_positions_config_changed)
        self.robot_configuration_widget.go_to_position_requested.connect(self._on_view_go_to_position_requested)
        self.robot_configuration_widget.robot_cad_models_changed.connect(self._on_view_robot_cad_models_changed)
        self.robot_configuration_widget.new_config_requested.connect(self._on_view_new_config_requested)
        self.robot_configuration_widget.load_config_requested.connect(self._on_view_load_config_requested)
        self.robot_configuration_widget.export_config_requested.connect(self._on_view_export_config_requested)
        self.robot_configuration_widget.save_as_config_requested.connect(self._on_view_save_as_config_requested)

    def _on_robot_configuration_changed(self) -> None:
        self.update_current_configuration_view()
        self.update_robot_name_view()
        self.update_dh_table_view()
        self.update_measured_dh_table_view()
        self.update_axis_config_view()
        self.update_axis_colliders_view()
        self.update_positions_config_view()
        self.update_cad_view()
        self._refresh_configuration_status()

    def _on_robot_name_changed(self) -> None:
        self.update_robot_name_view()
        self._refresh_configuration_status()

    def _on_robot_dh_table_changed(self) -> None:
        self.update_dh_table_view()
        self._refresh_configuration_status()

    def _on_robot_measured_dh_table_changed(self) -> None:
        self.update_measured_dh_table_view()
        self._refresh_configuration_status()

    def _on_robot_measured_dh_enabled_changed(self) -> None:
        self.robot_model.compute_fk_tcp()
        self._refresh_configuration_status()

    def _on_robot_axis_config_changed(self) -> None:
        self.update_axis_config_view()
        self._refresh_configuration_status()

    def _on_robot_cad_models_changed(self) -> None:
        self.update_cad_view()
        self._refresh_configuration_status()

    def _on_robot_persistence_state_changed(self) -> None:
        self._refresh_configuration_status()

    def _on_view_name_changed(self) -> None:
        self.robot_model.set_robot_name(self.robot_configuration_widget.get_robot_name())

    def _on_view_dh_value_changed(self, row: int, col: int, value: float) -> None:
        self.robot_model.set_dh_param(row, col, float(value))

    def _on_view_measured_dh_enabled_changed(self, enabled: bool) -> None:
        self.robot_model.set_measured_dh_enabled(enabled)

    def _on_view_axis_config_changed(
        self,
        axis_limits: list[tuple[float, float]],
        cartesian_slider_limits_xyz: list[tuple[float, float]],
        axis_speed_limits: list[float],
        axis_accel_limits: list[float],
        axis_jerk_limits: list[float],
        axis_reversed: list[int],
    ) -> None:
        self.robot_model.inhibit_auto_compute_fk_tcp(True)
        self.robot_model.set_cartesian_slider_limits_xyz(cartesian_slider_limits_xyz)
        self.robot_model.set_axis_speed_limits(axis_speed_limits)
        self.robot_model.set_axis_jerk_limits(axis_jerk_limits)
        self.robot_model.set_axis_accel_limits(axis_accel_limits)
        self.robot_model.set_axis_limits(axis_limits)
        self.robot_model.set_axis_reversed(axis_reversed)
        self.robot_model.inhibit_auto_compute_fk_tcp(False)
        self.robot_model.compute_fk_tcp()

    def _on_view_axis_colliders_config_changed(self, axis_colliders: list[RobotAxisColliderData]) -> None:
        self.robot_model.set_axis_colliders(axis_colliders)

    def _on_view_positions_config_changed(
        self,
        home_position: list[float],
        position_zero: list[float],
        position_calibration: list[float],
    ) -> None:
        self.robot_model.set_position_zero(position_zero)
        self.robot_model.set_position_calibration(position_calibration)
        self.robot_model.set_home_position(home_position)
        self._refresh_configuration_status()

    def _on_view_go_to_position_requested(self, joint_values: list[float]) -> None:
        if len(joint_values) < 6:
            return
        self.robot_model.set_joints([float(value) for value in joint_values[:6]])

    def _on_view_robot_cad_models_changed(self, robot_cad_models: list[str]) -> None:
        self.robot_model.set_robot_cad_models(robot_cad_models)

    def _on_view_load_config_requested(self) -> None:
        self.load_configuration()

    def _on_view_new_config_requested(self) -> None:
        self.new_configuration()

    def _on_view_export_config_requested(self) -> None:
        self.save_configuration()

    def _on_view_save_as_config_requested(self) -> None:
        self.save_configuration_as()

    def update_robot_name_view(self) -> None:
        self.robot_configuration_widget.set_robot_name(self.robot_model.get_robot_name())

    def update_current_configuration_view(self) -> None:
        current_config_file = self.robot_model.get_current_config_file()
        if not current_config_file:
            self.robot_configuration_widget.set_current_configuration_name("Aucune configuration")
            return
        self.robot_configuration_widget.set_current_configuration_name(os.path.basename(current_config_file))

    def update_dh_table_view(self) -> None:
        self.robot_configuration_widget.set_dh_params(self.robot_model.get_dh_params())

    def update_measured_dh_table_view(self) -> None:
        measured_params = self.robot_model.get_measured_dh_params()
        has_measured_params = self.robot_model.get_measured_dh_enabled() or any(
            any(abs(value) > 1e-9 for value in row) for row in measured_params
        )
        display_params = measured_params if has_measured_params else [[0.0, 0.0, 0.0, 0.0] for _ in range(6)]
        self.robot_configuration_widget.set_measured_dh_params(display_params)
        self.robot_configuration_widget.measured_dh_toggle.setEnabled(has_measured_params)
        self.robot_configuration_widget.set_measured_dh_table_enabled(self.robot_model.get_measured_dh_enabled())

    def update_axis_config_view(self) -> None:
        self.robot_configuration_widget.set_axis_config(
            self.robot_model.get_axis_limits(),
            self.robot_model.get_cartesian_slider_limits_xyz(),
            self.robot_model.get_axis_speed_limits(),
            self.robot_model.get_axis_accel_limits(),
            self.robot_model.get_axis_jerk_limits(),
            self.robot_model.get_axis_reversed(),
        )

    def update_axis_colliders_view(self) -> None:
        self.robot_configuration_widget.set_axis_colliders(self.robot_model.get_axis_collider_data())

    def update_positions_config_view(self) -> None:
        self.robot_configuration_widget.set_positions_config(
            self.robot_model.get_home_position(),
            self.robot_model.get_position_zero(),
            self.robot_model.get_position_calibration(),
        )

    def update_cad_view(self) -> None:
        self.robot_configuration_widget.set_robot_cad_models(self.robot_model.get_robot_cad_models())

    @staticmethod
    def _normalize_snapshot_value(value: object) -> object:
        if isinstance(value, float):
            return round(value, 9)
        if isinstance(value, list):
            return [RobotConfigurationController._normalize_snapshot_value(item) for item in value]
        if isinstance(value, dict):
            return {
                str(key): RobotConfigurationController._normalize_snapshot_value(item)
                for key, item in value.items()
            }
        return value

    def _capture_current_snapshot(self) -> str:
        config_dict = RobotConfigurationFile.from_robot_model(self.robot_model).to_dict()
        normalized_config_dict = RobotConfigurationController._normalize_snapshot_value(config_dict)
        return json.dumps(normalized_config_dict, sort_keys=True, ensure_ascii=True)

    def _mark_as_saved_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = True
        self._clean_status_text = RobotConfigurationController.STATUS_SAVED
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _mark_as_loaded_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = True
        self._clean_status_text = RobotConfigurationController.STATUS_LOADED
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _mark_as_unsaved_reference(self) -> None:
        self._saved_snapshot = self._capture_current_snapshot()
        self._has_saved_reference = False
        self._clean_status_text = RobotConfigurationController.STATUS_UNSAVED
        self._was_dirty_since_reference = False
        self._refresh_configuration_status()

    def _is_dirty(self) -> bool:
        current_snapshot = self._capture_current_snapshot()
        return self._saved_snapshot is None or current_snapshot != self._saved_snapshot

    def _refresh_configuration_status(self) -> None:
        if not self._has_saved_reference:
            self.robot_configuration_widget.set_configuration_status(
                RobotConfigurationController.STATUS_UNSAVED,
                "#808080",
            )
            return
        is_dirty = self._is_dirty()
        if is_dirty:
            self._was_dirty_since_reference = True
            self.robot_configuration_widget.set_configuration_status(
                RobotConfigurationController.STATUS_MODIFIED,
                "#d97706",
            )
            return
        if self._was_dirty_since_reference:
            self.robot_configuration_widget.set_configuration_status(
                RobotConfigurationController.STATUS_UP_TO_DATE,
                "#15803d",
            )
            return
        self.robot_configuration_widget.set_configuration_status(
            self._clean_status_text,
            "#15803d",
        )

    def load_configuration(self) -> None:
        configuration_dir = self._robot_configuration_directory()
        file_path, data = FileIOHandler.select_and_load_json(
            self.robot_configuration_widget,
            "Charger une configuration robot",
            configuration_dir,
        )
        if data:
            if not isinstance(data, dict):
                show_error_popup(
                    "Erreur d'importation",
                    "Le fichier de configuration n'est pas au format adapte. Veuillez verifier le contenu.",
                )
                return
            self.load_configuration_from_path(file_path)

    def save_configuration(self) -> None:
        current_path = self.robot_model.get_current_config_file()
        if not current_path:
            self.save_configuration_as()
            return

        config = RobotConfigurationFile.from_robot_model(self.robot_model)
        try:
            FileIOHandler.write_json(current_path, config.to_dict())
            self._mark_as_saved_reference()
        except OSError as exc:
            show_error_popup(
                "Erreur d'enregistrement",
                f"Impossible d'enregistrer la configuration:\n{exc}",
            )

    def save_configuration_as(self) -> None:
        configuration_dir = self._robot_configuration_directory()
        config = RobotConfigurationFile.from_robot_model(self.robot_model)
        file_path = FileIOHandler.save_json(
            self.robot_configuration_widget,
            "Enregistrer une configuration robot",
            config.to_dict(),
            configuration_dir,
        )
        if file_path:
            self.robot_model.current_config_file = file_path
            self.robot_model.has_configuration = True
            self.robot_model.configuration_changed.emit()
            self._mark_as_saved_reference()

    def new_configuration(self) -> None:
        self.robot_model.reset_to_unconfigured_state()
        self._mark_as_unsaved_reference()

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
        self._mark_as_loaded_reference()
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
