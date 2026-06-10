from __future__ import annotations

import os
import re

from PyQt6.QtCore import QObject, QTimer
from PyQt6.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
)

from controllers.calibration_controller import CalibrationController
from controllers.camera_controller import CameraController
from controllers.cartesian_control_controller import CartesianControlController
from controllers.external_axes_controller import ExternalAxesController
from controllers.joint_control_controller import JointControlController
from controllers.machining_controller import MachiningController
from controllers.mgi_controller import MgiController
from controllers.program_controller import ProgramController
from controllers.project_controller import ProjectController
from controllers.robot_controller import RobotController
from controllers.trajectory_controller import TrajectoryController
from controllers.viewer3d_controller import Viewer3DController
from controllers.workspace_controller import WorkspaceController
from controllers.workpiece_controller import WorkpieceController
from models.app_session_file import AppSessionFile, ProgramBaseConfigState, ViewerDisplayState
from models.camera_model import CameraModel
from models.collision_scene_model import CollisionSceneModel
from models.external_axes_model import ExternalAxesModel
from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.tooling_model import ToolingModel
from models.workspace_model import WorkspaceModel
from models.workpiece_model import WorkpieceModel
from utils.status_badge import apply_status_badge
from views.main_window import MainWindow


class MainController(QObject):
    DEFAULT_SESSION_FILE = "app_session.json"

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        camera_model: CameraModel,
        external_axes_model: ExternalAxesModel,
        workpiece_model: WorkpieceModel,
        tooling_model: ToolingModel,
        main_window: MainWindow,
        startup_options: dict | None = None,
        trajectory_benchmark_verbose: bool = False,
        validity_pool_size: int = 1,
        parent: QObject = None,
    ):
        super().__init__(parent)

        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.camera_model = camera_model
        self.external_axes_model = external_axes_model
        self.workpiece_model = workpiece_model
        self.tooling_model = tooling_model
        self.main_window = main_window
        self.startup_options = dict(startup_options or {})
        self.project_root = os.getcwd()
        self.session_path = self._resolve_session_path(
            self.startup_options.get("session") or MainController.DEFAULT_SESSION_FILE
        )

        self._session_save_timer = QTimer(self)
        self._session_save_timer.setSingleShot(True)
        self._session_save_timer.setInterval(250)
        self._session_save_timer.timeout.connect(self.flush_session)
        self._startup_completed = False
        self.robot_model.set_tool(self.tool_model.get_tool())
        self.collision_scene_model = CollisionSceneModel(
            self.robot_model,
            self.tool_model,
            self.workspace_model,
            self,
        )

        self.viewer3d_controller = Viewer3DController(
            robot_model,
            tool_model,
            workspace_model,
            self.collision_scene_model,
            main_window.get_viewer3d(),
            external_axes_model=external_axes_model,
        )
        self.camera_controller = CameraController(
            camera_model,
            main_window.get_camera_view(),
            self.viewer3d_controller,
            parent=self,
        )
        self.robot_controller = RobotController(
            robot_model,
            tool_model,
            main_window.get_robot_view(),
            main_window.get_tool_view(),
            self.viewer3d_controller,
        )
        self.calibration_controller = CalibrationController(robot_model, tool_model, main_window.get_calibration_view())
        self.joint_control_controller = JointControlController(robot_model, main_window.get_joint_control_view())
        self.mgi_controller = MgiController(
            robot_model,
            main_window.get_mgi_solutions_widget(),
        )
        self.cartesian_control_controller = CartesianControlController(
            robot_model,
            tool_model,
            workspace_model,
            main_window.get_cartesian_control_view(),
            self.mgi_controller,
        )
        self.trajectory_controller = TrajectoryController(
            robot_model,
            tool_model,
            workspace_model,
            main_window.get_trajectory_view(),
            self.viewer3d_controller,
            trajectory_benchmark_verbose=trajectory_benchmark_verbose,
            validity_pool_size=validity_pool_size,
        )
        self.workspace_controller = WorkspaceController(
            workspace_model,
            main_window.get_workspace_view(),
            self.viewer3d_controller,
        )
        self.external_axes_controller = ExternalAxesController(
            external_axes_model,
            workspace_model,
            main_window.get_external_axes_view(),
            self.viewer3d_controller,
        )
        self.workpiece_controller = WorkpieceController(
            workpiece_model,
            tooling_model,
            workspace_model,
            external_axes_model,
            main_window.get_workpiece_view(),
            self.viewer3d_controller,
        )
        self.program_controller = ProgramController(
            robot_model,
            tool_model,
            workspace_model,
            external_axes_model,
            self.workpiece_controller,
            main_window.get_program_view(),
            main_window.get_viewer_playback_widget(),
            self.viewer3d_controller,
        )
        self.project_controller = ProjectController(self)
        self.machining_controller = MachiningController(
            robot_model,
            tool_model,
            main_window.get_machining_view(),
            self.program_controller,
        )

        self._on_robot_model_config_changed()
        self._setup_connections()
        self._on_main_tab_changed(self.main_window.tabs.currentIndex())

    def _setup_connections(self) -> None:
        self.main_window.tabs.currentChanged.connect(self._on_main_tab_changed)
        self.main_window.cell_configuration_tabs.currentChanged.connect(self._on_cell_configuration_tab_changed)
        self.robot_model.configuration_changed.connect(self._on_robot_model_config_changed)
        self.robot_model.configuration_changed.connect(self._schedule_session_save)
        self.robot_model.axis_colliders_changed.connect(self._schedule_session_save)
        self.robot_model.measured_dh_params_changed.connect(self._schedule_session_save)
        self.robot_model.measured_dh_enabled_changed.connect(self._schedule_session_save)
        self.robot_controller.configuration_loaded.connect(self._on_config_loaded)
        self.robot_controller.dh_controller.validation_state_changed.connect(self.main_window.set_robot_tab_validated)
        self.robot_controller.tool_controller.validation_state_changed.connect(self.main_window.set_tool_tab_validated)
        self.external_axes_controller.validation_state_changed.connect(self.main_window.set_external_axes_tab_validated)
        self.workspace_controller.validation_state_changed.connect(self.main_window.set_workspace_tab_validated)
        self.workpiece_controller.validation_state_changed.connect(self.main_window.set_workpiece_tab_validated)
        self.camera_controller.validation_state_changed.connect(self.main_window.set_camera_tab_validated)
        self.camera_controller.session_state_changed.connect(self._schedule_session_save)
        self.project_controller.project_changed.connect(self._schedule_session_save)
        self.main_window.verify_configurations_requested.connect(self._on_verify_configurations_requested)
        self.main_window.fit_scene_view_requested.connect(self._on_fit_scene_view_requested)
        self.main_window.manage_viewer_themes_requested.connect(self._on_manage_viewer_themes_requested)
        self.main_window.show_keyboard_shortcuts_requested.connect(self._on_show_keyboard_shortcuts_requested)
        self.main_window.show_about_requested.connect(self._on_show_about_requested)
        self.calibration_controller.apply_measured_dh_requested.connect(self._on_apply_measured_dh_requested)

        self.tool_model.tool_changed.connect(self._on_tool_changed)
        self.tool_model.tool_visual_changed.connect(self._schedule_session_save)
        self.tool_model.tool_profile_changed.connect(self._schedule_session_save)
        self.tool_model.tool_colliders_changed.connect(self._schedule_session_save)
        self.workspace_model.workspace_changed.connect(self._schedule_session_save)
        self.main_window.get_viewer3d().display_state_changed.connect(self._schedule_session_save)
        self.main_window.get_cartesian_control_view().get_cartesian_control_widget().reference_frame_changed.connect(
            self._schedule_session_save
        )
        self.main_window.get_trajectory_view().get_config_widget().cartesianDisplayFrameChanged.connect(
            self._schedule_session_save
        )

    def shutdown(self) -> None:
        self._session_save_timer.stop()
        self.flush_session()
        self.trajectory_controller.shutdown()

    def _on_apply_measured_dh_requested(self) -> None:
        measured_dh = self.calibration_controller.measurement_controller.get_selected_measured_dh_params()
        self.robot_model.set_measured_dh_params(measured_dh)
        self.robot_model.set_measured_dh_enabled(True)
        self.robot_controller.dh_controller.robot_configuration_widget.set_measured_dh_params(measured_dh)
        self.robot_controller.dh_controller.robot_configuration_widget.set_measured_dh_table_enabled(True)
        QMessageBox.information(
            self.main_window,
            "Configuration modifiée",
            "Les paramètres sélectionnés ont été appliqués à la configuration robot.",
        )
        self.robot_controller.dh_controller.robot_configuration_widget.measured_dh_toggle.setEnabled(True)

    def _on_verify_configurations_requested(self) -> None:
        dialog = QDialog(self.main_window)
        dialog.setWindowTitle("Vérification des configurations")
        dialog.resize(820, 340)

        layout = QVBoxLayout(dialog)
        summary_label = QLabel(dialog)
        summary_label.setStyleSheet("font-size: 13px; font-weight: 600;")
        layout.addWidget(summary_label)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        grid.addWidget(QLabel("Configuration", dialog), 0, 0)
        grid.addWidget(QLabel("Statut", dialog), 0, 1)
        grid.addWidget(QLabel("Chemin", dialog), 0, 2)

        rows: dict[str, dict[str, object]] = {}
        for row_index, (key, label, _loading_message) in enumerate(self._configuration_specs(), start=1):
            name_label = QLabel(label, dialog)
            status_label = QLabel(dialog)
            status_label.setMinimumWidth(170)

            path_field = QLineEdit(dialog)
            path_field.setReadOnly(True)
            path_field.setMinimumWidth(360)
            path_field.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

            load_button = QPushButton("...", dialog)
            load_button.setFixedWidth(34)
            load_button.setToolTip(f"Charger une configuration {label.lower()}")
            load_button.clicked.connect(lambda _checked=False, config_key=key: self._load_configuration_from_dialog(dialog, config_key, rows, summary_label))

            path_layout = QHBoxLayout()
            path_layout.setContentsMargins(0, 0, 0, 0)
            path_layout.addWidget(path_field, 1)
            path_layout.addWidget(load_button)

            grid.addWidget(name_label, row_index, 0)
            grid.addWidget(status_label, row_index, 1)
            grid.addLayout(path_layout, row_index, 2)
            rows[key] = {
                "status_label": status_label,
                "path_field": path_field,
            }

        grid.setColumnStretch(2, 1)
        layout.addLayout(grid)

        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close, dialog)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)

        self._refresh_configurations_dialog(rows, summary_label)
        dialog.exec()

    def _on_fit_scene_view_requested(self) -> None:
        viewer = self.main_window.get_viewer3d()
        if hasattr(viewer, "_set_camera_preset"):
            viewer._set_camera_preset("isometric")

    def _on_manage_viewer_themes_requested(self) -> None:
        viewer = self.main_window.get_viewer3d()
        if hasattr(viewer, "open_viewer_theme_settings_dialog"):
            viewer.open_viewer_theme_settings_dialog()

    def _configuration_specs(self) -> tuple[tuple[str, str, str], ...]:
        return (
            ("robot", "Robot", "Chargement configuration robot ..."),
            ("tool", "Tool", "Chargement configuration tool ..."),
            ("external_axes", "Axe externe", "Chargement axes externes ..."),
            ("scene", "Scene", "Chargement configuration scene ..."),
            ("piece", "Pièce", "Chargement configuration pièce ..."),
            ("camera", "Camera", "Chargement cameras ..."),
        )

    def _refresh_configurations_dialog(
        self,
        rows: dict[str, dict[str, object]],
        summary_label: QLabel,
    ) -> None:
        entries = self.get_project_configuration_paths()
        ok_count = 0
        missing_count = 0
        unsaved_count = 0
        modified_count = 0
        not_found_count = 0

        for key, _label, _loading_message in self._configuration_specs():
            path = entries.get(key, "")
            status_text, status_color = self._configuration_status_for_key(key, path)
            if status_color == "#6fcf97":
                ok_count += 1
            elif status_color == "#d9534f":
                not_found_count += 1
            elif self._is_unsaved_configuration_status(status_text):
                unsaved_count += 1
            elif status_color == "#d97706":
                modified_count += 1
            else:
                missing_count += 1

            row = rows.get(key, {})
            status_label = row.get("status_label")
            path_field = row.get("path_field")
            if isinstance(status_label, QLabel):
                apply_status_badge(status_label, status_text, status_color)
            if isinstance(path_field, QLineEdit):
                path_field.setText(path)
                path_field.setPlaceholderText("Aucune configuration")
                path_field.setToolTip(path)

        parts = [f"{ok_count} OK"]
        if missing_count:
            parts.append(f"{missing_count} non chargée(s)")
        if unsaved_count:
            parts.append(f"{unsaved_count} non enregistrée(s)")
        if modified_count:
            parts.append(f"{modified_count} modifiée(s)")
        if not_found_count:
            parts.append(f"{not_found_count} introuvable(s)")
        summary_label.setText("Aperçu des statuts : " + " · ".join(parts))

    @staticmethod
    def _is_unsaved_configuration_status(status_text: str) -> bool:
        normalized = status_text.strip().lower()
        return "non enregistr" in normalized and "modification" not in normalized

    def _configuration_status_for_key(self, config_key: str, path: str) -> tuple[str, str]:
        if str(path or "").strip() and not self._configuration_path_exists(path):
            return "Fichier introuvable", "#d9534f"

        status_label = self._configuration_status_label(config_key)
        if status_label is not None:
            text = status_label.text().strip()
            color = self._color_from_stylesheet(status_label.styleSheet())
            if text:
                return text, color or "#808080"

        if not str(path or "").strip():
            return "Configuration non chargée", "#808080"
        if self._configuration_path_exists(path):
            return "Configuration à jour", "#6fcf97"
        return "Fichier introuvable", "#d9534f"

    def _configuration_status_label(self, config_key: str) -> QLabel | None:
        widgets = {
            "robot": self.robot_controller.dh_controller.robot_configuration_widget,
            "tool": self.robot_controller.tool_controller.robot_configuration_widget,
            "external_axes": self.external_axes_controller._panel,
            "scene": self.workspace_controller.workspace_widget,
            "piece": self.workpiece_controller.view,
            "camera": self.camera_controller.camera_widget,
        }
        widget = widgets.get(config_key)
        status_label = getattr(widget, "status_label", None)
        return status_label if isinstance(status_label, QLabel) else None

    @staticmethod
    def _color_from_stylesheet(stylesheet: str) -> str:
        match = re.search(r"background-color\s*:\s*(#[0-9A-Fa-f]{6})", stylesheet or "")
        if match:
            return match.group(1)
        match = re.search(r"color\s*:\s*(#[0-9A-Fa-f]{6})", stylesheet or "")
        return match.group(1) if match else ""

    def _configuration_path_exists(self, path: str) -> bool:
        normalized = str(path or "").strip()
        if not normalized:
            return False
        resolved = (
            os.path.abspath(normalized)
            if os.path.isabs(normalized)
            else os.path.abspath(os.path.join(self.project_root, normalized))
        )
        return os.path.exists(resolved)

    def _load_configuration_from_dialog(
        self,
        dialog: QDialog,
        config_key: str,
        rows: dict[str, dict[str, object]],
        summary_label: QLabel,
    ) -> None:
        selected_path = self._select_configuration_file(config_key, dialog)
        if not selected_path:
            return

        if self._load_single_configuration(config_key, selected_path):
            self._fit_scene_view_after_loading()
            self._schedule_session_save()
        self._refresh_configurations_dialog(rows, summary_label)

    def _select_configuration_file(self, config_key: str, dialog: QDialog) -> str:
        entries = self.get_project_configuration_paths()
        current_path = self._resolve_existing_path(entries.get(config_key, ""))
        start_dir = os.path.dirname(current_path) if current_path else self.project_root
        selected_path, _filter = QFileDialog.getOpenFileName(
            dialog,
            "Charger une configuration",
            start_dir,
            "Configurations JSON (*.json);;Tous les fichiers (*)",
        )
        return selected_path

    def _load_single_configuration(self, config_key: str, path: str) -> bool:
        loaders = {
            "robot": lambda selected_path: self.robot_controller.dh_controller.load_configuration_from_path(
                selected_path,
                show_errors=True,
            ),
            "tool": lambda selected_path: self.robot_controller.tool_controller.load_tool_profile_from_path(
                selected_path,
                show_errors=True,
            ),
            "external_axes": self.external_axes_controller.load_configuration_from_path,
            "scene": lambda selected_path: self.workspace_controller.load_workspace_from_path(
                selected_path,
                show_errors=True,
            ),
            "piece": lambda selected_path: self.workpiece_controller.load_configuration_from_path(
                selected_path,
                show_errors=True,
            ),
            "camera": lambda selected_path: self.camera_controller.load_configuration_from_path(
                selected_path,
                show_errors=True,
            ),
        }
        loading_messages = {
            key: loading_message
            for key, _label, loading_message in self._configuration_specs()
        }
        loader = loaders.get(config_key)
        if loader is None:
            return False
        return self._load_with_feedback(
            loading_messages.get(config_key, "Chargement configuration ..."),
            lambda: loader(path),
        )

    def _on_show_keyboard_shortcuts_requested(self) -> None:
        QMessageBox.information(
            self.main_window,
            "Raccourcis clavier",
            "Ctrl+N : Nouveau projet\n"
            "Ctrl+O : Charger projet\n"
            "Ctrl+S : Enregistrer projet\n"
            "Ctrl+Shift+S : Enregistrer projet sous\n"
            "Ctrl+0 : Adapter la vue à la scène\n"
            "F11 : Plein écran\n"
            "Ctrl+Q : Quitter",
        )

    def _on_show_about_requested(self) -> None:
        QMessageBox.about(
            self.main_window,
            "À propos de CalibraX",
            "CalibraX\n\n"
            "Configuration, calibration et visualisation de cellule robotisée.",
        )

    def bootstrap_startup(self) -> None:
        session = self._load_session()
        startup = self._build_startup_payload(session)
        self.project_controller.set_recent_projects(startup.get("recent_projects", []))

        viewer_state = startup.get("viewer_state")
        if isinstance(viewer_state, ViewerDisplayState):
            self.main_window.get_viewer3d().apply_display_state(viewer_state)

        project_path = self._resolve_existing_path(startup.get("project", ""))
        project_loaded = bool(project_path and self.project_controller.load_project_from_path(project_path))
        if project_loaded:
            program_base_config = startup.get("program_base_config") or {}
            if program_base_config:
                self.program_controller.load_base_config_state(program_base_config)
            self._fit_scene_view_after_loading()
            self._schedule_session_save()
            return

        robot_configuration_loaded = False
        config_path = self._resolve_existing_path(startup.get("config", ""))
        if config_path:
            self.viewer3d_controller.begin_loading_feedback("Chargement configuration robot ...")
            try:
                robot_configuration_loaded = self.robot_controller.dh_controller.load_configuration_from_path(
                    config_path,
                    show_errors=False,
                )
            finally:
                self.viewer3d_controller.end_loading_feedback()

        tool_path = self._resolve_existing_path(startup.get("tool", ""))
        if not robot_configuration_loaded and tool_path:
            self.viewer3d_controller.begin_loading_feedback("Chargement configuration tool ...")
            try:
                self.robot_controller.tool_controller.load_tool_profile_from_path(tool_path, show_errors=False)
            finally:
                self.viewer3d_controller.end_loading_feedback()

        workspace_path = self._resolve_existing_path(startup.get("workspace", ""))
        if workspace_path:
            self._load_with_feedback(
                "Chargement configuration scene ...",
                lambda: self.workspace_controller.load_workspace_from_path(workspace_path, show_errors=False),
            )

        # Chargement de la configuration des axes externes
        external_axes_path = self._resolve_existing_path(startup.get("external_axes_config", "")) or ""
        if not external_axes_path:
            external_axes_path = self._guess_external_axes_config_from_workspace(startup.get("workspace", ""))
        if not external_axes_path:
            external_axes_data = startup.get("external_axes_data") or {}
            external_axes_path = self._resolve_existing_path(
                self.external_axes_controller.infer_config_file_from_state_data(external_axes_data)
            ) or ""
        if external_axes_path:
            self.viewer3d_controller.begin_loading_feedback("Chargement axes externes ...")
            try:
                self.external_axes_controller.load_configuration_from_path(external_axes_path)
            finally:
                self.viewer3d_controller.end_loading_feedback()

        camera_path = self._resolve_existing_path(startup.get("camera_config", ""))
        if camera_path:
            self.viewer3d_controller.begin_loading_feedback("Chargement cameras ...")
            try:
                self.camera_controller.load_configuration_from_path(camera_path, show_errors=False)
            finally:
                self.viewer3d_controller.end_loading_feedback()

        # Restauration de l'outillage + pièce
        workpiece_config_path = self._guess_workpiece_config_from_workspace(startup.get("workspace", ""))
        if workpiece_config_path:
            self._load_with_feedback(
                "Chargement configuration pièce ...",
                lambda: self.workpiece_controller.load_configuration_from_path(workpiece_config_path, show_errors=False),
            )
        else:
            combined_data = {
                "tooling": startup.get("tooling_data") or {},
                "workpiece": startup.get("workpiece_data") or {},
            }
            if any(combined_data.values()):
                workpiece_source_name = f"Scene : {os.path.basename(workspace_path)}" if workspace_path else "Session"
                self.workpiece_controller.restore_state(
                    combined_data,
                    mark_as_reference=True,
                    display_name=workpiece_source_name,
                )

        # Restauration de la config base programme
        program_base_config = startup.get("program_base_config") or {}
        if program_base_config:
            self.program_controller.load_base_config_state(program_base_config)

        self._startup_completed = True
        self._fit_scene_view_after_loading()
        self._schedule_session_save()

    def flush_session(self) -> None:
        base_cfg = self.program_controller.get_base_config_state()
        session = AppSessionFile(
            project_path=self._normalize_project_path(self.project_controller.get_current_project_file()),
            recent_project_paths=[
                self._normalize_project_path(path)
                for path in self.project_controller.get_recent_projects()
            ],
            application_theme="system",
            robot_config_path=self._normalize_project_path(self.robot_model.get_current_config_file()),
            tool_profile_path=self._session_tool_profile_path(),
            workspace_path=self._normalize_project_path(self.workspace_model.get_workspace_file_path()),
            external_axes_config_path=self._normalize_project_path(
                self.external_axes_controller.get_current_config_file()
            ),
            camera_config_path=self._normalize_project_path(self.camera_model.get_current_file_path()),
            viewer_state=self.main_window.get_viewer3d().get_display_state(),
            external_axes_data={},
            workpiece_data=self.workpiece_controller.get_serializable_state().get("workpiece", {}),
            tooling_data=self.workpiece_controller.get_serializable_state().get("tooling", {}),
            program_base_config=ProgramBaseConfigState.from_dict(base_cfg),
        )

        session_dir = os.path.dirname(self.session_path)
        if session_dir:
            os.makedirs(session_dir, exist_ok=True)

        try:
            session.save(self.session_path)
        except (OSError, ValueError, TypeError) as exc:
            print(f"Impossible d'enregistrer la session dans {self.session_path}: {exc}")

    def _on_main_tab_changed(self, index: int) -> None:
        self._update_camera_visibility_mode()

    def _on_cell_configuration_tab_changed(self, index: int) -> None:
        self._update_camera_visibility_mode()

    def _update_camera_visibility_mode(self) -> None:
        is_cell_configuration = (
            self.main_window.tabs.currentWidget() == self.main_window.cell_configuration_tabs
        )
        is_camera = (
            is_cell_configuration
            and self.main_window.cell_configuration_tabs.currentWidget() == self.main_window.camera_view
        )
        self.main_window.get_viewer3d().set_camera_visibility_active(is_camera)

    def _on_robot_model_config_changed(self) -> None:
        self.main_window.update_enabled_tabs(self.robot_model.get_has_configuration())

    def _on_config_loaded(self) -> None:
        self.main_window.get_viewer3d().load_cad(self.robot_model, self.tool_model)

    def _on_tool_changed(self) -> None:
        self.robot_model.set_tool(self.tool_model.get_tool())
        self._schedule_session_save()

    def _schedule_session_save(self, *_args) -> None:
        if not self._startup_completed:
            return
        self._session_save_timer.start()

    def _load_session(self) -> AppSessionFile | None:
        if not os.path.exists(self.session_path):
            return None
        try:
            return AppSessionFile.load(self.session_path)
        except (OSError, ValueError, TypeError) as exc:
            print(f"Impossible de charger la session {self.session_path}: {exc}")
            return None

    def _build_startup_payload(self, session: AppSessionFile | None) -> dict[str, object]:
        config_override = self.startup_options.get("config") or ""
        tool_override = self.startup_options.get("tool") or ""
        workspace_override = self.startup_options.get("workspace") or ""

        return {
            "project": session.project_path if session is not None else "",
            "recent_projects": session.recent_project_paths if session is not None else [],
            "application_theme": "system",
            "config": config_override or (session.robot_config_path if session is not None else ""),
            "tool": tool_override or (session.tool_profile_path if session is not None else ""),
            "workspace": workspace_override or (session.workspace_path if session is not None else ""),
            "external_axes_config": session.external_axes_config_path if session is not None else "",
            "camera_config": session.camera_config_path if session is not None else "",
            "viewer_state": session.viewer_state if session is not None else None,
            "external_axes_data": session.external_axes_data if session is not None else {},
            "workpiece_data": session.workpiece_data if session is not None else {},
            "tooling_data": session.tooling_data if session is not None else {},
            "program_base_config": session.program_base_config.to_dict() if session is not None else {},
        }

    def _guess_external_axes_config_from_workspace(self, workspace_path: str) -> str:
        resolved_workspace = self._resolve_existing_path(workspace_path)
        if not resolved_workspace:
            return ""
        workspace_name = os.path.splitext(os.path.basename(resolved_workspace))[0]
        if not workspace_name:
            return ""
        candidate = os.path.join("default_data", "external_axes_configs", f"{workspace_name}.json")
        return self._resolve_existing_path(candidate)

    def _guess_workpiece_config_from_workspace(self, workspace_path: str) -> str:
        resolved_workspace = self._resolve_existing_path(workspace_path)
        if not resolved_workspace:
            return ""
        workspace_name = os.path.splitext(os.path.basename(resolved_workspace))[0]
        if not workspace_name:
            return ""
        candidate = os.path.join("default_data", "piece_configs", f"{workspace_name}.json")
        return self._resolve_existing_path(candidate)

    def get_project_configuration_paths(self, base_dir: str = "") -> dict[str, str]:
        paths = {
            "robot": self.robot_model.get_current_config_file(),
            "tool": self.tool_model.get_selected_tool_profile(),
            "external_axes": self.external_axes_controller.get_current_config_file(),
            "scene": self.workspace_model.get_workspace_file_path(),
            "piece": self.workpiece_controller.get_current_config_file(),
            "camera": self.camera_model.get_current_file_path(),
        }
        return {
            key: self._project_relative_path(value, base_dir)
            for key, value in paths.items()
            if str(value or "").strip()
        }

    def load_project_configurations(self, configurations: dict[str, str], base_dir: str = "") -> bool:
        def resolve_config_path(key: str) -> str:
            raw_path = str(configurations.get(key, "") or "").strip()
            if not raw_path:
                return ""
            candidates = []
            if os.path.isabs(raw_path):
                candidates.append(raw_path)
            else:
                if base_dir:
                    candidates.append(os.path.join(base_dir, raw_path))
                candidates.append(os.path.join(self.project_root, raw_path))
            for candidate in candidates:
                resolved = os.path.abspath(candidate)
                if os.path.exists(resolved):
                    return resolved
            print(f"Fichier projet introuvable pour {key}: {raw_path}")
            return ""

        loaders = (
            ("robot", "Chargement configuration robot ...", lambda path: self.robot_controller.dh_controller.load_configuration_from_path(path, show_errors=True)),
            ("tool", "Chargement configuration tool ...", lambda path: self.robot_controller.tool_controller.load_tool_profile_from_path(path, show_errors=True)),
            ("external_axes", "Chargement axes externes ...", lambda path: self.external_axes_controller.load_configuration_from_path(path)),
            ("scene", "Chargement configuration scene ...", lambda path: self.workspace_controller.load_workspace_from_path(path, show_errors=True)),
            ("piece", "Chargement configuration pièce ...", lambda path: self.workpiece_controller.load_configuration_from_path(path, show_errors=True)),
            ("camera", "Chargement cameras ...", lambda path: self.camera_controller.load_configuration_from_path(path, show_errors=True)),
        )
        for key, loading_message, loader in loaders:
            path = resolve_config_path(key)
            if path and not self._load_with_feedback(loading_message, lambda path=path, loader=loader: loader(path)):
                QMessageBox.warning(
                    self.main_window,
                    "Chargement projet",
                    f"Impossible de charger la configuration {key}: {path}",
                )
                return False
        self._fit_scene_view_after_loading()
        self._schedule_session_save()
        return True

    def _load_with_feedback(self, message: str, load_callback) -> bool:
        self.viewer3d_controller.begin_loading_feedback(message)
        try:
            return bool(load_callback())
        finally:
            self.viewer3d_controller.end_loading_feedback()

    def _fit_scene_view_after_loading(self) -> None:
        QTimer.singleShot(0, self._on_fit_scene_view_requested)

    def reset_project_configurations(self) -> None:
        self.robot_controller.dh_controller.new_configuration()
        self.robot_controller.tool_controller.reset_tool_configuration()
        self.external_axes_controller.new_configuration()
        self.workspace_controller.new_configuration()
        self.workpiece_controller.new_configuration()
        self.camera_controller.new_configuration()
        self._schedule_session_save()

    def _project_relative_path(self, path: str | None, base_dir: str = "") -> str:
        normalized = str(path or "").strip()
        if not normalized:
            return ""
        absolute_path = (
            os.path.abspath(normalized)
            if os.path.isabs(normalized)
            else os.path.abspath(os.path.join(self.project_root, normalized))
        )
        if base_dir:
            try:
                return os.path.relpath(absolute_path, os.path.abspath(base_dir)).replace("\\", "/")
            except ValueError:
                return absolute_path
        return self._normalize_project_path(absolute_path)

    def _session_tool_profile_path(self) -> str:
        # The session only persists a standalone tool selection when no robot configuration is active.
        if self.robot_model.get_current_config_file():
            return ""
        return self._normalize_project_path(self.tool_model.get_selected_tool_profile())

    def _resolve_session_path(self, path: str) -> str:
        if os.path.isabs(path):
            return os.path.abspath(path)
        return os.path.abspath(os.path.join(self.project_root, path))

    def _resolve_existing_path(self, path: str) -> str:
        normalized = str(path or "").strip()
        if normalized == "":
            return ""

        resolved = (
            os.path.abspath(normalized)
            if os.path.isabs(normalized)
            else os.path.abspath(os.path.join(self.project_root, normalized))
        )
        if os.path.exists(resolved):
            return resolved

        print(f"Fichier introuvable au démarrage: {resolved}")
        return ""

    def _normalize_project_path(self, path: str | None) -> str:
        if path is None:
            return ""

        normalized = str(path).strip()
        if normalized == "":
            return ""

        absolute_path = (
            os.path.abspath(normalized)
            if os.path.isabs(normalized)
            else os.path.abspath(os.path.join(self.project_root, normalized))
        )
        try:
            relative_path = os.path.relpath(absolute_path, self.project_root)
        except ValueError:
            return absolute_path

        if relative_path.startswith(".."):
            return absolute_path
        return relative_path

