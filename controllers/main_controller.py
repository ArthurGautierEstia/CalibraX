from __future__ import annotations

import os

from PyQt6.QtCore import QObject, QTimer
from PyQt6.QtGui import QColor, QPalette
from PyQt6.QtWidgets import QApplication, QMessageBox

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
        app = QApplication.instance()
        self._system_palette = QPalette(app.palette()) if app is not None else QPalette()
        self._application_theme = "system"

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
        self.main_window.application_theme_changed.connect(self.apply_application_theme)
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
        entries = self.get_project_configuration_paths()
        expected = (
            ("robot", "Robot"),
            ("tool", "Tool"),
            ("external_axes", "Axe externe"),
            ("scene", "Scene"),
            ("piece", "Pièce"),
            ("camera", "Camera"),
        )
        lines = []
        for key, label in expected:
            path = entries.get(key, "")
            if not path:
                lines.append(f"{label}: aucune configuration")
                continue
            resolved = self._resolve_existing_path(path)
            state = "OK" if resolved else "fichier introuvable"
            lines.append(f"{label}: {state}\n  {path}")

        QMessageBox.information(
            self.main_window,
            "Vérification des configurations",
            "\n\n".join(lines),
        )

    def _on_fit_scene_view_requested(self) -> None:
        viewer = self.main_window.get_viewer3d()
        if hasattr(viewer, "_set_camera_preset"):
            viewer._set_camera_preset("isometric")

    def _on_manage_viewer_themes_requested(self) -> None:
        viewer = self.main_window.get_viewer3d()
        if hasattr(viewer, "open_viewer_theme_settings_dialog"):
            viewer.open_viewer_theme_settings_dialog()

    def apply_application_theme(self, theme_key: str, save_session: bool = True) -> None:
        normalized = str(theme_key or "").strip().lower()
        if normalized not in {"system", "light", "dark"}:
            normalized = "system"
        self._application_theme = normalized

        app = QApplication.instance()
        if app is None:
            return

        if normalized == "dark":
            palette = self._build_dark_application_palette()
        elif normalized == "light":
            palette = self._build_light_application_palette()
        else:
            palette = QPalette(self._system_palette)

        self._apply_accent_to_palette(palette)
        app.setPalette(palette)
        self.main_window.set_application_theme_selection(normalized)
        if save_session:
            self._schedule_session_save()

    def _build_light_application_palette(self) -> QPalette:
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor("#f3f3f3"))
        palette.setColor(QPalette.ColorRole.WindowText, QColor("#1f1f1f"))
        palette.setColor(QPalette.ColorRole.Base, QColor("#ffffff"))
        palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#f0f0f0"))
        palette.setColor(QPalette.ColorRole.ToolTipBase, QColor("#ffffff"))
        palette.setColor(QPalette.ColorRole.ToolTipText, QColor("#1f1f1f"))
        palette.setColor(QPalette.ColorRole.Text, QColor("#1f1f1f"))
        palette.setColor(QPalette.ColorRole.Button, QColor("#f3f3f3"))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor("#1f1f1f"))
        palette.setColor(QPalette.ColorRole.BrightText, QColor("#ffffff"))
        return palette

    def _build_dark_application_palette(self) -> QPalette:
        palette = QPalette()
        palette.setColor(QPalette.ColorRole.Window, QColor("#2b2b2b"))
        palette.setColor(QPalette.ColorRole.WindowText, QColor("#e6e6e6"))
        palette.setColor(QPalette.ColorRole.Base, QColor("#1f1f1f"))
        palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#303030"))
        palette.setColor(QPalette.ColorRole.ToolTipBase, QColor("#2b2b2b"))
        palette.setColor(QPalette.ColorRole.ToolTipText, QColor("#e6e6e6"))
        palette.setColor(QPalette.ColorRole.Text, QColor("#e6e6e6"))
        palette.setColor(QPalette.ColorRole.Button, QColor("#333333"))
        palette.setColor(QPalette.ColorRole.ButtonText, QColor("#e6e6e6"))
        palette.setColor(QPalette.ColorRole.BrightText, QColor("#ffffff"))
        return palette

    def _apply_accent_to_palette(self, palette: QPalette) -> None:
        accent = QColor("#FF6F00")
        palette.setColor(QPalette.ColorRole.Highlight, accent)
        palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#FFFFFF"))
        palette.setColor(QPalette.ColorRole.Link, accent)
        palette.setColor(QPalette.ColorRole.LinkVisited, QColor("#D65E00"))
        if hasattr(QPalette.ColorRole, "Accent"):
            palette.setColor(QPalette.ColorRole.Accent, accent)

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
        self.apply_application_theme(str(startup.get("application_theme", "system") or "system"), save_session=False)

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

        # Restauration des réglages de génération
        generation_settings_data = startup.get("program_generation_settings") or {}
        if generation_settings_data:
            self.program_controller.load_generation_settings_dict(generation_settings_data)
        from models.krl_header_file import load_header_template
        loaded_header = load_header_template()
        self.program_controller._generation_settings.header_text = loaded_header
        self.program_controller.generation_widget.set_settings(
            self.program_controller._generation_settings
        )
        self.program_controller.generation_widget.set_header_text(loaded_header)

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
            application_theme=self._application_theme,
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
            program_generation_settings=self.program_controller.get_generation_settings(),
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
            "application_theme": session.application_theme if session is not None else "system",
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
            "program_generation_settings": session.program_generation_settings.to_dict() if session is not None else {},
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

