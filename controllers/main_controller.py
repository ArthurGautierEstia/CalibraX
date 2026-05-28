from __future__ import annotations

import os

from PyQt6.QtCore import QObject, QTimer
from PyQt6.QtWidgets import QMessageBox

from controllers.calibration_controller import CalibrationController
from controllers.cartesian_control_controller import CartesianControlController
from controllers.external_axes_controller import ExternalAxesController
from controllers.joint_control_controller import JointControlController
from controllers.machining_controller import MachiningController
from controllers.mgi_controller import MgiController
from controllers.program_controller import ProgramController
from controllers.robot_controller import RobotController
from controllers.trajectory_controller import TrajectoryController
from controllers.viewer3d_controller import Viewer3DController
from controllers.workspace_controller import WorkspaceController
from controllers.workpiece_controller import WorkpieceController
from models.app_session_file import AppSessionFile, ProgramBaseConfigState, ViewerDisplayState
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
            main_window.get_mgi_view(),
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
        self.workspace_controller = WorkspaceController(workspace_model, main_window.get_workspace_view())
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
            self.viewer3d_controller,
        )
        self.machining_controller = MachiningController(
            robot_model,
            tool_model,
            main_window.get_machining_view(),
            self.program_controller,
        )

        self._on_robot_model_config_changed()
        self._setup_connections()

    def _setup_connections(self) -> None:
        self.robot_model.configuration_changed.connect(self._on_robot_model_config_changed)
        self.robot_model.configuration_changed.connect(self._schedule_session_save)
        self.robot_model.axis_colliders_changed.connect(self._schedule_session_save)
        self.robot_model.measured_dh_params_changed.connect(self._schedule_session_save)
        self.robot_model.measured_dh_enabled_changed.connect(self._schedule_session_save)
        self.robot_controller.configuration_loaded.connect(self._on_config_loaded)
        self.robot_controller.dh_controller.validation_state_changed.connect(self.main_window.set_robot_tab_validated)
        self.robot_controller.tool_controller.validation_state_changed.connect(self.main_window.set_tool_tab_validated)
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

    def bootstrap_startup(self) -> None:
        session = self._load_session()
        startup = self._build_startup_payload(session)

        viewer_state = startup.get("viewer_state")
        if isinstance(viewer_state, ViewerDisplayState):
            self.main_window.get_viewer3d().apply_display_state(viewer_state)

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
            self.workspace_controller.load_workspace_from_path(workspace_path, show_errors=False)

        # Restauration des axes externes
        external_axes_data = startup.get("external_axes_data") or {}
        if external_axes_data:
            self.viewer3d_controller.begin_loading_feedback("Chargement axes externes ...")
            try:
                self.external_axes_controller.restore_state(external_axes_data)
            finally:
                self.viewer3d_controller.end_loading_feedback()

        # Restauration de l'outillage + pièce
        combined_data = {
            "tooling": startup.get("tooling_data") or {},
            "workpiece": startup.get("workpiece_data") or {},
        }
        if any(combined_data.values()):
            self.workpiece_controller.restore_state(combined_data)

        # Restauration de la config base programme
        program_base_config = startup.get("program_base_config") or {}
        if program_base_config:
            self.program_controller.load_base_config_state(program_base_config)

        self._startup_completed = True
        self._schedule_session_save()

    def flush_session(self) -> None:
        base_cfg = self.program_controller.get_base_config_state()
        session = AppSessionFile(
            robot_config_path=self._normalize_project_path(self.robot_model.get_current_config_file()),
            tool_profile_path=self._session_tool_profile_path(),
            workspace_path=self._normalize_project_path(self.workspace_model.get_workspace_file_path()),
            viewer_state=self.main_window.get_viewer3d().get_display_state(),
            external_axes_data=self.external_axes_controller.get_serializable_state(),
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
            "config": config_override or (session.robot_config_path if session is not None else ""),
            "tool": tool_override or (session.tool_profile_path if session is not None else ""),
            "workspace": workspace_override or (session.workspace_path if session is not None else ""),
            "viewer_state": session.viewer_state if session is not None else None,
            "external_axes_data": session.external_axes_data if session is not None else {},
            "workpiece_data": session.workpiece_data if session is not None else {},
            "tooling_data": session.tooling_data if session is not None else {},
            "program_base_config": session.program_base_config.to_dict() if session is not None else {},
        }

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

