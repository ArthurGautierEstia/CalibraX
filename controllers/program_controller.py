from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass, replace
from pathlib import Path
import time

import numpy as np
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QFileDialog, QMessageBox

from controllers.viewer3d_controller import Viewer3DController
from models.reference_frame import ReferenceFrame
from models.robot_program import (
    ProgramCompensationOutputMode,
    ProgramSimulationSample,
    ProgramSimulationResult,
    RobotProgram,
    RobotProgramMotion,
    RobotProgramTarget,
    RobotProgramTargetType,
    RobotProgramMotionMode,
)

from models.external_axes_model import ExternalAxesModel
from models.program_generation_settings import ProgramGenerationSettings
from models.robot_model import RobotModel
from models.robot_program import MotionRole, ProgramBaseSource, ProgramOrigin
from models.tool_model import ToolModel
from models.types.approach_retract import ApproachAxisRef
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.types import JointAngles6, Pose6
from models.workspace_model import WorkspaceModel
from utils.reference_frame_utils import matrix_to_pose, pose_to_matrix
from utils.program_simulator import ProgramSimulator
from utils.mgi import RobotTool
from utils.aptsource_parser import load_aptsource_program
from utils.catnc_parser import load_catnc_program
from utils.robot_program_kuka import export_kuka_src_program, generate_program_to_path, load_kuka_src_program
from widgets.program_view.program_base_dialog import ProgramBaseDialog
from utils.trajectory_keypoint_utils import resolve_keypoint_xyz
from widgets.program_view.program_target_dialog import ProgramTargetDialog
from widgets.program_view.program_keypoints_widget import ProgramKeypointsWidget
from widgets.program_view.program_playback_widget import ProgramPlaybackWidget
from widgets.program_view.program_tool_orientation_dialog import ProgramToolOrientationDialog
from views.program_view import ProgramView





@dataclass(frozen=True)

class _ProgramTargetRef:

    motion_index: int

    is_via_target: bool





class ProgramController:
    STATUS_NONE = "Aucun programme chargé"
    STATUS_LOADED = "Programme chargé"
    STATUS_SAVED = "Programme enregistré"
    STATUS_MODIFIED = "Programme modifié"

    DEFAULT_PROGRAMS_DIR = Path(__file__).resolve().parents[1] / "user_data" / "programs"



    MEASURED_COLOR: tuple[float, float, float, float] = (0.0, 0.35, 1.0, 1.0)
    COMPENSATED_COLOR: tuple[float, float, float, float] = (0.0, 0.85, 0.35, 1.0)

    _TRAJ_REFRESH_INTERVAL_S: float = 1.0 / 30.0



    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        external_axes_model: ExternalAxesModel,
        workpiece_controller,
        program_view: ProgramView,
        playback_widget: ProgramPlaybackWidget,
        viewer3d_controller: Viewer3DController,
    ) -> None:

        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.external_axes_model = external_axes_model
        self.workpiece_controller = workpiece_controller
        self.program_view = program_view
        self.playback_widget = playback_widget
        self.viewer3d_controller = viewer3d_controller
        self.header_widget = self.program_view.get_header_widget()
        self.config_widget: ProgramKeypointsWidget = self.program_view.get_config_widget()
        self.actions_widget = self.program_view.get_actions_widget()
        self.graphs_widget = self.program_view.get_graphs_widget()
        self.playback_widgets: list[ProgramPlaybackWidget] = [self.playback_widget]
        self.program_simulator = ProgramSimulator(self.robot_model, self.tool_model)
        self.current_program: RobotProgram | None = None
        self.current_result: ProgramSimulationResult | None = None

        self._nominal_cartesian_result: ProgramSimulationResult | None = None
        self._nominal_articular_result: ProgramSimulationResult | None = None
        self._compensated_cartesian_result: ProgramSimulationResult | None = None
        self._compensated_articular_result: ProgramSimulationResult | None = None

        self._articular_program: RobotProgram | None = None
        self._cartesian_program: RobotProgram | None = None
        self._compensated_cartesian_program: RobotProgram | None = None
        self._compensated_articular_program: RobotProgram | None = None

        self._compensation_computed: bool = False
        self._display_keypoints: list[TrajectoryKeypoint] = []
        self._display_keypoint_tools: list[RobotTool] = []
        self._display_target_refs: list[_ProgramTargetRef] = []
        self._selected_keypoint_index: int | None = None
        self._base_source: ProgramBaseSource = ProgramBaseSource.MANUAL
        self._base_offset: Pose6 = Pose6.zeros()
        self._manual_base: Pose6 = Pose6.zeros()
        self._tool_source: str = "CURRENT"
        self._saved_robot_tool: RobotTool | None = None  # Sauvegarde du tool robot original
        self._nominal_segments_cache: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        self._measured_segments_cache: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        self._compensated_segments_cache: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        # Géométrie nominale pré-construite une fois (étape 2 perf)
        self._nom_seg_pts: list[np.ndarray] = []   # pts robot-frame par segment, (K,3)
        self._nom_seg_times: list[np.ndarray] = [] # temps par vertex par segment, (K,)
        self._current_time_s = 0.0
        self._last_split_sample_index: int = -1
        self._last_traj_refresh_wall_s: float = 0.0
        self._playback_sample_times: list[float] = []
        self._playback_timer = QTimer()
        self._playback_timer.setSingleShot(False)
        self._playback_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._playback_timer.timeout.connect(self._on_playback_tick)
        self._playback_wall_start_s: float | None = None
        self._playback_sim_start_s = 0.0
        self._playback_speed_scale = 1.0
        self._playback_speed_offset_percent = 0
        self._simulation_dirty = False
        self._program_dirty = False
        self._clean_status_text = ProgramController.STATUS_NONE
        self._generation_settings: ProgramGenerationSettings = ProgramGenerationSettings()
        self._setup_connections()
        self._refresh_view()



    def _setup_connections(self) -> None:
        self.header_widget.load_program_requested.connect(self._on_load_program_requested)
        self.header_widget.save_program_requested.connect(self._on_save_program_requested)
        self.header_widget.save_program_as_requested.connect(self._on_save_program_as_requested)
        self.header_widget.clear_requested.connect(self._on_clear_requested)
        self.actions_widget.recompute_requested.connect(self._on_recompute_requested)
        self.actions_widget.export_requested.connect(self._on_export_requested)
        self._connect_playback_widget(self.playback_widget)
        self.actions_widget.trajectory_visibility_changed.connect(self._on_trajectory_visibility_changed)
        self.actions_widget.compute_compensation_requested.connect(self._on_compute_compensation_requested)
        self.config_widget.goToRequested.connect(self._on_go_to_requested)
        self.config_widget.keypointSelectionChanged.connect(self._on_keypoint_selection_changed)
        self.config_widget.add_requested.connect(self._on_program_add_requested)
        self.config_widget.edit_requested.connect(self._on_program_edit_requested)
        self.config_widget.delete_requested.connect(self._on_program_delete_requested)
        self.config_widget.editProgramBaseRequested.connect(self._on_edit_program_base_requested)
        self.config_widget.editToolOrientationRequested.connect(self._on_edit_tool_orientation_requested)
        self.config_widget.cartesianDisplayFrameChanged.connect(self._on_program_display_frame_changed)
        self.config_widget.toolSourceChanged.connect(self._on_tool_source_changed)
        self.config_widget.motionModeChanged.connect(self._on_motion_mode_changed)
        self.config_widget.targetModeChanged.connect(self._on_target_mode_changed)
        self.viewer3d_controller.accent_color_changed.connect(self._on_accent_color_changed)
        self.graphs_widget.error_graph_visibility_changed.connect(self._on_error_graph_visibility_changed)
        self.workspace_model.workspace_changed.connect(self._refresh_view)
        self.external_axes_model.axes_values_changed.connect(self._on_external_chain_changed)
        self.external_axes_model.mount_topology_changed.connect(self._on_external_chain_changed)
        self.external_axes_model.axes_changed.connect(self._on_external_chain_changed)
        self.workpiece_controller.workpiece_model.workpiece_changed.connect(self._on_workpiece_changed)

    def register_playback_widget(self, playback_widget: ProgramPlaybackWidget) -> None:
        if playback_widget in self.playback_widgets:
            return
        self.playback_widgets.append(playback_widget)
        self._connect_playback_widget(playback_widget)
        self._initialize_playback_widget(playback_widget)

    def _connect_playback_widget(self, playback_widget: ProgramPlaybackWidget) -> None:
        playback_widget.play_requested.connect(self._on_play_requested)
        playback_widget.pause_requested.connect(self._on_pause_requested)
        playback_widget.stop_requested.connect(self._on_stop_requested)
        playback_widget.time_value_changed.connect(self._on_time_value_changed)
        playback_widget.speed_offset_changed.connect(self._on_speed_offset_changed)

    def _initialize_playback_widget(self, playback_widget: ProgramPlaybackWidget) -> None:
        samples = self._playback_samples()
        if samples:
            playback_widget.set_time_range(0.0, float(samples[-1].time_s))
            playback_widget.set_playback_enabled(True)
            playback_widget.set_time_value(self._current_time_s)
        else:
            playback_widget.set_time_range(0.0, 0.0)
            playback_widget.set_playback_enabled(False)
        playback_widget.set_playing(self._playback_wall_start_s is not None)
        self._set_playback_widget_speed(playback_widget, self._playback_speed_offset_percent)

    def _set_playback_widget_speed(self, playback_widget: ProgramPlaybackWidget, offset_percent: int) -> None:
        playback_widget.set_speed_offset_percent(int(offset_percent))



    def _on_load_program_requested(self) -> None:

        self.DEFAULT_PROGRAMS_DIR.mkdir(parents=True, exist_ok=True)
        file_path, _selected_filter = QFileDialog.getOpenFileName(
            self.program_view,
            "Importer programme robot",
            str(self.DEFAULT_PROGRAMS_DIR),
            "KUKA (*.src);;APT Source (*.apt *.aptsource *.cls *.cl);;CATNC (*.nc *.cnc *.mpf *.gcode);;Tous (*.*)",
        )

        if not file_path:
            return

        self.viewer3d_controller.begin_loading_feedback("Chargement programme KRL ...")
        QTimer.singleShot(0, lambda: self._load_program_from_path(file_path))

    def _on_save_program_requested(self) -> None:
        if self.current_program is None:
            return
        current_path = str(self.current_program.source_path).strip()
        if not current_path:
            self._on_save_program_as_requested()
            return
        self._save_program_to_path(current_path)

    def _on_save_program_as_requested(self) -> None:
        if self.current_program is None:
            return

        self.DEFAULT_PROGRAMS_DIR.mkdir(parents=True, exist_ok=True)
        source_path = (
            Path(self.current_program.source_path)
            if str(self.current_program.source_path).strip()
            else self.DEFAULT_PROGRAMS_DIR / "programme.src"
        )
        default_path = str(source_path if source_path.suffix.lower() == ".src" else source_path.with_suffix(".src"))
        file_path, _selected_filter = QFileDialog.getSaveFileName(
            self.program_view,
            "Enregistrer le programme sous",
            default_path,
            "Programmes KUKA (*.src);;Tous les fichiers (*.*)",
        )
        if not file_path:
            return
        self._save_program_to_path(file_path)

    def _save_program_to_path(self, file_path: str) -> None:
        if self.current_program is None:
            return
        try:
            generation_settings = getattr(self, "_generation_settings", None)
            generate_program_to_path(
                file_path,
                self.current_program,
                generation_settings,
            )
        except OSError as exc:
            QMessageBox.critical(self.program_view, "Programme robot", f"Impossible d'enregistrer le programme.\n{exc}")
            return

        self.current_program = replace(self.current_program, source_path=str(Path(file_path)))
        self._program_dirty = False
        self._clean_status_text = ProgramController.STATUS_SAVED
        self._refresh_view()



    def _load_program_from_path(self, file_path: str) -> None:
        from models.robot_program import ProgramOrigin
        try:
            suffix = Path(file_path).suffix.lower()
            if suffix in {".apt", ".aptsource", ".cls", ".cl"}:
                loaded = load_aptsource_program(file_path)
            elif suffix in {".nc", ".cnc", ".mpf", ".gcode"}:
                loaded = load_catnc_program(file_path)
            else:
                loaded = load_kuka_src_program(file_path)

            is_imported = loaded.origin in {ProgramOrigin.IMPORTED_APT, ProgramOrigin.IMPORTED_CATNC}

            if is_imported:
                # Programmes FAO : base pièce, outil courant
                self._base_source = ProgramBaseSource.WORKPIECE
                self._tool_source = "CURRENT"
                self.config_widget.set_tool_source("CURRENT", emit_signal=False)
                self.config_widget.set_cartesian_display_frame(ReferenceFrame.PROGRAM.value, emit_signal=False)
            else:
                self._manual_base = loaded.program_base_pose.copy()
                self._tool_source = "PROGRAM"
                self.config_widget.set_tool_source("PROGRAM", emit_signal=False)
                self.config_widget.set_cartesian_display_frame(ReferenceFrame.PROGRAM.value, emit_signal=False)

            effective = self._compute_effective_base()
            program_with_base = self._program_with_updated_base_pose(loaded, effective)
            if is_imported:
                self.current_program = self._rebuild_derived_motions(program_with_base, self._generation_settings)
            else:
                self.current_program = program_with_base
            self._program_dirty = False
            self._clean_status_text = ProgramController.STATUS_LOADED
            self._apply_program_tool()
            self._recompute_current_program()
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self.program_view, "Programme robot", f"Impossible de charger le programme.\n{exc}")
            return
        finally:
            self.viewer3d_controller.end_loading_feedback()



    def _on_recompute_requested(self) -> None:
        self.viewer3d_controller.begin_loading_feedback("Calcul de la trajectoire en cours ...")
        try:
            self._recompute_current_program(reset_modes=False)
        finally:
            self.viewer3d_controller.end_loading_feedback()

    

    def _on_clear_requested(self) -> None:

        self.current_program = None
        self._program_dirty = False
        self._clean_status_text = ProgramController.STATUS_NONE
        self._restore_robot_tool()  # Restaurer le tool robot quand on efface le programme
        self._recompute_current_program()



    def _on_context_changed(self, *_args) -> None:

        if self.current_program is None:
            return

        self._recompute_current_program()



    def _recompute_current_program(self, reset_modes: bool = True) -> None:
        self._stop_playback()
        self._current_time_s = 0.0

        if self.current_program is None:
            # Reinitialisation complete
            self._nominal_cartesian_result = None
            self._nominal_articular_result = None
            self._compensated_cartesian_result = None
            self._compensated_articular_result = None
            self._articular_program = None
            self._cartesian_program = None
            self._compensated_cartesian_program = None
            self._compensated_articular_program = None
            self._compensation_computed = False
            self._simulation_dirty = False
            self._program_dirty = False
            self._clean_status_text = ProgramController.STATUS_NONE
            self.actions_widget.set_compensated_checkbox_enabled(False)
            self.current_result = None
            self._display_keypoints = []
            self._display_keypoint_tools = []
            self._display_target_refs = []
            self._nominal_segments_cache = []
            self._measured_segments_cache = []
            self._compensated_segments_cache = []
            self._nom_seg_pts = []
            self._nom_seg_times = []
            self._last_split_sample_index = -1
            self._refresh_view()
            return

        if self._base_source == ProgramBaseSource.WORKPIECE:
            effective = self._compute_effective_base()
            self._update_program_base_pose(effective)

        simulation_program = self._get_simulation_program()
        if simulation_program is None:
            self._nominal_cartesian_result = None
            self._nominal_articular_result = None
            self.current_result = None
            self._simulation_dirty = False
            self._display_keypoints = []
            self._display_keypoint_tools = []
            self._display_target_refs = []
            self._refresh_view()
            return

        program_type = self._detect_program_type()

        if program_type == "CARTESIAN":
            self._nominal_cartesian_result = self.program_simulator.simulate_program(
                simulation_program, include_compensation=False
            )
            self._articular_program = self._build_articular_program(simulation_program)
            if self._articular_program:
                self._nominal_articular_result = self.program_simulator.simulate_program(
                    self._articular_program, include_compensation=False
                )
            else:
                self._nominal_articular_result = None
        else:
            self._nominal_articular_result = self.program_simulator.simulate_program(
                simulation_program, include_compensation=False
            )
            self._cartesian_program = self._build_cartesian_program(simulation_program)
            if self._cartesian_program:
                self._nominal_cartesian_result = self.program_simulator.simulate_program(
                    self._cartesian_program, include_compensation=False
                )
            else:
                self._nominal_cartesian_result = None

        self._compensated_cartesian_result = None
        self._compensated_articular_result = None
        self._compensated_cartesian_program = None
        self._compensated_articular_program = None
        self._compensation_computed = False
        self.actions_widget.set_compensated_checkbox_enabled(False)

        if reset_modes:
            active_motion_mode = program_type
            active_target_mode = "THEORETICAL"
            self.config_widget.set_motion_mode(active_motion_mode, emit_signal=False)
            self.config_widget.set_target_mode(active_target_mode, emit_signal=False)
        else:
            active_motion_mode = self.config_widget.get_motion_mode()
            active_target_mode = self.config_widget.get_target_mode()

        self._update_current_result_from_modes(active_target_mode, active_motion_mode)

        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = (
            self._build_display_keypoints_for_mode(active_motion_mode, active_target_mode)
        )

        _theo_samples = self._get_samples_for_modes("THEORETICAL", active_motion_mode)
        self._nominal_segments_cache, self._measured_segments_cache = self._build_nominal_and_measured_segments(
            _theo_samples,
            self._get_nominal_color(),
            self.MEASURED_COLOR,
        )
        self._nom_seg_pts, self._nom_seg_times = self._build_nom_segs_np(_theo_samples)
        self._compensated_segments_cache = []
        self._simulation_dirty = False

        self.actions_widget.set_compensation_enabled(self.current_program is not None and not self._simulation_dirty)

        self._refresh_view()


    def _refresh_view(self) -> None:
        self._refresh_program_info()
        self._refresh_keypoint_table()
        self._refresh_status()
        self._refresh_viewer_segments()
        self._refresh_viewer_keypoints()
        self._refresh_program_frame()
        self._refresh_error_graph()
        self._refresh_timeline()

    def _on_trajectory_visibility_changed(self, *_args) -> None:
        if self.current_program is None:
            return
        self._refresh_viewer_segments()
        self._refresh_error_graph()



    def _refresh_program_info(self) -> None:

        if self.current_program is None:
            self.header_widget.set_program_info("", 0)
            self.header_widget.set_log_lines([])
            self.header_widget.set_program_save_enabled(False)
            self.header_widget.set_program_save_as_enabled(False)
            self.header_widget.set_program_status(ProgramController.STATUS_NONE, "#808080")
            self.actions_widget.set_export_enabled(False)
            self.actions_widget.set_simulation_enabled(False)
            self.actions_widget.set_compensation_enabled(False)
            self.config_widget.set_program_base_edit_enabled(False)
            self.config_widget.set_tool_orientation_edit_enabled(False)
            return

        log_lines = list(self.current_program.warnings)

        if self.current_result is not None:
            log_lines.extend(self.current_result.warnings)

        self.header_widget.set_program_info(
            self.current_program.source_path,
            len(self.current_program.motions),
        )
        self.header_widget.set_program_save_enabled(True)
        self.header_widget.set_program_save_as_enabled(True)
        self._refresh_program_save_status()

        self.header_widget.set_log_lines(log_lines)
        measured_model_available = self._has_measured_model_available()
        self.actions_widget.set_export_enabled(
            measured_model_available and self._selected_compensated_program() is not None
        )
        self.actions_widget.set_simulation_enabled(self._simulation_dirty)
        is_simulated = self.current_result is not None and not self._simulation_dirty
        self.actions_widget.set_compensation_enabled(is_simulated and measured_model_available)
        self.actions_widget.set_compensated_checkbox_enabled(
            measured_model_available and self._compensation_computed
        )
        self.config_widget.set_program_base_edit_enabled(True)
        self.config_widget.set_tool_orientation_edit_enabled(
            self._count_linear_cartesian_targets(self.current_program) > 0
        )



    def _refresh_keypoint_table(self) -> None:

        if self.current_program is None:
            self.config_widget.set_program_loaded(False)
            self.config_widget.set_keypoints([])
            return

        self.config_widget.set_program_loaded(True)
        self.config_widget.set_keypoints(self._display_keypoints)



    def _refresh_status(self) -> None:

        if self.current_program is None:

            self.actions_widget.set_status_text("Aucun programme charge.")
            self.viewer3d_controller.clear_trajectory_path()
            self.viewer3d_controller.clear_trajectory_status_message()

            return

        if self.current_result is None:
            self.actions_widget.set_status_text("Programme charge, simulation non calculee.")
            return

        nominal_count = len(self.current_result.nominal_samples)
        measured_available = any(sample.measured_pose_base is not None for sample in self.current_result.nominal_samples)
        compensated_program = self._selected_compensated_program()
        compensated_count = len(self._selected_compensated_samples())
        self.actions_widget.set_status_text(
            f"Echantillons theorique: {nominal_count} | "
            f"trajectoire reelle: {'oui' if measured_available else 'non'} | "
            f"compensation: {'oui' if compensated_program is not None else 'non'} | "
            f"aperçu compense: {compensated_count}"

        )



    def _refresh_viewer_segments(self) -> None:
        result = self.current_result
        if result is None:
            self.viewer3d_controller.clear_trajectory_path()
            return

        segments: list[tuple[np.ndarray | list[list[float]], tuple[float, float, float, float] | np.ndarray]] = []

        show_split = self._current_time_s > 1e-9
        theoretical_visible = self.actions_widget.is_theoretical_visible()
        measured_visible = self.actions_widget.is_measured_visible()

        # Trajectoire nominale : geometry pré-construite, couleurs par-vertex vectorisées
        if theoretical_visible and self._nom_seg_pts:
            nominal_color_arr = np.array(self._get_nominal_color(), dtype=np.float64)
            if show_split:
                done_color_arr = np.array(
                    self.viewer3d_controller.get_accent_color_rgba(), dtype=np.float64
                )
                t_split = self._current_time_s
                for pts, times in zip(self._nom_seg_pts, self._nom_seg_times):
                    n = len(pts)
                    colors = np.empty((n, 4), dtype=np.float64)
                    mask = times <= t_split
                    colors[mask] = done_color_arr
                    colors[~mask] = nominal_color_arr
                    segments.append((pts, colors))
            else:
                for pts in self._nom_seg_pts:
                    segments.append((pts, tuple(nominal_color_arr)))

        # Trajectoire mesurée (chemin non critique, approche legacy)
        if measured_visible:
            if show_split:
                done_measured = self._compute_done_color(self.MEASURED_COLOR)
                motion_mode = self.config_widget.get_motion_mode()
                _, split_meas = self._build_nominal_and_measured_segments(
                    self._get_samples_for_modes("THEORETICAL", motion_mode),
                    self._get_nominal_color(),
                    self.MEASURED_COLOR,
                    done_nominal_color=self.viewer3d_controller.get_accent_color_rgba(),
                    done_measured_color=done_measured,
                    split_time_s=self._current_time_s,
                )
                segments.extend(split_meas)
            else:
                segments.extend(self._measured_segments_cache)

        if self._compensation_computed and self.actions_widget.is_compensated_visible():
            segments.extend(self._compensated_segments_cache)

        if segments:
            self.viewer3d_controller.set_trajectory_path_segments(segments)
        else:
            self.viewer3d_controller.clear_trajectory_path()



    def _refresh_viewer_keypoints(self) -> None:

        if not self._display_keypoints:
            self.viewer3d_controller.clear_trajectory_keypoints()
            return

        points_xyz: list[list[float]] = []
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()

        for row, (keypoint, keypoint_tool) in enumerate(zip(self._display_keypoints, self._display_keypoint_tools)):
            resolved_keypoint = self._viewer_keypoint_for_row(row, keypoint)
            point_xyz = resolve_keypoint_xyz(
                self.robot_model,
                resolved_keypoint,
                tool=keypoint_tool,
                robot_base_pose_world=robot_base_transform,

            )

            if point_xyz is None:

                continue

            points_xyz.append(point_xyz.to_list())

        if not points_xyz:

            self.viewer3d_controller.clear_trajectory_keypoints()

            return

        self.viewer3d_controller.set_trajectory_keypoints(points_xyz, self._selected_keypoint_index, None)



    def _refresh_error_graph(self) -> None:
        if not self.graphs_widget.is_error_graph_visible():
            self.graphs_widget.clear()
            return

        motion_mode = self.config_widget.get_motion_mode()
        nominal_samples = self._get_samples_for_modes("THEORETICAL", motion_mode)
        compensated_samples = (
            self._get_samples_for_modes("COMPENSATED", motion_mode)
            if self._compensation_computed and self.actions_widget.is_compensated_visible()
            else []
        )

        abscissa_mm, measured_error_y_mm, compensated_error_y_mm = self.program_simulator.build_error_curves(
            nominal_samples,
            compensated_samples,
        )
        if not self.actions_widget.is_measured_visible():
            measured_error_y_mm = []
        self.graphs_widget.set_error_curves(abscissa_mm, measured_error_y_mm, compensated_error_y_mm)



    def _on_error_graph_visibility_changed(self, visible: bool) -> None:

        if not visible:
            self.graphs_widget.clear()

            return

        self._refresh_error_graph()



    def _refresh_timeline(self) -> None:
        samples = self._playback_samples()
        self._playback_sample_times = [float(sample.time_s) for sample in samples]

        if not samples:
            for playback_widget in self.playback_widgets:
                playback_widget.set_time_range(0.0, 0.0)
                playback_widget.set_playback_enabled(False)
            self._apply_time_value(0.0, samples)
            return

        end_time = float(samples[-1].time_s)

        for playback_widget in self.playback_widgets:
            playback_widget.set_time_range(0.0, end_time)
            playback_widget.set_playback_enabled(True)

        self._apply_time_value(min(self._current_time_s, end_time), samples)



    def _selected_compensated_program(self) -> RobotProgram | None:

        if not self._compensation_computed:

            return None

        if ProgramCompensationOutputMode(self.config_widget.get_motion_mode()) == ProgramCompensationOutputMode.ARTICULAR:

            return self._compensated_articular_program

        return self._compensated_cartesian_program



    def _selected_compensated_samples(self) -> list[ProgramSimulationSample]:

        if not self._compensation_computed:

            return []

        if ProgramCompensationOutputMode(self.config_widget.get_motion_mode()) == ProgramCompensationOutputMode.ARTICULAR:

            return self._compensated_articular_result.nominal_samples if self._compensated_articular_result else []

        return self._compensated_cartesian_result.nominal_samples if self._compensated_cartesian_result else []



    def _playback_samples(self) -> list[ProgramSimulationSample]:
        motion_mode = self.config_widget.get_motion_mode()
        target_mode = self.config_widget.get_target_mode()
        return self._get_samples_for_modes(target_mode, motion_mode)



    def _apply_time_value(self, time_s: float, samples: list[ProgramSimulationSample] | None = None) -> None:
        if samples is None:
            samples = self._playback_samples()
        self._current_time_s = max(0.0, float(time_s))
        for playback_widget in self.playback_widgets:
            playback_widget.set_time_value(self._current_time_s)
        if not samples:
            return
        sample_index = self._sample_index_at_time(self._current_time_s)
        sample = samples[sample_index]
        self.robot_model.set_joints(sample.joints_deg.to_list())
        # Mise à jour directe du viewer (chemin léger, bypasse _refresh_robot_state_items)
        if self.viewer3d_controller.is_playback_active():
            self.viewer3d_controller.update_robot_poses_for_playback()
        # Met à jour la portion "réalisée" si l'index de split a changé (throttlé à ~30 Hz).
        new_split_index = sample_index if self._current_time_s > 1e-9 else -1
        if new_split_index != self._last_split_sample_index:
            self._last_split_sample_index = new_split_index
            now = time.perf_counter()
            if now - self._last_traj_refresh_wall_s >= self._TRAJ_REFRESH_INTERVAL_S:
                self._last_traj_refresh_wall_s = now
                self._refresh_viewer_segments()



    def _sample_index_at_time(self, time_s: float) -> int:

        if not self._playback_sample_times:

            return 0

        times = self._playback_sample_times

        if time_s <= 0.0:

            return 0

        index = bisect_left(times, float(time_s))

        if index <= 0:

            return 0

        if index >= len(times):

            return len(times) - 1

        previous_time = times[index - 1]

        next_time = times[index]

        return index - 1 if (float(time_s) - previous_time) <= (next_time - float(time_s)) else index



    def _on_time_value_changed(self, time_s: float) -> None:

        self._stop_playback()

        self._apply_time_value(time_s)

    def _on_speed_offset_changed(self, offset_percent: int) -> None:
        normalized_percent = max(-100, min(100, int(offset_percent)))
        self._playback_speed_offset_percent = normalized_percent
        if normalized_percent >= 0:
            self._playback_speed_scale = 1.0 + (float(normalized_percent) / 100.0)
        else:
            self._playback_speed_scale = 1.0 / (1.0 + (abs(float(normalized_percent)) / 100.0))
        for playback_widget in self.playback_widgets:
            self._set_playback_widget_speed(playback_widget, normalized_percent)
        if self._playback_wall_start_s is None:
            return
        self._playback_sim_start_s = float(self._current_time_s)
        self._playback_wall_start_s = time.perf_counter()



    def _stop_playback(self) -> None:

        self._playback_wall_start_s = None
        self._last_traj_refresh_wall_s = 0.0

        self._playback_timer.stop()
        self.robot_model.inhibit_ik(False)
        self.viewer3d_controller.set_playback_active(False)
        # Propager l'état final à tous les abonnés UI maintenant que le playback est terminé
        self.robot_model.compute_fk_tcp()
        for playback_widget in self.playback_widgets:
            playback_widget.set_playing(False)



    def _on_play_requested(self) -> None:
        samples = self._playback_samples()
        if not samples:
            return
        end_time_s = float(samples[-1].time_s)
        if self._current_time_s >= end_time_s:
            self._apply_time_value(0.0, samples)
        self._playback_sim_start_s = float(self._current_time_s)
        self._playback_wall_start_s = time.perf_counter()
        self.robot_model.inhibit_ik(True)
        self.viewer3d_controller.set_playback_active(True)
        for playback_widget in self.playback_widgets:
            playback_widget.set_playing(True)
        self._playback_timer.start(20)
        self._on_playback_tick()



    def _on_pause_requested(self) -> None:

        self._stop_playback()



    def _on_stop_requested(self) -> None:
        self._stop_playback()
        self._apply_time_value(0.0)



    def _on_playback_tick(self) -> None:
        samples = self._playback_samples()
        if not samples:
            self._stop_playback()
            return
        wall_start = self._playback_wall_start_s
        if wall_start is None:
            self._playback_wall_start_s = time.perf_counter()
            self._playback_sim_start_s = float(self._current_time_s)
            wall_start = self._playback_wall_start_s
        elapsed_s = max(0.0, time.perf_counter() - wall_start)
        target_time_s = self._playback_sim_start_s + (elapsed_s * self._playback_speed_scale)
        end_time_s = float(samples[-1].time_s)
        if target_time_s >= end_time_s:
            if self._is_loop_enabled():
                self._apply_time_value(0.0, samples)
                self._playback_sim_start_s = 0.0
                self._playback_wall_start_s = time.perf_counter()
                return
            self._stop_playback()
            self._apply_time_value(end_time_s, samples)
            return
        self._apply_time_value(target_time_s, samples)

    def _is_loop_enabled(self) -> bool:
        return any(playback_widget.is_loop_enabled() for playback_widget in self.playback_widgets)



    def _on_export_requested(self) -> None:

        program = self._selected_compensated_program()

        if self.current_program is None or program is None:

            QMessageBox.warning(self.program_view, "Programme robot", "Aucun programme compense a exporter.")

            return

        source_path = Path(self.current_program.source_path)

        suffix = "_compense_articulaire" if ProgramCompensationOutputMode(self.config_widget.get_motion_mode()) == ProgramCompensationOutputMode.ARTICULAR else "_compense_cartesien"

        default_path = str(source_path.with_name(f"{source_path.stem}{suffix}{source_path.suffix}"))

        file_path, _selected_filter = QFileDialog.getSaveFileName(

            self.program_view,

            "Exporter programme compense",

            default_path,

            "Programmes KUKA (*.src)",

        )

        if not file_path:

            return

        try:

            export_kuka_src_program(
                file_path,
                self.current_program.source_text,
                program.motions,
                self.current_program.program_base_pose,
            )

        except OSError as exc:

            QMessageBox.critical(self.program_view, "Programme robot", f"Impossible d'exporter le programme.\n{exc}")

            return

        QMessageBox.information(self.program_view, "Programme robot", f"Programme exporte :\n{file_path}")



    def _on_go_to_requested(self, row: int) -> None:

        if row < 0 or row >= len(self._display_keypoints):

            return

        keypoint = self._display_keypoints[row]

        keypoint_tool = self._display_keypoint_tools[row] if row < len(self._display_keypoint_tools) else self.tool_model.get_tool()

        if keypoint.target_type == KeypointTargetType.JOINT:

            self.robot_model.set_joints(keypoint.joint_target.to_list())

            return

        target_pose = self._target_pose_base_for_row(row)

        if target_pose is None:

            return

        mgi_result = self.robot_model.compute_ik_target(target_pose, tool=keypoint_tool)

        best_solution = self.robot_model.get_best_mgi_solution(mgi_result)

        if best_solution is None:

            return

        self.robot_model.set_joints(best_solution[1].joints[:6])



    def _on_keypoint_selection_changed(self, row: object) -> None:

        self._selected_keypoint_index = row if isinstance(row, int) and row >= 0 else None

        self._refresh_viewer_keypoints()



    def _on_program_display_frame_changed(self, _frame: str) -> None:

        if self.current_program is None:

            return

        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = self._build_display_keypoints()

        self._refresh_keypoint_table()

        self._refresh_viewer_keypoints()



    def _get_simulation_program(self) -> RobotProgram | None:
        if self.current_program is None:

            return None

        

        if self._tool_source == "CURRENT":
            current_tool = self.tool_model.get_tool()
            tool_pose = self.program_simulator._tool_to_pose(current_tool)
            if tool_pose is None:
                return self.current_program
            motions_with_robot_tool = []
            for motion in self.current_program.motions:
                updated_motion = replace(motion, tool_pose=tool_pose)
                motions_with_robot_tool.append(updated_motion)
            return replace(self.current_program, motions=motions_with_robot_tool)
        else:
            return self.current_program



    def _apply_program_tool(self) -> None:
        if self.current_program is None or self._saved_robot_tool is not None:
            return
        self._saved_robot_tool = self.tool_model.get_tool()
        for motion in self.current_program.motions:
            program_tool = self.program_simulator._tool_from_pose(motion.tool_pose)
            if program_tool is not None:
                self.tool_model.set_tool(program_tool)
                break



    def _restore_robot_tool(self) -> None:
        if self._saved_robot_tool is not None:

            self.tool_model.set_tool(self._saved_robot_tool)

            self._saved_robot_tool = None



    def _on_tool_source_changed(self, tool_source: str) -> None:

        self._tool_source = tool_source

        

        if tool_source == "PROGRAM":

            self._apply_program_tool()

        else:

            self._restore_robot_tool()

        

        if self.current_program is not None:

            self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = self._build_display_keypoints()

            self._refresh_keypoint_table()

            self._refresh_viewer_keypoints()
            self._mark_simulation_dirty()
            self._refresh_status()



    def _on_program_edit_requested(self) -> None:

        row = self._selected_keypoint_index

        if row is None:

            return

        self._open_program_target_dialog(row)

    def _on_program_add_requested(self) -> None:

        if self.current_program is None:

            return

        self._open_program_target_dialog(None)

    def _on_program_delete_requested(self) -> None:

        row = self._selected_keypoint_index
        if self.current_program is None or row is None or row < 0 or row >= len(self._display_target_refs):

            return

        target_ref = self._display_target_refs[row]
        motion = self.current_program.motions[target_ref.motion_index]
        motion_label = "mouvement circulaire" if motion.mode == RobotProgramMotionMode.CIRCULAR else "mouvement"
        answer = QMessageBox.question(
            self.program_view,
            "Supprimer une ligne du programme",
            (
                "Voulez-vous supprimer ce "
                f"{motion_label} du programme importe ?"
            ),
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        self.current_program = self._program_with_deleted_motion(target_ref.motion_index)
        self._program_dirty = True
        self._mark_simulation_dirty()
        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = self._build_display_keypoints()
        self._refresh_keypoint_table()
        self._refresh_viewer_keypoints()
        self._refresh_status()
        self._refresh_program_save_status()

        if not self._display_target_refs:
            return
        next_row = min(row, len(self._display_target_refs) - 1)
        self.config_widget.select_row(next_row)

    def _on_edit_program_base_requested(self) -> None:

        if self.current_program is None:
            return

        prev_effective = self._compute_effective_base()

        T_wp_robot = self.workpiece_controller.compute_workpiece_frame_in_robot()
        workpiece_in_robot = matrix_to_pose(np.array(T_wp_robot, dtype=float))
        dialog = ProgramBaseDialog(
            base_source=self._base_source,
            manual_base=self._manual_base,
            base_offset=self._base_offset,
            workpiece_frame_in_robot=workpiece_in_robot,
            parent=self.program_view,
        )
        dialog.base_pose_preview_changed.connect(self._on_program_base_preview_changed)

        if dialog.exec() != dialog.DialogCode.Accepted:
            self._update_program_base_preview(prev_effective)

            return

        new_source, new_manual_base, new_offset = dialog.get_base_config()
        self._base_source = new_source
        self._manual_base = new_manual_base
        self._base_offset = new_offset
        updated_base_pose = self._compute_effective_base()

        if updated_base_pose == prev_effective:

            return

        self._invalidate_simulation_results()
        self._program_dirty = True
        self._refresh_status()
        self._refresh_program_save_status()

    def _on_program_base_preview_changed(self, base_pose: Pose6) -> None:

        self._update_program_base_preview(base_pose)

    def _on_edit_tool_orientation_requested(self) -> None:

        if self.current_program is None:

            return

        linear_target_count = self._count_linear_cartesian_targets(self.current_program)
        if linear_target_count <= 0:
            QMessageBox.information(
                self.program_view,
                "Orientation outil programme",
                "Aucun point cartesien lineaire n'est disponible dans le programme.",
            )
            return

        shared_orientation = self._shared_linear_cartesian_orientation(self.current_program)
        dialog = ProgramToolOrientationDialog(shared_orientation, linear_target_count, self.program_view)

        if dialog.exec() != dialog.DialogCode.Accepted:

            return

        updated_orientation = dialog.get_orientation()
        updated_program = self._program_with_updated_linear_cartesian_orientation(
            self.current_program,
            updated_orientation,
        )
        if updated_program == self.current_program:

            return

        self.current_program = updated_program
        self._program_dirty = True
        self._mark_simulation_dirty()
        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = self._build_display_keypoints()
        self._refresh_keypoint_table()
        self._refresh_viewer_keypoints()
        self._refresh_status()
        self._refresh_program_save_status()



    def _open_program_target_dialog(self, row: int | None) -> None:
        if self.current_program is None:
            return

        if row is None:
            insertion_motion_index = self._insertion_motion_index_for_selected_row()
            draft_motion = self._build_default_program_motion(insertion_motion_index)
            dialog = ProgramTargetDialog(
                self.robot_model,
                draft_motion,
                draft_motion.target,
                False,
                allow_motion_type_editing=True,
                parent=self.program_view,
            )

            if dialog.exec() != dialog.DialogCode.Accepted:
                return

            updated_motion = self._motion_from_dialog(draft_motion, dialog, is_via_target=False)
            self.current_program = self._program_with_inserted_motion(updated_motion, insertion_motion_index)
            self._refresh_program_after_target_change()

            inserted_motion_index = len(self.current_program.motions) - 1 if insertion_motion_index is None else insertion_motion_index + 1
            new_selection_row = self._display_row_for_motion_target(inserted_motion_index)
            if new_selection_row is not None:
                self.config_widget.select_row(new_selection_row)
            return

        if self.current_program is None or row < 0 or row >= len(self._display_target_refs):

            return

        target_ref = self._display_target_refs[row]

        motion = self.current_program.motions[target_ref.motion_index]

        target = motion.via_target if target_ref.is_via_target else motion.target

        if target is None:

            return

        dialog = ProgramTargetDialog(
            self.robot_model,
            motion,
            target,
            target_ref.is_via_target,
            allow_motion_type_editing=not target_ref.is_via_target,
            parent=self.program_view,
        )

        if dialog.exec() != dialog.DialogCode.Accepted:

            return

        updated_motion = self._motion_from_dialog(motion, dialog, is_via_target=target_ref.is_via_target)
        self.current_program = self._program_with_replaced_motion(target_ref.motion_index, updated_motion)
        self._refresh_program_after_target_change()
        self.config_widget.select_row(row)

    def _motion_from_dialog(
        self,
        source_motion: RobotProgramMotion,
        dialog: ProgramTargetDialog,
        is_via_target: bool,
    ) -> RobotProgramMotion:
        updated_target = dialog.get_target()
        if is_via_target:
            return replace(source_motion, via_target=updated_target)

        updated_mode = dialog.get_motion_mode()
        return replace(
            source_motion,
            mode=updated_mode,
            target=updated_target,
            cp_speed_mps=(
                None
                if updated_mode == RobotProgramMotionMode.PTP
                else source_motion.cp_speed_mps or ProgramSimulator.DEFAULT_LINEAR_SPEED_MPS
            ),
        )

    def _refresh_program_after_target_change(self) -> None:
        self._program_dirty = True
        self._mark_simulation_dirty()
        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = self._build_display_keypoints()
        self._refresh_keypoint_table()
        self._refresh_viewer_keypoints()
        self._refresh_status()
        self._refresh_program_save_status()

    def _insertion_motion_index_for_selected_row(self) -> int | None:

        row = self._selected_keypoint_index
        if row is None or row < 0 or row >= len(self._display_target_refs):
            if self.current_program is None or not self.current_program.motions:
                return None
            return len(self.current_program.motions) - 1

        return self._display_target_refs[row].motion_index

    def _build_default_program_motion(self, insertion_motion_index: int | None) -> RobotProgramMotion:

        reference_motion = None
        if self.current_program is not None and insertion_motion_index is not None and 0 <= insertion_motion_index < len(self.current_program.motions):
            reference_motion = self.current_program.motions[insertion_motion_index]

        if reference_motion is None and self.current_program is not None and self.current_program.motions:
            reference_motion = self.current_program.motions[-1]

        if reference_motion is not None:
            target_type = reference_motion.target.target_type
            motion_mode = (
                RobotProgramMotionMode.PTP
                if reference_motion.mode == RobotProgramMotionMode.PTP
                else RobotProgramMotionMode.LINEAR
            )
            base_pose = reference_motion.base_pose.copy()
            tool_pose = reference_motion.tool_pose.copy()
            cp_speed_mps = reference_motion.cp_speed_mps
            if target_type == RobotProgramTargetType.JOINT:
                draft_target = RobotProgramTarget(
                    target_type=RobotProgramTargetType.JOINT,
                    joint_angles=reference_motion.target.joint_angles.copy(),
                )
            else:
                draft_target = RobotProgramTarget(
                    target_type=RobotProgramTargetType.CARTESIAN,
                    cartesian_pose=reference_motion.target.cartesian_pose.copy(),
                )
        else:
            target_type = (
                RobotProgramTargetType.JOINT
                if self.config_widget.get_motion_mode() == "ARTICULAR"
                else RobotProgramTargetType.CARTESIAN
            )
            motion_mode = (
                RobotProgramMotionMode.PTP
                if target_type == RobotProgramTargetType.JOINT
                else RobotProgramMotionMode.LINEAR
            )
            base_pose = self.current_program.program_base_pose.copy() if self.current_program is not None else Pose6.zeros()
            tool_pose = self._program_tool_pose()
            cp_speed_mps = ProgramSimulator.DEFAULT_LINEAR_SPEED_MPS
            if target_type == RobotProgramTargetType.JOINT:
                draft_target = RobotProgramTarget(
                    target_type=RobotProgramTargetType.JOINT,
                    joint_angles=JointAngles6.from_values(self.robot_model.get_joints()),
                )
            else:
                draft_target = RobotProgramTarget(
                    target_type=RobotProgramTargetType.CARTESIAN,
                    cartesian_pose=self._current_tcp_pose_in_program_frame(base_pose),
                )

        return RobotProgramMotion(
            mode=motion_mode,
            target=draft_target,
            line_number=0,
            source="",
            base_pose=base_pose,
            tool_pose=tool_pose,
            cp_speed_mps=cp_speed_mps,
        )

    def _current_tcp_pose_in_program_frame(self, base_pose: Pose6) -> Pose6:

        current_pose_base = self.robot_model.get_tcp_pose()
        return self.program_simulator._pose_from_robot_base_to_program_base(current_pose_base, base_pose)

    def _program_tool_pose(self) -> Pose6:

        if self.current_program is not None and self.current_program.motions:
            return self.current_program.motions[-1].tool_pose.copy()
        current_tool = self.tool_model.get_tool()
        return Pose6.from_values([
            current_tool.tx,
            current_tool.ty,
            current_tool.tz,
            current_tool.rz,
            current_tool.ry,
            current_tool.rx,
        ])

    def _program_with_inserted_motion(self, new_motion: RobotProgramMotion, insertion_motion_index: int | None) -> RobotProgram:

        assert self.current_program is not None

        source_lines = self.current_program.source_text.splitlines()
        motions = list(self.current_program.motions)
        insert_at = len(motions) if insertion_motion_index is None else insertion_motion_index + 1

        if insert_at <= 0:
            insertion_line_number = 1
            source_lines.insert(0, self._format_program_motion_source(new_motion))
        else:
            reference_motion = motions[insert_at - 1]
            insertion_line_number = int(reference_motion.line_number) + 1
            source_line_index = min(max(insertion_line_number - 1, 0), len(source_lines))
            source_lines.insert(source_line_index, self._format_program_motion_source(new_motion))

        updated_motions: list[RobotProgramMotion] = []
        for motion_index, motion in enumerate(motions):
            shifted_line_number = int(motion.line_number) + (1 if motion_index >= insert_at else 0)
            updated_motions.append(replace(motion, line_number=shifted_line_number))

        inserted_motion = replace(
            new_motion,
            line_number=insertion_line_number,
            source=self._format_program_motion_source(new_motion),
        )
        updated_motions.insert(insert_at, inserted_motion)

        return replace(
            self.current_program,
            source_text="\n".join(source_lines),
            motions=updated_motions,
        )

    def _program_with_deleted_motion(self, motion_index: int) -> RobotProgram:

        assert self.current_program is not None

        motions = list(self.current_program.motions)
        if motion_index < 0 or motion_index >= len(motions):
            return self.current_program

        removed_motion = motions[motion_index]
        removed_line_number = int(removed_motion.line_number)
        source_lines = self.current_program.source_text.splitlines()
        source_line_index = removed_line_number - 1
        if 0 <= source_line_index < len(source_lines):
            del source_lines[source_line_index]

        remaining_motions: list[RobotProgramMotion] = []
        for index, motion in enumerate(motions):
            if index == motion_index:
                continue
            shifted_line_number = int(motion.line_number) - (1 if int(motion.line_number) > removed_line_number else 0)
            remaining_motions.append(replace(motion, line_number=shifted_line_number))

        return replace(
            self.current_program,
            source_text="\n".join(source_lines),
            motions=remaining_motions,
        )

    def _program_with_replaced_motion(self, motion_index: int, new_motion: RobotProgramMotion) -> RobotProgram:

        assert self.current_program is not None

        motions = list(self.current_program.motions)
        if motion_index < 0 or motion_index >= len(motions):
            return self.current_program

        existing_motion = motions[motion_index]
        updated_motion = replace(
            new_motion,
            line_number=existing_motion.line_number,
            source=self._format_program_motion_source(new_motion),
        )
        motions[motion_index] = updated_motion

        source_lines = self.current_program.source_text.splitlines()
        source_line_index = int(existing_motion.line_number) - 1
        if 0 <= source_line_index < len(source_lines):
            source_lines[source_line_index] = updated_motion.source

        return replace(
            self.current_program,
            source_text="\n".join(source_lines),
            motions=motions,
        )


    def _display_row_for_motion_target(self, motion_index: int) -> int | None:

        for row_index, target_ref in enumerate(self._display_target_refs):
            if target_ref.motion_index == motion_index and not target_ref.is_via_target:
                return row_index
        return None

    @staticmethod
    def _format_program_motion_source(motion: RobotProgramMotion) -> str:

        target_text = ProgramController._format_program_target_source(motion.target)
        if motion.mode == RobotProgramMotionMode.PTP:
            return f"PTP {target_text}"
        if motion.mode == RobotProgramMotionMode.LINEAR:
            return f"LIN {target_text}"
        if motion.mode == RobotProgramMotionMode.CIRCULAR and motion.via_target is not None:
            via_text = ProgramController._format_program_target_source(motion.via_target)
            return f"CIRC {via_text}, {target_text}"
        return motion.source

    @staticmethod
    def _format_program_target_source(target: RobotProgramTarget) -> str:

        if target.target_type == RobotProgramTargetType.JOINT:
            joint_values = target.joint_angles.to_list()
            return (
                "{A1 "
                f"{joint_values[0]:.3f},A2 {joint_values[1]:.3f},A3 {joint_values[2]:.3f},"
                f"A4 {joint_values[3]:.3f},A5 {joint_values[4]:.3f},A6 {joint_values[5]:.3f}"
                "}"
            )

        pose_values = target.cartesian_pose.to_list()
        return (
            "{X "
            f"{pose_values[0]:.3f},Y {pose_values[1]:.3f},Z {pose_values[2]:.3f},"
            f"A {pose_values[3]:.3f},B {pose_values[4]:.3f},C {pose_values[5]:.3f}"
            "}"
        )



    def _viewer_keypoint_for_row(self, row: int, keypoint: TrajectoryKeypoint) -> TrajectoryKeypoint:

        if keypoint.target_type != KeypointTargetType.CARTESIAN or keypoint.cartesian_frame != ReferenceFrame.PROGRAM:

            return keypoint

        target_pose_base = self._target_pose_base_for_row(row)

        if target_pose_base is None:

            return keypoint

        viewer_keypoint = keypoint.clone()

        viewer_keypoint.cartesian_target = target_pose_base

        viewer_keypoint.cartesian_frame = ReferenceFrame.ROBOT

        return viewer_keypoint

    def _display_motion_tool(self, motion: RobotProgramMotion) -> RobotTool:

        current_tool = self.tool_model.get_tool()

        if self._tool_source != "PROGRAM":

            return current_tool

        program_tool = self.program_simulator._tool_from_pose(motion.tool_pose)

        if program_tool is not None:

            return program_tool

        return current_tool

    def _program_base_pose(self) -> Pose6 | None:

        if self.current_program is None:

            return None

        return self.current_program.program_base_pose.copy()

    def _compute_effective_base(self) -> Pose6:
        if self._base_source == ProgramBaseSource.WORKPIECE:
            T_source = self.workpiece_controller.compute_workpiece_frame_in_robot()
        else:
            T_source = pose_to_matrix(self._manual_base)
        offset = self._base_offset
        if offset == Pose6.zeros():
            return matrix_to_pose(T_source)
        return matrix_to_pose(T_source @ pose_to_matrix(offset))

    def _on_workpiece_changed(self) -> None:
        if self._base_source != ProgramBaseSource.WORKPIECE or self.current_program is None:
            return
        effective = self._compute_effective_base()
        self._update_program_base_preview(effective)
        self._mark_simulation_dirty()

    def _on_external_chain_changed(self, *_args) -> None:
        if self.current_program is None:
            return
        if self._base_source != ProgramBaseSource.WORKPIECE:
            return
        new_base = self._compute_effective_base()
        self._update_program_base_preview(new_base)
        if not self._simulation_dirty:
            self._mark_simulation_dirty()

    def get_base_config_state(self) -> dict:
        return {
            "base_source": self._base_source.value,
            "base_offset": self._base_offset.to_list(),
        }

    def load_base_config_state(self, state: dict) -> None:
        source_str = state.get("base_source", "MANUAL")
        try:
            self._base_source = ProgramBaseSource(source_str)
        except ValueError:
            self._base_source = ProgramBaseSource.MANUAL
        raw_offset = state.get("base_offset", [0.0] * 6)
        self._base_offset = (
            Pose6.from_values(raw_offset)
            if isinstance(raw_offset, list) and len(raw_offset) == 6
            else Pose6.zeros()
        )

    @staticmethod
    def _count_linear_cartesian_targets(program: RobotProgram | None) -> int:

        if program is None:

            return 0

        return sum(
            1
            for motion in program.motions
            if motion.mode == RobotProgramMotionMode.LINEAR
            and motion.target.target_type == RobotProgramTargetType.CARTESIAN
        )

    @staticmethod
    def _shared_linear_cartesian_orientation(program: RobotProgram | None) -> Pose6 | None:

        if program is None:

            return None

        shared_pose: Pose6 | None = None
        for motion in program.motions:
            if motion.mode != RobotProgramMotionMode.LINEAR:
                continue
            if motion.target.target_type != RobotProgramTargetType.CARTESIAN:
                continue

            current_pose = motion.target.cartesian_pose
            if shared_pose is None:
                shared_pose = current_pose.copy()
                continue

            if (
                current_pose.a != shared_pose.a
                or current_pose.b != shared_pose.b
                or current_pose.c != shared_pose.c
            ):
                return None

        return shared_pose

    @staticmethod
    def _program_with_updated_linear_cartesian_orientation(
        program: RobotProgram | None,
        orientation_pose: Pose6,
    ) -> RobotProgram | None:

        if program is None:

            return None

        updated_motions: list[RobotProgramMotion] = []
        for motion in program.motions:
            if (
                motion.mode == RobotProgramMotionMode.LINEAR
                and motion.target.target_type == RobotProgramTargetType.CARTESIAN
            ):
                target_pose = motion.target.cartesian_pose
                updated_target = replace(
                    motion.target,
                    cartesian_pose=Pose6(
                        target_pose.x,
                        target_pose.y,
                        target_pose.z,
                        orientation_pose.a,
                        orientation_pose.b,
                        orientation_pose.c,
                    ),
                )
                updated_motions.append(replace(motion, target=updated_target))
                continue

            updated_motions.append(motion)

        return replace(program, motions=updated_motions)

    @staticmethod
    def _program_with_updated_base_pose(program: RobotProgram | None, base_pose: Pose6) -> RobotProgram | None:

        if program is None:

            return None

        updated_motions = [
            replace(motion, base_pose=base_pose.copy())
            for motion in program.motions
        ]

        return replace(program, program_base_pose=base_pose.copy(), motions=updated_motions)

    def _update_program_base_pose(self, base_pose: Pose6) -> None:

        if self.current_program is None:

            return

        updated_base_pose = base_pose.copy()
        self.current_program = self._program_with_updated_base_pose(self.current_program, updated_base_pose)
        self._articular_program = self._program_with_updated_base_pose(self._articular_program, updated_base_pose)
        self._cartesian_program = self._program_with_updated_base_pose(self._cartesian_program, updated_base_pose)
        self._compensated_cartesian_program = self._program_with_updated_base_pose(
            self._compensated_cartesian_program,
            updated_base_pose,
        )
        self._compensated_articular_program = self._program_with_updated_base_pose(
            self._compensated_articular_program,
            updated_base_pose,
        )

    def _update_program_base_preview(self, base_pose: Pose6) -> None:

        self._update_program_base_pose(base_pose)
        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = self._build_display_keypoints()
        self._refresh_keypoint_table()
        self._refresh_viewer_keypoints()
        self._refresh_program_frame()

    def _invalidate_simulation_results(self) -> None:

        self.current_result = None
        self._nominal_cartesian_result = None
        self._nominal_articular_result = None
        self._compensated_cartesian_result = None
        self._compensated_articular_result = None
        self._compensation_computed = False
        self._simulation_dirty = True
        self._nominal_segments_cache = []
        self._measured_segments_cache = []
        self._compensated_segments_cache = []
        self._nom_seg_pts = []
        self._nom_seg_times = []
        self._last_split_sample_index = -1
        self.actions_widget.set_compensated_checkbox_enabled(False)
        self.actions_widget.set_simulation_enabled(True)
        self.actions_widget.set_compensation_enabled(False)
        self.viewer3d_controller.clear_trajectory_path()

    def _mark_simulation_dirty(self) -> None:
        if self.current_program is None:
            return
        self._invalidate_simulation_results()

    def _refresh_program_save_status(self) -> None:
        if self.current_program is None:
            self.header_widget.set_program_status(ProgramController.STATUS_NONE, "#808080")
            return
        if self._program_dirty:
            self.header_widget.set_program_status(ProgramController.STATUS_MODIFIED, "#f2c94c")
            return
        status_text = self._clean_status_text or ProgramController.STATUS_LOADED
        status_color = "#6fcf97" if status_text in {
            ProgramController.STATUS_LOADED,
            ProgramController.STATUS_SAVED,
        } else "#808080"
        self.header_widget.set_program_status(status_text, status_color)

    def _refresh_program_frame(self) -> None:

        program_base_pose = self._program_base_pose()

        if program_base_pose is None:

            self.viewer3d_controller.clear_program_frame()

            return

        self.viewer3d_controller.set_program_frame(program_base_pose, "Program Frame")



    def _target_pose_base_for_row(self, row: int) -> Pose6 | None:

        if self.current_program is None or row < 0 or row >= len(self._display_target_refs):

            return None

        target_ref = self._display_target_refs[row]

        motion = self.current_program.motions[target_ref.motion_index]

        target = motion.via_target if target_ref.is_via_target else motion.target

        if target is None or target.target_type != RobotProgramTargetType.CARTESIAN:

            return None

        return self.program_simulator._pose_from_program_base_to_robot_base(target.cartesian_pose, motion.base_pose)



    @staticmethod

    def _downsample_segments(

        segments: list[tuple[list[list[float]], tuple[float, float, float, float]]],

        max_points: int = 3000,

    ) -> list[tuple[list[list[float]], tuple[float, float, float, float]]]:

        total_points = sum(len(points) for points, _color in segments)

        if total_points <= max_points:

            return segments

        stride = max(1, int(total_points / max_points))

        reduced_segments: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []

        for points, color in segments:

            if len(points) <= 2:

                reduced_segments.append((points, color))

                continue

            reduced_points = points[::stride]

            if reduced_points[-1] != points[-1]:

                reduced_points.append(points[-1])

            reduced_segments.append((reduced_points, color))

        return reduced_segments



    def _motion_target_to_keypoint(

        self,

        motion: RobotProgramMotion,

        target: RobotProgramTarget,

        is_circular: bool,

        display_frame: ReferenceFrame,

    ) -> TrajectoryKeypoint | None:

        if target.target_type == RobotProgramTargetType.JOINT:

            mode = KeypointMotionMode.PTP

            return TrajectoryKeypoint(

                target_type=KeypointTargetType.JOINT,

                joint_target=target.joint_angles,

                mode=mode,

                ptp_speed_percent=ProgramSimulator.DEFAULT_PTP_SPEED_PERCENT,

            )

        if display_frame == ReferenceFrame.PROGRAM:

            display_pose = target.cartesian_pose.copy()

        else:

            display_pose = self.program_simulator._pose_from_program_base_to_robot_base(target.cartesian_pose, motion.base_pose)

        mode = KeypointMotionMode.LINEAR if (motion.mode.value != "PTP" or is_circular) else KeypointMotionMode.PTP

        return TrajectoryKeypoint(

            target_type=KeypointTargetType.CARTESIAN,

            cartesian_target=display_pose,

            cartesian_frame=display_frame,

            mode=mode,

            linear_speed_mps=motion.cp_speed_mps if motion.cp_speed_mps is not None else ProgramSimulator.DEFAULT_LINEAR_SPEED_MPS,

            ptp_speed_percent=ProgramSimulator.DEFAULT_PTP_SPEED_PERCENT,

        )



    



    def _detect_program_type(self) -> str:
        if self.current_program is None:
            return "CARTESIAN"
        has_cartesian = any(
            motion.target.target_type == RobotProgramTargetType.CARTESIAN
            for motion in self.current_program.motions
        )
        return "CARTESIAN" if has_cartesian else "ARTICULAR"

    def _build_articular_program(self, source_program: RobotProgram | None = None) -> RobotProgram | None:
        program = source_program or self.current_program
        if program is None:
            return None

        new_motions: list[RobotProgramMotion] = []

        for motion in program.motions:
            motion_tool = self.program_simulator._tool_from_pose(motion.tool_pose)

            # Convertir la cible principale
            if motion.target.target_type == RobotProgramTargetType.JOINT:
                new_target = motion.target
            else:
                # CARTESIAN -> JOINT via IK
                target_pose_base = self.program_simulator._target_pose_base(motion, motion.target, motion_tool)
                mgi_result = self.robot_model.compute_ik_target(target_pose_base, tool=motion_tool)
                if mgi_result is None:
                    new_motions.append(motion)
                    continue
                best_solution = self.robot_model.get_best_mgi_solution(mgi_result)
                if best_solution is None:
                    new_motions.append(motion)
                    continue
                new_joints = [float(x) for x in best_solution[1].joints[:6]]
                new_target = RobotProgramTarget(
                    target_type=RobotProgramTargetType.JOINT,
                    joint_angles=JointAngles6.from_values(new_joints)
                )

            # Convertir via_target si CIRCULAR
            new_via_target = None
            if motion.mode == RobotProgramMotionMode.CIRCULAR and motion.via_target is not None:
                if motion.via_target.target_type == RobotProgramTargetType.JOINT:
                    new_via_target = motion.via_target
                else:
                    via_pose_base = self.program_simulator._target_pose_base(motion, motion.via_target, motion_tool)
                    via_mgi_result = self.robot_model.compute_ik_target(via_pose_base, tool=motion_tool)
                    if via_mgi_result is not None:
                        via_best_solution = self.robot_model.get_best_mgi_solution(via_mgi_result)
                        if via_best_solution is not None:
                            via_joints = [float(x) for x in via_best_solution[1].joints[:6]]
                            new_via_target = RobotProgramTarget(
                                target_type=RobotProgramTargetType.JOINT,
                                joint_angles=JointAngles6.from_values(via_joints)
                            )

            # TOUT devient PTP en mode articulaire
            new_motion = RobotProgramMotion(
                mode=RobotProgramMotionMode.PTP,
                target=new_target,
                via_target=new_via_target,
                line_number=motion.line_number,
                source=motion.source,
                base_pose=motion.base_pose,
                tool_pose=motion.tool_pose,
                cp_speed_mps=motion.cp_speed_mps,
            )
            new_motions.append(new_motion)

        return replace(program, motions=new_motions)

    def _build_cartesian_program(self, source_program: RobotProgram | None = None) -> RobotProgram | None:
        program = source_program or self.current_program
        if program is None:
            return None

        new_motions: list[RobotProgramMotion] = []

        for motion in program.motions:
            motion_tool = self.program_simulator._tool_from_pose(motion.tool_pose)

            if motion.target.target_type == RobotProgramTargetType.CARTESIAN:
                new_target = motion.target
            else:
                # JOINT -> CARTESIAN via FK, résultat converti en repère programme
                fk_result = self.robot_model.compute_fk_joints(
                    motion.target.joint_angles.to_list(), tool=motion_tool
                )
                if fk_result is None:
                    new_motions.append(motion)
                    continue
                new_pose = self.program_simulator._pose_from_robot_base_to_program_base(
                    fk_result.dh_pose, motion.base_pose
                )
                new_target = RobotProgramTarget(
                    target_type=RobotProgramTargetType.CARTESIAN,
                    cartesian_pose=new_pose
                )

            # Convertir via_target si CIRCULAR
            new_via_target = None
            if motion.mode == RobotProgramMotionMode.CIRCULAR and motion.via_target is not None:
                if motion.via_target.target_type == RobotProgramTargetType.CARTESIAN:
                    new_via_target = motion.via_target
                else:
                    via_fk_result = self.robot_model.compute_fk_joints(
                        motion.via_target.joint_angles.to_list(), tool=motion_tool
                    )
                    if via_fk_result is not None:
                        via_pose = self.program_simulator._pose_from_robot_base_to_program_base(
                            via_fk_result.dh_pose, motion.base_pose
                        )
                        new_via_target = RobotProgramTarget(
                            target_type=RobotProgramTargetType.CARTESIAN,
                            cartesian_pose=via_pose
                        )

            # Conserver le mode original pour les cibles cartesiennes
            new_mode = motion.mode

            new_motion = RobotProgramMotion(
                mode=new_mode,
                target=new_target,
                via_target=new_via_target,
                line_number=motion.line_number,
                source=motion.source,
                base_pose=motion.base_pose,
                tool_pose=motion.tool_pose,
                cp_speed_mps=motion.cp_speed_mps,
            )
            new_motions.append(new_motion)

        return replace(program, motions=new_motions)

    def _compute_compensation(self) -> None:
        if self.current_program is None:
            self._compensation_computed = False
            return

        measured_dh = self.program_simulator._normalized_measured_dh_table()
        if measured_dh is None:
            self._compensation_computed = False
            return

        cartesian_prog = self._get_program_for_mode("CARTESIAN") or self.current_program
        cartesian_comp_program = self.program_simulator._build_compensated_program(
            cartesian_prog, ProgramCompensationOutputMode.CARTESIAN, measured_dh
        )
        if cartesian_comp_program:
            self._compensated_cartesian_program = cartesian_comp_program
            self._compensated_cartesian_result = self.program_simulator.simulate_program(
                cartesian_comp_program, include_compensation=False
            )

        articular_prog = self._cartesian_program or self.current_program
        articular_comp_program = self.program_simulator._build_compensated_program(
            articular_prog, ProgramCompensationOutputMode.ARTICULAR, measured_dh
        )
        if articular_comp_program:
            self._compensated_articular_program = articular_comp_program
            self._compensated_articular_result = self.program_simulator.simulate_program(
                articular_comp_program, include_compensation=False
            )

        self._compensation_computed = True

    def _has_measured_model_available(self) -> bool:
        return self.program_simulator._normalized_measured_dh_table() is not None

    def _get_samples_for_modes(self, target_mode: str, motion_mode: str) -> list[ProgramSimulationSample]:
        if target_mode == "THEORETICAL":
            if motion_mode == "ARTICULAR" and self._nominal_articular_result:
                return self._nominal_articular_result.nominal_samples
            if self._nominal_cartesian_result:
                return self._nominal_cartesian_result.nominal_samples
        else:
            if motion_mode == "ARTICULAR" and self._compensated_articular_result:
                return self._compensated_articular_result.nominal_samples
            if self._compensated_cartesian_result:
                return self._compensated_cartesian_result.nominal_samples
        return []

    def _get_program_for_mode(self, motion_mode: str) -> RobotProgram | None:
        if motion_mode == "ARTICULAR":
            return self._articular_program or self.current_program
        return self._cartesian_program or self.current_program

    def _update_current_result_from_modes(self, target_mode: str, motion_mode: str) -> None:
        if target_mode == "THEORETICAL":
            if motion_mode == "ARTICULAR":
                self.current_result = self._nominal_articular_result
            else:
                self.current_result = self._nominal_cartesian_result
        else:
            nominal_samples = self._get_samples_for_modes("THEORETICAL", motion_mode)
            compensated_samples = self._get_samples_for_modes("COMPENSATED", motion_mode)

            self.current_result = ProgramSimulationResult(
                nominal_samples=nominal_samples,
                cartesian_compensated_samples=compensated_samples if motion_mode == "CARTESIAN" else [],
                articular_compensated_samples=compensated_samples if motion_mode == "ARTICULAR" else [],
                cartesian_compensated_program=self._compensated_cartesian_program if motion_mode == "CARTESIAN" else None,
                articular_compensated_program=self._compensated_articular_program if motion_mode == "ARTICULAR" else None,
                warnings=self._collect_all_warnings(),
                compensation_computed=self._compensation_computed,
            )

    def _collect_all_warnings(self) -> list[str]:
        warnings: list[str] = []
        for result in [
            self._nominal_cartesian_result,
            self._nominal_articular_result,
            self._compensated_cartesian_result,
            self._compensated_articular_result,
        ]:
            if result:
                warnings.extend(result.warnings)
        if self.current_program:
            warnings.extend(self.current_program.warnings)
        return list(dict.fromkeys(warnings))

    def _on_motion_mode_changed(self, motion_mode: str) -> None:
        if self.current_program is None:
            return

        target_mode = self.config_widget.get_target_mode()
        self._update_current_result_from_modes(target_mode, motion_mode)

        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = (
            self._build_display_keypoints_for_mode(motion_mode, target_mode)
        )

        _theo_samples_2 = self._get_samples_for_modes("THEORETICAL", motion_mode)
        self._nominal_segments_cache, self._measured_segments_cache = self._build_nominal_and_measured_segments(
            _theo_samples_2,
            self._get_nominal_color(),
            self.MEASURED_COLOR,
        )
        self._nom_seg_pts, self._nom_seg_times = self._build_nom_segs_np(_theo_samples_2)
        self._compensated_segments_cache = self._build_segments(
            self._get_samples_for_modes("COMPENSATED", motion_mode),
            self.COMPENSATED_COLOR,
        ) if self._compensation_computed else []

        self._refresh_view()

    def _on_target_mode_changed(self, target_mode: str) -> None:
        if self.current_program is None:
            return

        motion_mode = self.config_widget.get_motion_mode()
        self._update_current_result_from_modes(target_mode, motion_mode)

        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = (
            self._build_display_keypoints_for_mode(motion_mode, target_mode)
        )
        self._refresh_status()
        self._refresh_keypoint_table()
        self._refresh_viewer_segments()
        self._refresh_error_graph()
        self._refresh_timeline()

    def _on_compute_compensation_requested(self) -> None:
        if self.current_program is None:
            return

        self.viewer3d_controller.begin_loading_feedback("Calcul de la compensation en cours ...")
        try:
            self._compute_compensation()
            if self._compensation_computed:
                self.config_widget.set_target_mode_enabled(True)
                self.actions_widget.set_compensated_checkbox_enabled(True)

            motion_mode = self.config_widget.get_motion_mode()
            self._compensated_segments_cache = self._build_segments(
                self._get_samples_for_modes("COMPENSATED", motion_mode),
                self.COMPENSATED_COLOR,
            )
            self._refresh_viewer_segments()
            self._refresh_error_graph()
        finally:
            self.viewer3d_controller.end_loading_feedback()

    def _get_program_for_target_and_motion_mode(self, target_mode: str, motion_mode: str) -> RobotProgram | None:
        if target_mode == "COMPENSATED" and self._compensation_computed:
            if motion_mode == "ARTICULAR":
                return self._compensated_articular_program or self._compensated_cartesian_program
            return self._compensated_cartesian_program or self._compensated_articular_program
        return self._get_program_for_mode(motion_mode)

    def _build_display_keypoints_for_mode(self, motion_mode: str, target_mode: str | None = None) -> tuple[list[TrajectoryKeypoint], list[RobotTool], list[_ProgramTargetRef]]:
        resolved_target_mode = target_mode or self.config_widget.get_target_mode()
        program = self._get_program_for_target_and_motion_mode(resolved_target_mode, motion_mode)
        if program is None:
            return [], [], []

        keypoints: list[TrajectoryKeypoint] = []
        keypoint_tools: list[RobotTool] = []
        target_refs: list[_ProgramTargetRef] = []

        display_frame = ReferenceFrame.from_value(
            self.config_widget.cartesian_display_frame_combo.currentData(),
            ReferenceFrame.PROGRAM,
        )

        robot_tool = self.tool_model.get_tool()

        for motion_index, motion in enumerate(program.motions):
            if self._tool_source == "PROGRAM":
                motion_tool = self.program_simulator._tool_from_pose(motion.tool_pose) or robot_tool
            else:
                motion_tool = robot_tool

            if motion.mode == RobotProgramMotionMode.CIRCULAR and motion.via_target is not None:
                via_keypoint = self._motion_target_to_keypoint(motion, motion.via_target, is_circular=True, display_frame=display_frame)
                if via_keypoint is not None:
                    keypoints.append(via_keypoint)
                    keypoint_tools.append(motion_tool)
                    target_refs.append(_ProgramTargetRef(motion_index=motion_index, is_via_target=True))

            keypoint = self._motion_target_to_keypoint(motion, motion.target, is_circular=False, display_frame=display_frame)
            if keypoint is not None:
                keypoints.append(keypoint)
                keypoint_tools.append(motion_tool)
                target_refs.append(_ProgramTargetRef(motion_index=motion_index, is_via_target=False))

        return keypoints, keypoint_tools, target_refs

    def _build_display_keypoints(self) -> tuple[list[TrajectoryKeypoint], list[RobotTool], list[_ProgramTargetRef]]:
        motion_mode = self.config_widget.get_motion_mode()
        return self._build_display_keypoints_for_mode(motion_mode)


    def _get_nominal_color(self) -> tuple[float, float, float, float]:
        return (1.0, 1.0, 1.0, 1.0)  # blanc : trajectoire non encore réalisée

    def _on_accent_color_changed(self) -> None:
        if self.current_result is None:
            return
        # Le cache nominal est blanc (constante), pas besoin de le reconstruire.
        # Le split "réalisé" lit l'accent dynamiquement dans _refresh_viewer_segments.
        self._refresh_viewer_segments()

    @staticmethod
    def _compute_done_color(accent: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        c = QColor.fromRgbF(accent[0], accent[1], accent[2])
        h, s, v, _ = c.getHsvF()
        # Même teinte, très désaturé et plus clair — effet "réalisé/passé"
        done = QColor.fromHsvF(h, s * 0.25, min(1.0, v * 0.55 + 0.55))
        return (done.redF(), done.greenF(), done.blueF(), 1.0)

    @staticmethod
    def _build_nominal_and_measured_segments(
        samples: list[ProgramSimulationSample],
        nominal_color: tuple[float, float, float, float],
        measured_color: tuple[float, float, float, float],
        done_nominal_color: tuple[float, float, float, float] | None = None,
        done_measured_color: tuple[float, float, float, float] | None = None,
        split_time_s: float | None = None,
    ) -> tuple[
        list[tuple[list[list[float]], tuple[float, float, float, float]]],
        list[tuple[list[list[float]], tuple[float, float, float, float]]],
    ]:
        nominal_segments: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        measured_segments: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []

        use_split = (split_time_s is not None
                     and done_nominal_color is not None
                     and done_measured_color is not None)

        nom_points: list[list[float]] = []
        meas_points: list[list[float]] = []
        current_key: tuple[str, int] | None = None
        current_nom_is_done: bool = False
        current_meas_is_done: bool = False

        def _flush_nom(is_done: bool) -> None:
            if len(nom_points) >= 2:
                color = done_nominal_color if is_done else nominal_color
                nominal_segments.append((nom_points[:], color))

        def _flush_meas(is_done: bool) -> None:
            if len(meas_points) >= 2:
                color = done_measured_color if is_done else measured_color
                measured_segments.append((meas_points[:], color))

        for sample in samples:
            nom_pose = sample.nominal_pose_base
            meas_pose = sample.measured_pose_base
            if nom_pose is None and meas_pose is None:
                continue

            motion_key = (sample.motion_mode.value, int(sample.source_line))
            sample_is_done = use_split and float(sample.time_s) <= split_time_s

            motion_break = current_key is not None and motion_key != current_key

            if motion_break:
                _flush_nom(current_nom_is_done)
                nom_points = [nom_points[-1]] if nom_points else []
                _flush_meas(current_meas_is_done)
                meas_points = [meas_points[-1]] if meas_points else []
                current_nom_is_done = sample_is_done
                current_meas_is_done = sample_is_done
            elif use_split and nom_points and sample_is_done != current_nom_is_done:
                # Couleur change : on coupe le segment nominal au point de transition
                _flush_nom(current_nom_is_done)
                nom_points = [nom_points[-1]]
                current_nom_is_done = sample_is_done
            elif use_split and meas_points and sample_is_done != current_meas_is_done:
                _flush_meas(current_meas_is_done)
                meas_points = [meas_points[-1]]
                current_meas_is_done = sample_is_done

            current_key = motion_key
            if not use_split:
                current_nom_is_done = False
                current_meas_is_done = False
            else:
                current_nom_is_done = sample_is_done
                current_meas_is_done = sample_is_done

            if nom_pose is not None:
                nom_points.append([nom_pose.x, nom_pose.y, nom_pose.z])
            if meas_pose is not None:
                meas_points.append([meas_pose.x, meas_pose.y, meas_pose.z])

        if current_key is not None:
            _flush_nom(current_nom_is_done)
            _flush_meas(current_meas_is_done)

        return nominal_segments, measured_segments

    @staticmethod
    def _build_nom_segs_np(
        samples: list[ProgramSimulationSample],
    ) -> tuple[list[np.ndarray], list[np.ndarray]]:
        """Pré-construit les segments nominaux comme numpy arrays (robot frame).

        Retourne deux listes parallèles :
        - pts_segs  : (K, 3) float64 par segment
        - time_segs : (K,)   float64 par segment (temps en secondes par vertex)
        """
        pts_segs: list[np.ndarray] = []
        time_segs: list[np.ndarray] = []
        cur_pts: list[list[float]] = []
        cur_times: list[float] = []
        cur_key: tuple[str, int] | None = None

        def _flush() -> None:
            if len(cur_pts) >= 2:
                pts_segs.append(np.array(cur_pts, dtype=np.float64))
                time_segs.append(np.array(cur_times, dtype=np.float64))

        for sample in samples:
            nom = sample.nominal_pose_base
            if nom is None:
                continue
            key = (sample.motion_mode.value, int(sample.source_line))
            if cur_key is not None and key != cur_key:
                _flush()
                cur_pts = [cur_pts[-1]] if cur_pts else []
                cur_times = [cur_times[-1]] if cur_times else []
            cur_key = key
            cur_pts.append([nom.x, nom.y, nom.z])
            cur_times.append(float(sample.time_s))

        _flush()
        return pts_segs, time_segs

    def _build_segments(
        self,
        samples: list[ProgramSimulationSample],
        color: tuple[float, float, float, float],
    ) -> list[tuple[list[list[float]], tuple[float, float, float, float]]]:
        segments: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        current_points: list[list[float]] = []
        current_key: tuple[str, int] | None = None

        for sample in samples:
            pose = sample.measured_pose_base
            if pose is None:
                continue
            motion_key = (sample.motion_mode.value, int(sample.source_line))
            if current_key is not None and motion_key != current_key and len(current_points) >= 2:
                segments.append((current_points, color))
                current_points = [current_points[-1]]
            current_key = motion_key
            current_points.append([pose.x, pose.y, pose.z])

        if current_key is not None and len(current_points) >= 2:
            segments.append((current_points, color))

        return segments

    # ---------------------------------------------------------------------------
    # Réglages de génération (HOME / approche / retrait / header / approximation)
    # ---------------------------------------------------------------------------

    def get_generation_settings(self) -> ProgramGenerationSettings:
        return self._generation_settings

    def set_generation_settings(self, settings: ProgramGenerationSettings) -> None:
        self._generation_settings = settings
        if self.current_program is not None:
            self.current_program = self._rebuild_derived_motions(self.current_program, settings)
            self._program_dirty = True
            self._recompute_current_program(reset_modes=False)

    def get_generation_settings_dict(self) -> dict:
        return self._generation_settings.to_dict()

    def load_generation_settings_dict(self, data: dict) -> None:
        self._generation_settings = ProgramGenerationSettings.from_dict(data)

    def _rebuild_derived_motions(
        self,
        program: RobotProgram,
        settings: ProgramGenerationSettings,
    ) -> RobotProgram:
        """Retire les motions dérivées (rôle ≠ NORMAL) et reconstruit HOME/APPROACH/RETRACT."""
        # Garder uniquement les NORMAL
        normal_motions = [m for m in program.motions if m.role == MotionRole.NORMAL]

        derived_prefix: list[RobotProgramMotion] = []
        derived_suffix: list[RobotProgramMotion] = []

        effective_base = self._compute_effective_base()

        if settings.home_enabled:
            home_joints = self.robot_model.get_home_position()
            home_target = RobotProgramTarget(
                target_type=RobotProgramTargetType.JOINT,
                joint_angles=home_joints,
            )
            home_start = RobotProgramMotion(
                mode=RobotProgramMotionMode.PTP,
                target=home_target,
                line_number=0,
                source="; HOME",
                base_pose=effective_base,
                role=MotionRole.HOME_START,
            )
            home_end = replace(home_start, role=MotionRole.HOME_END)
            derived_prefix.append(home_start)
            derived_suffix.append(home_end)

        first_cartesian = next(
            (m for m in normal_motions if m.target.target_type == RobotProgramTargetType.CARTESIAN),
            None,
        )
        last_cartesian = next(
            (m for m in reversed(normal_motions) if m.target.target_type == RobotProgramTargetType.CARTESIAN),
            None,
        )

        if settings.approach.enabled and first_cartesian is not None:
            approach_pose = self._compute_offset_pose(
                first_cartesian.target.cartesian_pose,
                first_cartesian.tool_pose,
                first_cartesian.base_pose,
                settings.approach.axis_ref,
                settings.approach.distance_mm,
            )
            approach_target = RobotProgramTarget(
                target_type=RobotProgramTargetType.CARTESIAN,
                cartesian_pose=approach_pose,
            )
            approach_motion = replace(
                first_cartesian,
                mode=RobotProgramMotionMode.LINEAR,
                target=approach_target,
                cp_speed_mps=settings.approach.speed_mps,
                role=MotionRole.APPROACH,
                line_number=0,
                source="; APPROACH",
            )
            derived_prefix.append(approach_motion)

        if settings.retract.enabled and last_cartesian is not None:
            retract_pose = self._compute_offset_pose(
                last_cartesian.target.cartesian_pose,
                last_cartesian.tool_pose,
                last_cartesian.base_pose,
                settings.retract.axis_ref,
                settings.retract.distance_mm,
            )
            retract_target = RobotProgramTarget(
                target_type=RobotProgramTargetType.CARTESIAN,
                cartesian_pose=retract_pose,
            )
            retract_motion = replace(
                last_cartesian,
                mode=RobotProgramMotionMode.LINEAR,
                target=retract_target,
                cp_speed_mps=settings.retract.speed_mps,
                role=MotionRole.RETRACT,
                line_number=0,
                source="; RETRACT",
            )
            derived_suffix.insert(0, retract_motion)

        all_motions = derived_prefix + normal_motions + derived_suffix
        return replace(program, motions=all_motions)

    def _compute_offset_pose(
        self,
        target_pose: Pose6,
        tool_pose: Pose6,
        base_pose: Pose6,
        axis_ref: ApproachAxisRef,
        distance_mm: float,
    ) -> Pose6:
        """Calcule une pose décalée de distance_mm dans la direction axis_ref."""
        import utils.math_utils as math_utils

        # Axe de décalage en repère robot
        if axis_ref == ApproachAxisRef.TOOL_Z:
            # Z outil au point cible
            T_base = math_utils.pose_zyx_to_matrix(base_pose)
            T_target_in_prog = math_utils.pose_zyx_to_matrix(target_pose)
            T_target_in_robot = T_base @ T_target_in_prog
            direction = T_target_in_robot[:3, 2]
        elif axis_ref == ApproachAxisRef.PIECE_X:
            T_base = math_utils.pose_zyx_to_matrix(base_pose)
            direction = T_base[:3, 0]
        elif axis_ref == ApproachAxisRef.PIECE_Y:
            T_base = math_utils.pose_zyx_to_matrix(base_pose)
            direction = T_base[:3, 1]
        else:  # PIECE_Z
            T_base = math_utils.pose_zyx_to_matrix(base_pose)
            direction = T_base[:3, 2]

        norm = float(np.linalg.norm(direction))
        if norm < 1e-9:
            return target_pose

        direction = direction / norm

        # Décalage en repère programme
        T_base = math_utils.pose_zyx_to_matrix(base_pose)
        T_base_inv = math_utils.invert_homogeneous_transform(T_base)
        offset_robot = direction * distance_mm
        offset_prog = T_base_inv[:3, :3] @ offset_robot

        return Pose6(
            target_pose.x + offset_prog[0],
            target_pose.y + offset_prog[1],
            target_pose.z + offset_prog[2],
            target_pose.a,
            target_pose.b,
            target_pose.c,
        )

