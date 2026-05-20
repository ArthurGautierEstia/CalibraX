from __future__ import annotations


from bisect import bisect_left
from dataclasses import dataclass, replace
from pathlib import Path
import time
from PyQt6.QtCore import QTimer, Qt
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

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.types import JointAngles6, Pose6
from models.workspace_model import WorkspaceModel
from utils.program_simulator import ProgramSimulator
from utils.mgi import RobotTool
from utils.robot_program_kuka import export_kuka_src_program, load_kuka_src_program
from widgets.program_view.program_base_dialog import ProgramBaseDialog
from utils.trajectory_keypoint_utils import resolve_keypoint_xyz
from widgets.program_view.program_target_dialog import ProgramTargetDialog
from widgets.program_view.program_keypoints_widget import ProgramKeypointsWidget
from views.program_view import ProgramView





@dataclass(frozen=True)

class _ProgramTargetRef:

    motion_index: int

    is_via_target: bool





class ProgramController:

    DEFAULT_PROGRAMS_DIR = Path(__file__).resolve().parents[1] / "user_data" / "programs"



    NOMINAL_PATH_COLORS = {

        "PTP": (1.0, 0.55, 0.0, 1.0),

        "LINEAR": (1.0, 0.55, 0.0, 1.0),

        "CIRCULAR": (1.0, 0.55, 0.0, 1.0),

    }

    MEASURED_PATH_COLORS = {

        "PTP": (0.0, 0.35, 1.0, 1.0),

        "LINEAR": (0.0, 0.35, 1.0, 1.0),

        "CIRCULAR": (0.0, 0.35, 1.0, 1.0),

    }

    COMPENSATED_PATH_COLORS = {

        "PTP": (0.0, 0.85, 0.35, 1.0),

        "LINEAR": (0.0, 0.85, 0.35, 1.0),

        "CIRCULAR": (0.0, 0.85, 0.35, 1.0),

    }



    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        program_view: ProgramView,
        viewer3d_controller: Viewer3DController,
    ) -> None:

        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.program_view = program_view
        self.viewer3d_controller = viewer3d_controller
        self.header_widget = self.program_view.get_header_widget()
        self.config_widget: ProgramKeypointsWidget = self.program_view.get_config_widget()
        self.actions_widget = self.program_view.get_actions_widget()
        self.graphs_widget = self.program_view.get_graphs_widget()
        self.playback_widget = self.program_view.get_playback_widget()
        self.program_simulator = ProgramSimulator(self.robot_model, self.tool_model)
        self.current_program: RobotProgram | None = None
        self.current_result: ProgramSimulationResult | None = None

        # NOUVEAU : Stockage des 4 versions de resultats
        self._nominal_cartesian_result: ProgramSimulationResult | None = None
        self._nominal_articular_result: ProgramSimulationResult | None = None
        self._compensated_cartesian_result: ProgramSimulationResult | None = None
        self._compensated_articular_result: ProgramSimulationResult | None = None

        # NOUVEAU : Programmes convertis
        self._articular_program: RobotProgram | None = None
        self._cartesian_program: RobotProgram | None = None
        self._compensated_cartesian_program: RobotProgram | None = None
        self._compensated_articular_program: RobotProgram | None = None

        # NOUVEAU : Flag de calcul de compensation
        self._compensation_computed: bool = False
        self._display_keypoints: list[TrajectoryKeypoint] = []
        self._display_keypoint_tools: list[RobotTool] = []
        self._display_target_refs: list[_ProgramTargetRef] = []
        self._selected_keypoint_index: int | None = None
        self._tool_source: str = "CURRENT"  # CURRENT or PROGRAM
        self._saved_robot_tool: RobotTool | None = None  # Sauvegarde du tool robot original
        self._nominal_segments_cache: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        self._measured_segments_cache: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        self._compensated_segments_cache: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        self._current_time_s = 0.0
        self._playback_index = 0
        self._playback_sample_times: list[float] = []
        self._playback_timer = QTimer()
        self._playback_timer.setSingleShot(False)
        self._playback_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._playback_timer.timeout.connect(self._on_playback_tick)
        self._playback_wall_start_s: float | None = None
        self._playback_sim_start_s = 0.0
        self._setup_connections()
        self._refresh_view()



    def _setup_connections(self) -> None:
        self.header_widget.load_program_requested.connect(self._on_load_program_requested)
        self.header_widget.clear_requested.connect(self._on_clear_requested)
        self.actions_widget.recompute_requested.connect(self._on_recompute_requested)
        self.actions_widget.export_requested.connect(self._on_export_requested)
        self.playback_widget.play_requested.connect(self._on_play_requested)
        self.playback_widget.pause_requested.connect(self._on_pause_requested)
        self.playback_widget.stop_requested.connect(self._on_stop_requested)
        self.playback_widget.time_value_changed.connect(self._on_time_value_changed)
        self.actions_widget.trajectory_visibility_changed.connect(self._on_trajectory_visibility_changed)
        self.actions_widget.compute_compensation_requested.connect(self._on_compute_compensation_requested)
        self.config_widget.goToRequested.connect(self._on_go_to_requested)
        self.config_widget.keypointSelectionChanged.connect(self._on_keypoint_selection_changed)
        self.config_widget.edit_requested.connect(self._on_program_edit_requested)
        self.config_widget.editProgramBaseRequested.connect(self._on_edit_program_base_requested)
        self.config_widget.keypoints_changed.connect(self._on_program_keypoints_changed)
        self.config_widget.cartesianDisplayFrameChanged.connect(self._on_program_display_frame_changed)
        self.config_widget.toolSourceChanged.connect(self._on_tool_source_changed)
        # NOUVEAU : Connexions pour Mode et Cibles
        self.config_widget.motionModeChanged.connect(self._on_motion_mode_changed)
        self.config_widget.targetModeChanged.connect(self._on_target_mode_changed)
        self.graphs_widget.error_graph_visibility_changed.connect(self._on_error_graph_visibility_changed)
        self.workspace_model.workspace_changed.connect(self._refresh_view)



    def _on_load_program_requested(self) -> None:

        self.DEFAULT_PROGRAMS_DIR.mkdir(parents=True, exist_ok=True)
        file_path, _selected_filter = QFileDialog.getOpenFileName(
            self.program_view,
            "Importer programme robot",
            str(self.DEFAULT_PROGRAMS_DIR),
            "Programmes KUKA (*.src);;Tous les fichiers (*.*)",
        )

        if not file_path:
            return

        self.viewer3d_controller.begin_loading_feedback("Chargement programme KRL ...")
        QTimer.singleShot(0, lambda: self._load_program_from_path(file_path))



    def _load_program_from_path(self, file_path: str) -> None:
        try:
            if Path(file_path).suffix.lower() != ".src":
                QMessageBox.warning(
                    self.program_view,
                    "Programme robot",
                    "Seuls les programmes KUKA .src sont supportes dans cette premiere version.",
                )
                return
            self.current_program = load_kuka_src_program(file_path)
            self._tool_source = "PROGRAM"
            self.config_widget.set_tool_source("PROGRAM", emit_signal=False)
            self.config_widget.set_cartesian_display_frame(ReferenceFrame.PROGRAM.value, emit_signal=False)
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
            self.actions_widget.set_compensated_checkbox_enabled(False)
            self.current_result = None
            self._display_keypoints = []
            self._display_keypoint_tools = []
            self._display_target_refs = []
            self._nominal_segments_cache = []
            self._measured_segments_cache = []
            self._compensated_segments_cache = []
            self._refresh_view()
            return

        simulation_program = self._get_simulation_program()
        if simulation_program is None:
            self._nominal_cartesian_result = None
            self._nominal_articular_result = None
            self.current_result = None
            self._display_keypoints = []
            self._display_keypoint_tools = []
            self._display_target_refs = []
            self._refresh_view()
            return

        # Detection type + mode par defaut
        program_type = self._detect_program_type()

        # Calcul des 2 versions nominales
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

        # Reinitialisation compensation
        self._compensated_cartesian_result = None
        self._compensated_articular_result = None
        self._compensated_cartesian_program = None
        self._compensated_articular_program = None
        self._compensation_computed = False
        self.actions_widget.set_compensated_checkbox_enabled(False)

        # Modes actifs apres recalcul
        if reset_modes:
            active_motion_mode = program_type
            active_target_mode = "THEORETICAL"
            self.config_widget.set_motion_mode(active_motion_mode, emit_signal=False)
            self.config_widget.set_target_mode(active_target_mode, emit_signal=False)
        else:
            active_motion_mode = self.config_widget.get_motion_mode()
            active_target_mode = self.config_widget.get_target_mode()

        # Mise a jour current_result
        self._update_current_result_from_modes(active_target_mode, active_motion_mode)

        # Build display keypoints
        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = (
            self._build_display_keypoints_for_mode(active_motion_mode, active_target_mode)
        )

        # Build caches — Lot D : une seule passe pour nominal + measured
        self._nominal_segments_cache, self._measured_segments_cache = self._build_nominal_and_measured_segments(
            self._get_samples_for_modes("THEORETICAL", active_motion_mode),
            self.NOMINAL_PATH_COLORS,
            self.MEASURED_PATH_COLORS,
        )
        self._compensated_segments_cache = []

        self.actions_widget.set_compensation_enabled(True)

        self._refresh_view()


    def _ensure_compensation_result(self) -> None:
        if self.current_program is None or self.current_result is None:

            return

        if self.current_result.compensation_computed:

            return

        compensated_result = self.program_simulator.simulate_program(self._get_simulation_program(), include_compensation=True)

        self.current_result = replace(

            compensated_result,

            nominal_samples=self.current_result.nominal_samples,

            warnings=list(dict.fromkeys([*self.current_result.warnings, *compensated_result.warnings])),

        )

        self._compensated_segments_cache = self._build_segments(

            self._selected_compensated_samples(),

            self.COMPENSATED_PATH_COLORS,

            "measured",

        )



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



    def _refresh_display_only(self) -> None:

        if self.current_result is not None and self.config_widget.get_target_mode() == "COMPENSATED":
            self._ensure_compensation_result()

        if self.current_result is not None:
            self._compensated_segments_cache = self._build_segments(
                self._selected_compensated_samples(),
                self.COMPENSATED_PATH_COLORS,
                "measured",
            )

        self._refresh_status()
        self._refresh_viewer_segments()
        self._refresh_error_graph()
        self._refresh_timeline()
        self._refresh_keypoint_table()



    def _refresh_program_info(self) -> None:

        if self.current_program is None:
            self.header_widget.set_program_info("", 0)
            self.header_widget.set_log_lines([])
            self.actions_widget.set_export_enabled(False)
            self.config_widget.set_program_base_edit_enabled(False)
            return

        log_lines = list(self.current_program.warnings)

        if self.current_result is not None:
            log_lines.extend(self.current_result.warnings)

        self.header_widget.set_program_info(
            self.current_program.source_path,
            len(self.current_program.motions),
        )

        self.header_widget.set_log_lines(log_lines)
        self.actions_widget.set_export_enabled(self._selected_compensated_program() is not None)
        self.config_widget.set_program_base_edit_enabled(True)



    def _refresh_keypoint_table(self) -> None:

        if self.current_program is None:
            self.config_widget.set_keypoints([])
            return

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

        segments: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        if self.actions_widget.is_theoretical_visible():
            segments.extend(self._nominal_segments_cache)
        if self.actions_widget.is_measured_visible():
            segments.extend(self._measured_segments_cache)

        if self._compensation_computed and self.actions_widget.is_compensated_visible():
            segments.extend(self._compensated_segments_cache)

        if segments:
            self.viewer3d_controller.set_trajectory_path_segments(self._downsample_segments(segments))

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
            self.playback_widget.set_time_range(0.0, 0.0)
            self.playback_widget.set_playback_enabled(False)
            self._apply_time_value(0.0)
            return

        end_time = float(samples[-1].time_s)

        self.playback_widget.set_time_range(0.0, end_time)
        self.playback_widget.set_playback_enabled(True)

        self._apply_time_value(min(self._current_time_s, end_time))



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

        if self.config_widget.get_target_mode() == "COMPENSATED":

            return self._selected_compensated_samples()

        return self.current_result.nominal_samples



    def _apply_time_value(self, time_s: float) -> None:

        samples = self._playback_samples()

        self._current_time_s = max(0.0, float(time_s))

        self.playback_widget.set_time_value(self._current_time_s)

        if not samples:

            return

        sample_index = self._sample_index_at_time(self._current_time_s)

        sample = samples[sample_index]

        self.robot_model.set_joints(sample.joints_deg.to_list())



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



    def _stop_playback(self) -> None:

        self._playback_wall_start_s = None

        self._playback_timer.stop()



    def _on_play_requested(self) -> None:

        samples = self._playback_samples()

        if not samples:

            return

        self._playback_index = self._sample_index_at_time(self._current_time_s)

        self._playback_sim_start_s = float(self._current_time_s)

        self._playback_wall_start_s = time.perf_counter()

        self._playback_timer.start(20)

        self._on_playback_tick()



    def _on_pause_requested(self) -> None:

        self._stop_playback()



    def _on_stop_requested(self) -> None:

        self._stop_playback()

        self._playback_index = 0

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

        target_time_s = self._playback_sim_start_s + elapsed_s

        end_time_s = float(samples[-1].time_s)

        if target_time_s >= end_time_s:

            self._stop_playback()

            self._apply_time_value(end_time_s)

            return

        self._playback_index = self._sample_index_at_time(target_time_s)

        self._apply_time_value(target_time_s)



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

            export_kuka_src_program(file_path, self.current_program.source_text, program.motions)

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

            self.robot_model.set_joints(keypoint.joint_target[:6])

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

        """Retourne le programme à simuler, avec les tools adaptés selon le mode sélectionné."""

        if self.current_program is None:

            return None

        

        if self._tool_source == "CURRENT":

            # En mode Tool Robot : toutes les motions utilisent le tool robot actuel

            current_tool = self.tool_model.get_tool()

            tool_pose = self.program_simulator._tool_to_pose(current_tool)

            if tool_pose is None:

                return self.current_program

            

            # Créer un programme temporaire avec le tool robot pour toutes les motions

            motions_with_robot_tool = []

            for motion in self.current_program.motions:

                updated_motion = replace(motion, tool_pose=tool_pose)

                motions_with_robot_tool.append(updated_motion)

            return replace(self.current_program, motions=motions_with_robot_tool)

        else:

            # En mode Tool Programme : utiliser le programme original avec ses tools

            return self.current_program



    def _apply_program_tool(self) -> None:

        """Applique le tool de la première motion du programme au tool_model."""

        if self.current_program is None or self._saved_robot_tool is not None:

            return

        # Sauvegarde du tool actuel

        self._saved_robot_tool = self.tool_model.get_tool()

        # Applique le tool de la première motion

        for motion in self.current_program.motions:

            program_tool = self.program_simulator._tool_from_pose(motion.tool_pose)

            if program_tool is not None:

                self.tool_model.set_tool(program_tool)

                break



    def _restore_robot_tool(self) -> None:

        """Restaure le tool robot original."""

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

            self._refresh_viewer_segments()



    def _on_program_edit_requested(self) -> None:

        row = self._selected_keypoint_index

        if row is None:

            return

        self._edit_program_target_at_row(row)

    def _on_edit_program_base_requested(self) -> None:

        program_base_pose = self._program_base_pose()

        if program_base_pose is None:

            return

        dialog = ProgramBaseDialog(program_base_pose, self.program_view)
        dialog.base_pose_preview_changed.connect(self._on_program_base_preview_changed)

        if dialog.exec() != dialog.DialogCode.Accepted:
            self._update_program_base_preview(program_base_pose)

            return

        updated_base_pose = dialog.get_base_pose()

        if updated_base_pose == program_base_pose:

            return

        self._invalidate_simulation_results()
        self._refresh_status()

    def _on_program_base_preview_changed(self, base_pose: Pose6) -> None:

        self._update_program_base_preview(base_pose)



    def _on_program_table_item_double_clicked(self, item) -> None:

        row = item.row()

        if row < 0:

            return

        self._edit_program_target_at_row(row)



    def _edit_program_target_at_row(self, row: int) -> None:

        if self.current_program is None or row < 0 or row >= len(self._display_target_refs):

            return

        target_ref = self._display_target_refs[row]

        motion = self.current_program.motions[target_ref.motion_index]

        target = motion.via_target if target_ref.is_via_target else motion.target

        if target is None:

            return

        dialog = ProgramTargetDialog(motion, target, target_ref.is_via_target, self.program_view)

        if dialog.exec() != dialog.DialogCode.Accepted:

            return

        updated_target = dialog.get_target()

        updated_motion = replace(motion, via_target=updated_target) if target_ref.is_via_target else replace(motion, target=updated_target)

        updated_motions = list(self.current_program.motions)

        updated_motions[target_ref.motion_index] = updated_motion

        self.current_program = replace(self.current_program, motions=updated_motions)

        self._recompute_current_program()

        self.config_widget.select_row(row)



    def _on_program_keypoints_changed(self, _keypoints: list[TrajectoryKeypoint]) -> None:

        return



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
        self._nominal_segments_cache = []
        self._measured_segments_cache = []
        self._compensated_segments_cache = []
        self.actions_widget.set_compensated_checkbox_enabled(False)
        self.actions_widget.set_compensation_enabled(True)
        self.viewer3d_controller.clear_trajectory_path()

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



    def _build_display_keypoints(self) -> tuple[list[TrajectoryKeypoint], list[RobotTool], list[_ProgramTargetRef]]:

        if self.current_program is None:

            return [], [], []

        keypoints: list[TrajectoryKeypoint] = []

        keypoint_tools: list[RobotTool] = []

        target_refs: list[_ProgramTargetRef] = []

        display_frame = ReferenceFrame.from_value(

            self.config_widget.cartesian_display_frame_combo.currentData(),

            ReferenceFrame.PROGRAM,

        )

        # Toutes les motions utilisent le tool actuellement configuré dans tool_model

        current_tool = self.tool_model.get_tool()

        for motion_index, motion in enumerate(self.current_program.motions):

            # Utilise le tool courant (robot ou programme selon la sélection)

            motion_tool = current_tool

            if motion.mode.value == "CIRCULAR" and motion.via_target is not None:

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

                joint_target=target.joint_angles.to_list(),

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



    



    # =========================================================================
    # NOUVELLES METHODES POUR GESTION MODE CARTESIEN/ARTICULAIRE
    # =========================================================================

    def _detect_program_type(self) -> str:
        """Detecte si le programme est CARTESIEN ou ARTICULAIRE."""
        if self.current_program is None:
            return "CARTESIAN"
        has_cartesian = any(
            motion.target.target_type == RobotProgramTargetType.CARTESIAN
            for motion in self.current_program.motions
        )
        return "CARTESIAN" if has_cartesian else "ARTICULAR"

    def _build_articular_program(self, source_program: RobotProgram | None = None) -> RobotProgram | None:
        """Convertit le programme en version purement articulaire (tout en PTP avec cibles JOINT)."""
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

        return replace(self.current_program, motions=new_motions)

    def _build_cartesian_program(self, source_program: RobotProgram | None = None) -> RobotProgram | None:
        """Convertit le programme en version cartesienne (cibles CARTESIAN)."""
        program = source_program or self.current_program
        if program is None:
            return None

        new_motions: list[RobotProgramMotion] = []

        for motion in program.motions:
            motion_tool = self.program_simulator._tool_from_pose(motion.tool_pose)

            if motion.target.target_type == RobotProgramTargetType.CARTESIAN:
                new_target = motion.target
            else:
                # JOINT -> CARTESIAN via FK
                fk_result = self.robot_model.compute_fk_joints(
                    motion.target.joint_angles.to_list(), tool=motion_tool
                )
                if fk_result is None:
                    new_motions.append(motion)
                    continue
                new_pose = fk_result.dh_pose
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
                        new_via_target = RobotProgramTarget(
                            target_type=RobotProgramTargetType.CARTESIAN,
                            cartesian_pose=via_fk_result.dh_pose
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

        return replace(self.current_program, motions=new_motions)

    def _compute_compensation(self) -> None:
        """Calcule les 2 versions compensees."""
        if self.current_program is None:
            self._compensation_computed = False
            return

        measured_dh = self.program_simulator._normalized_measured_dh_table()
        if measured_dh is None:
            self._compensation_computed = False
            return

        # Calcul compensation cartesienne
        cartesian_prog = self._get_program_for_mode("CARTESIAN") or self.current_program
        cartesian_comp_program = self.program_simulator._build_compensated_program(
            cartesian_prog, ProgramCompensationOutputMode.CARTESIAN, measured_dh
        )
        if cartesian_comp_program:
            self._compensated_cartesian_program = cartesian_comp_program
            self._compensated_cartesian_result = self.program_simulator.simulate_program(
                cartesian_comp_program, include_compensation=False
            )

        # Calcul compensation articulaire (toujours depuis les cibles cartesiennes)
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

    def _get_samples_for_modes(self, target_mode: str, motion_mode: str) -> list[ProgramSimulationSample]:
        """Retourne les samples selon target_mode et motion_mode."""
        if target_mode == "THEORETICAL":
            if motion_mode == "ARTICULAR" and self._nominal_articular_result:
                return self._nominal_articular_result.nominal_samples
            elif self._nominal_cartesian_result:
                return self._nominal_cartesian_result.nominal_samples
        else:
            if motion_mode == "ARTICULAR" and self._compensated_articular_result:
                return self._compensated_articular_result.nominal_samples
            elif self._compensated_cartesian_result:
                return self._compensated_cartesian_result.nominal_samples
        return []

    def _get_program_for_mode(self, motion_mode: str) -> RobotProgram | None:
        """Retourne le programme selon motion_mode."""
        if motion_mode == "ARTICULAR":
            return self._articular_program or self.current_program
        return self._cartesian_program or self.current_program

    def _update_current_result_from_modes(self, target_mode: str, motion_mode: str) -> None:
        """Met a jour current_result selon les modes."""
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
        """Collecte tous les warnings des 4 resultats."""
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
        """Handler pour changement de mode CARTESIEN/ARTICULAIRE."""
        if self.current_program is None:
            return

        target_mode = self.config_widget.get_target_mode()
        self._update_current_result_from_modes(target_mode, motion_mode)

        # Rebuild display keypoints selon le nouveau mode et le target mode courant
        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = (
            self._build_display_keypoints_for_mode(motion_mode, target_mode)
        )

        # Mise a jour des caches — Lot D : une seule passe pour nominal + measured
        self._nominal_segments_cache, self._measured_segments_cache = self._build_nominal_and_measured_segments(
            self._get_samples_for_modes("THEORETICAL", motion_mode),
            self.NOMINAL_PATH_COLORS,
            self.MEASURED_PATH_COLORS,
        )
        self._compensated_segments_cache = self._build_segments(
            self._get_samples_for_modes("COMPENSATED", motion_mode),
            self.COMPENSATED_PATH_COLORS,
            "measured",
        ) if self._compensation_computed else []

        self._refresh_view()

    def _on_target_mode_changed(self, target_mode: str) -> None:
        """Handler pour changement de cible THEORIQUE/COMPENSE."""
        if self.current_program is None:
            return

        motion_mode = self.config_widget.get_motion_mode()
        self._update_current_result_from_modes(target_mode, motion_mode)

        # Rebuild keypoints selon le nouveau target_mode (theorique vs compense)
        self._display_keypoints, self._display_keypoint_tools, self._display_target_refs = (
            self._build_display_keypoints_for_mode(motion_mode, target_mode)
        )

        # Rafraichir uniquement ce qui depend de target_mode
        self._refresh_status()
        self._refresh_keypoint_table()
        self._refresh_viewer_segments()
        self._refresh_error_graph()
        self._refresh_timeline()

    def _on_compute_compensation_requested(self) -> None:
        """Handler pour le bouton Calculer compensation."""
        if self.current_program is None:
            return

        self._compute_compensation()
        if self._compensation_computed:
            self.config_widget.set_target_mode_enabled(True)
            self.actions_widget.set_compensated_checkbox_enabled(True)

        # Rafraichir pour afficher la compensation
        motion_mode = self.config_widget.get_motion_mode()
        self._compensated_segments_cache = self._build_segments(
            self._get_samples_for_modes("COMPENSATED", motion_mode),
            self.COMPENSATED_PATH_COLORS,
            "measured",
        )
        self._refresh_viewer_segments()
        self._refresh_error_graph()

    def _get_program_for_target_and_motion_mode(self, target_mode: str, motion_mode: str) -> RobotProgram | None:
        """Retourne le programme selon target_mode et motion_mode."""
        if target_mode == "COMPENSATED" and self._compensation_computed:
            if motion_mode == "ARTICULAR":
                return self._compensated_articular_program or self._compensated_cartesian_program
            return self._compensated_cartesian_program or self._compensated_articular_program
        return self._get_program_for_mode(motion_mode)

    def _build_display_keypoints_for_mode(self, motion_mode: str, target_mode: str | None = None) -> tuple[list[TrajectoryKeypoint], list[RobotTool], list[_ProgramTargetRef]]:
        """Build keypoints pour un mode specifique."""
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

            if motion.mode.value == "CIRCULAR" and motion.via_target is not None:
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
        """Build keypoints selon le mode courant."""
        motion_mode = self.config_widget.get_motion_mode()
        return self._build_display_keypoints_for_mode(motion_mode)


    @staticmethod
    def _build_nominal_and_measured_segments(
        samples: list[ProgramSimulationSample],
        nominal_colors: dict[str, tuple[float, float, float, float]],
        measured_colors: dict[str, tuple[float, float, float, float]],
    ) -> tuple[
        list[tuple[list[list[float]], tuple[float, float, float, float]]],
        list[tuple[list[list[float]], tuple[float, float, float, float]]],
    ]:
        """Lot D : construit les segments nominal et measured en une seule passe sur les samples."""
        nominal_segments: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        measured_segments: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []

        nom_points: list[list[float]] = []
        meas_points: list[list[float]] = []
        current_key: tuple[str, int] | None = None

        for sample in samples:
            nom_pose = sample.nominal_pose_base
            meas_pose = sample.measured_pose_base
            if nom_pose is None and meas_pose is None:
                continue

            motion_key = (sample.motion_mode.value, int(sample.source_line))

            if current_key is not None and motion_key != current_key:
                if len(nom_points) >= 2:
                    nominal_segments.append((nom_points, nominal_colors.get(current_key[0], (1.0, 0.55, 0.0, 1.0))))
                    nom_points = [nom_points[-1]]
                if len(meas_points) >= 2:
                    measured_segments.append((meas_points, measured_colors.get(current_key[0], (0.0, 0.35, 1.0, 1.0))))
                    meas_points = [meas_points[-1]]

            current_key = motion_key
            if nom_pose is not None:
                nom_points.append([nom_pose.x, nom_pose.y, nom_pose.z])
            if meas_pose is not None:
                meas_points.append([meas_pose.x, meas_pose.y, meas_pose.z])

        if current_key is not None:
            if len(nom_points) >= 2:
                nominal_segments.append((nom_points, nominal_colors.get(current_key[0], (1.0, 0.55, 0.0, 1.0))))
            if len(meas_points) >= 2:
                measured_segments.append((meas_points, measured_colors.get(current_key[0], (0.0, 0.35, 1.0, 1.0))))

        return nominal_segments, measured_segments

    def _build_segments(
        self,
        samples: list[ProgramSimulationSample],
        colors: dict[str, tuple[float, float, float, float]],
        pose_kind: str

    ) -> list[tuple[list[list[float]], tuple[float, float, float, float]]]:

        segments: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []

        current_points: list[list[float]] = []

        current_key: tuple[str, int] | None = None

        for sample in samples:

            pose = sample.nominal_pose_base if pose_kind == "nominal" else sample.measured_pose_base

            if pose is None:

                continue

            motion_key = (sample.motion_mode.value, int(sample.source_line))

            if current_key is not None and motion_key != current_key and len(current_points) >= 2:

                segments.append((current_points, colors.get(current_key[0], (1.0, 0.55, 0.0, 1.0))))

                current_points = [current_points[-1]]

            current_key = motion_key

            current_points.append([pose.x, pose.y, pose.z])

        if current_key is not None and len(current_points) >= 2:

            segments.append((current_points, colors.get(current_key[0], (1.0, 0.55, 0.0, 1.0))))

        return segments

