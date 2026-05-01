import csv
from bisect import bisect_left
from pathlib import Path
import time
import os

from PyQt6.QtCore import QObject, QThread, QTimer, Qt
from PyQt6.QtWidgets import QFileDialog, QMessageBox

from models.robot_model import RobotModel
from models.tool_model import ToolModel
from models.types import Pose6, XYZ3
from models.workspace_model import WorkspaceModel
from models.trajectory_result import (
    TrajectoryDynamicViolationSeverity,
    TrajectoryComputationStatus,
    TrajectoryResult,
    TrajectorySample,
    TrajectorySampleErrorCode,
    TrajectorySegment,
)
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from models.reference_frame import ReferenceFrame
from utils.trajectory_builder import TrajectoryBuilder
from utils.trajectory_collision_analyzer import (
    TrajectoryCollisionAnalysisResult,
    TrajectoryCollisionCancelToken,
    TrajectoryCollisionContext,
    TrajectoryCollisionWorker,
    apply_trajectory_collision_result,
)
from utils.trajectory_keypoint_utils import resolve_keypoint_xyz
from utils.trajectory_status import build_trajectory_issue_messages, build_trajectory_warning_messages
from utils.reference_frame_utils import (
    convert_pose_from_base_frame,
    convert_pose_to_base_frame,
    twist_base_to_world,
)
from views.trajectory_view import TrajectoryView
from controllers.viewer3d_controller import Viewer3DController
import utils.math_utils as math_utils


TangentSegment = tuple[XYZ3, XYZ3]


class TrajectoryController(QObject):
    _PATH_COLOR_LIN_CUBIC = (1.0, 0.84, 0.1, 0.85)
    _PATH_COLOR_PTP = (0.25, 0.65, 1.0, 0.9)
    _PATH_COLOR_ERROR = (1.0, 0.2, 0.2, 0.95)

    def __init__(
        self,
        robot_model: RobotModel,
        tool_model: ToolModel,
        workspace_model: WorkspaceModel,
        trajectory_view: TrajectoryView,
        viewer3d_controller: Viewer3DController,
        parent: QObject = None,
    ):
        super().__init__(parent)

        self.robot_model = robot_model
        self.tool_model = tool_model
        self.workspace_model = workspace_model
        self.trajectory_view = trajectory_view
        self.viewer3d_controller = viewer3d_controller
        self.config_widget = self.trajectory_view.get_config_widget()
        self.actions_widget = self.trajectory_view.get_actions_widget()
        self.graphs_widget = self.trajectory_view.get_graphs_widget()

        self.trajectory_builder = TrajectoryBuilder(
            self.robot_model,
            self.tool_model,
            self.workspace_model,
            smooth_time_enabled=self.config_widget.is_time_smoothing_enabled(),
            bezier_degree=self.config_widget.get_bezier_degree(),
            jerk_check_enabled=self.config_widget.is_jerk_check_enabled(),
        )
        self.current_trajectory = TrajectoryResult()
        self.current_samples: list[TrajectorySample] = []
        self.current_sample_times: list[float] = []
        self._displayed_keypoints: list[TrajectoryKeypoint] = []
        self._current_time_s = 0.0
        self._playback_index = 0
        self._is_playing = False
        self._is_keypoint_preview_active = False
        self._selected_keypoint_index: int | None = None
        self._editing_keypoint_index: int | None = None
        self._collision_job_sequence = 0
        self._active_collision_job_id: int | None = None
        self._collision_threads: list[QThread] = []
        self._collision_workers: list[TrajectoryCollisionWorker] = []
        self._trajectory_generation_sequence = 0
        self._collision_job_traj_ids: dict[int, int] = {}
        self._collision_job_total_start_s: dict[int, float] = {}
        self._collision_job_detect_start_s: dict[int, float] = {}
        self._playback_wall_start_s: float | None = None
        self._playback_sim_start_s = 0.0
        self._playback_timer = QTimer(self)
        self._playback_timer.setSingleShot(False)
        self._playback_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._playback_timer.timeout.connect(self._on_playback_tick)

        self._setup_connections()
        self._reset_trajectory_visuals()

    def _setup_connections(self) -> None:
        self.config_widget.showRobotGhostRequested.connect(self._on_show_robot_ghost_requested)
        self.config_widget.hideRobotGhostRequested.connect(self._on_hide_robot_ghost_requested)
        self.config_widget.updateRobotGhostRequested.connect(self._on_update_robot_ghost_requested)
        self.config_widget.keypointSelectionChanged.connect(self._on_keypoint_selection_changed)
        self.config_widget.goToRequested.connect(self._on_go_to_requested)
        self.config_widget.editingSessionStarted.connect(self._on_editing_session_started)
        self.config_widget.editingSessionFinished.connect(self._on_editing_session_finished)
        self.config_widget.trajectoryPreviewRequested.connect(self._on_trajectory_preview_requested)
        self.config_widget.trajectoryPreviewFinished.connect(self._on_trajectory_preview_finished)
        self.config_widget.keypoints_changed.connect(self._on_keypoints_changed)
        self.config_widget.timeSmoothingChanged.connect(self._on_time_smoothing_changed)
        self.config_widget.jerkCheckChanged.connect(self._on_jerk_check_changed)
        self.config_widget.bezierDegreeChanged.connect(self._on_bezier_degree_changed)
        self.config_widget.cartesianDisplayFrameChanged.connect(self._on_cartesian_display_frame_changed)
        self.actions_widget.compute_requested.connect(self._on_compute_requested)
        self.actions_widget.export_trajectory_requested.connect(self._on_export_trajectory_requested)
        self.actions_widget.home_position_requested.connect(self._on_home_position_requested)
        self.actions_widget.play_requested.connect(self._on_play_requested)
        self.actions_widget.pause_requested.connect(self._on_pause_requested)
        self.actions_widget.stop_requested.connect(self._on_stop_requested)
        self.actions_widget.time_value_changed.connect(self._on_time_value_changed)
        self.workspace_model.workspace_changed.connect(self._on_workspace_changed)

    def _on_show_robot_ghost_requested(self) -> None:
        self.viewer3d_controller.show_robot_ghost()

    def _on_hide_robot_ghost_requested(self) -> None:
        self.viewer3d_controller.hide_robot_ghost()

    def _on_update_robot_ghost_requested(self, payload: object) -> None:
        joints: list[float] = []
        corrected_matrices = None

        if isinstance(payload, dict):
            raw_joints = payload.get("joints", [])
            if isinstance(raw_joints, list):
                joints = [float(v) for v in raw_joints[:6]]
            maybe_matrices = payload.get("corrected_matrices")
            if isinstance(maybe_matrices, list):
                corrected_matrices = maybe_matrices
        elif isinstance(payload, list):
            joints = [float(v) for v in payload[:6]]

        if len(joints) < 6:
            self.viewer3d_controller.hide_robot_ghost()
            return

        if corrected_matrices is None:
            fk_result = self.robot_model.compute_fk_joints(joints, tool=self.tool_model.get_tool())
            if fk_result is None:
                self.viewer3d_controller.hide_robot_ghost()
                return
            _, corrected_matrices, _, _, _ = fk_result

        self.viewer3d_controller.update_robot_ghost_with_matrices(joints, corrected_matrices)

    def _on_keypoints_changed(self, _keypoints: list[TrajectoryKeypoint]) -> None:
        # During live dialog preview, the final recompute is triggered by
        # trajectoryPreviewFinished to avoid duplicate recomputations.
        if self._is_keypoint_preview_active:
            return
        self._recompute_trajectory()

    def _on_time_smoothing_changed(self, _enabled: bool) -> None:
        self.trajectory_builder.set_time_smoothing_enabled(self.config_widget.is_time_smoothing_enabled())
        if self._is_keypoint_preview_active:
            return
        self._recompute_trajectory()

    def _on_jerk_check_changed(self, _enabled: bool) -> None:
        self.trajectory_builder.set_jerk_check_enabled(self.config_widget.is_jerk_check_enabled())
        if self._is_keypoint_preview_active:
            return
        self._recompute_trajectory()

    def _on_bezier_degree_changed(self, _degree: str) -> None:
        self.trajectory_builder.set_bezier_degree(self.config_widget.get_bezier_degree())
        if self._is_keypoint_preview_active:
            return
        self._recompute_trajectory()

    def _on_keypoint_selection_changed(self, row: object) -> None:
        self._selected_keypoint_index = row if isinstance(row, int) and row >= 0 else None
        self._update_3d_keypoint_overlays()

    def _on_editing_session_started(self, row_index: int) -> None:
        self._editing_keypoint_index = row_index if row_index >= 0 else None
        self.actions_widget.set_editing_locked(True)
        self._update_3d_keypoint_overlays()

    def _on_editing_session_finished(self) -> None:
        self._editing_keypoint_index = None
        self.actions_widget.set_editing_locked(False)
        self._update_3d_keypoint_overlays()

    def _on_trajectory_preview_requested(self, keypoints: list[TrajectoryKeypoint]) -> None:
        self._is_keypoint_preview_active = True
        self._recompute_trajectory(keypoints)

    def _on_trajectory_preview_finished(self) -> None:
        if not self._is_keypoint_preview_active:
            return
        self._is_keypoint_preview_active = False
        self._recompute_trajectory()

    def _on_compute_requested(self) -> None:
        self._recompute_trajectory()

    def _on_cartesian_display_frame_changed(self, _frame: str) -> None:
        self._update_graphs()

    def _on_workspace_changed(self) -> None:
        self._update_graphs()
        self._update_3d_trajectory_path()
        self._update_3d_keypoint_overlays()

    def _on_home_position_requested(self) -> None:
        self._stop_playback()
        self.robot_model.go_to_home_position()

    def _on_go_to_requested(self, row: int) -> None:
        keypoints = self.config_widget.get_keypoints()
        if row < 0 or row >= len(keypoints):
            return

        self._stop_playback()
        keypoint = keypoints[row]

        if keypoint.target_type == KeypointTargetType.JOINT:
            self.robot_model.set_joints(keypoint.joint_target[:6])
            return

        target_pose = convert_pose_to_base_frame(
            keypoint.cartesian_target,
            keypoint.cartesian_frame,
            self.workspace_model.get_robot_base_transform_world(),
        )
        mgi_result = self.robot_model.compute_ik_target(target_pose, tool=self.tool_model.get_tool())
        best_solution = self.robot_model.get_best_mgi_solution(mgi_result)
        if best_solution is None:
            QMessageBox.warning(
                self.trajectory_view,
                "Aller a un keypoint",
                "Aucune solution MGI valide pour cette cible.",
            )
            return

        _, solution = best_solution
        self.robot_model.set_joints(solution.joints[:6])

    @staticmethod
    def _fmt_csv(value: float) -> str:
        return f"{float(value):.6f}"

    @staticmethod
    def _trajectory_export_header() -> list[str]:
        return [
            "statut",
            "config",
            "time",
            "j1",
            "j2",
            "j3",
            "j4",
            "j5",
            "j6",
            "dj1",
            "dj2",
            "dj3",
            "dj4",
            "dj5",
            "dj6",
            "ddj1",
            "ddj2",
            "ddj3",
            "ddj4",
            "ddj5",
            "ddj6",
            "dddj1",
            "dddj2",
            "dddj3",
            "dddj4",
            "dddj5",
            "dddj6",
            "x",
            "y",
            "z",
            "a",
            "b",
            "c",
            "dx",
            "dy",
            "dz",
            "da",
            "db",
            "dc",
            "ddx",
            "ddy",
            "ddz",
            "dda",
            "ddb",
            "ddc",
            "dddx",
            "dddy",
            "dddz",
            "ddda",
            "dddb",
            "dddc",
            "dynamic_errors",
            "dynamic_warnings",
            "articular_velocity_valid",
            "articular_acceleration_valid",
            "articular_jerk_valid",
            "cartesian_velocity_valid",
            "cartesian_acceleration_valid",
            "cartesian_jerk_valid",
        ]

    @staticmethod
    def _format_dynamic_violations(sample: TrajectorySample, severity: TrajectoryDynamicViolationSeverity) -> str:
        parts: list[str] = []
        for violation in sample.dynamic_violations:
            if violation.severity != severity:
                continue
            axis = f"J{violation.axis + 1}" if violation.axis >= 0 else "J?"
            parts.append(
                f"{violation.kind.value}:{axis}:"
                f"{TrajectoryController._fmt_csv(violation.value)}/"
                f"{TrajectoryController._fmt_csv(violation.limit)}"
            )
        return "|".join(parts)

    @staticmethod
    def _preferred_trajectories_dir() -> str:
        current_dir = os.getcwd()
        trajectories_dir = os.path.join(current_dir, "trajectories")
        return trajectories_dir if os.path.exists(trajectories_dir) else current_dir

    def _on_export_trajectory_requested(self) -> None:
        if not self.current_samples:
            QMessageBox.warning(
                self.trajectory_view,
                "Export trajectoire",
                "Aucune trajectoire calculée à exporter.",
            )
            return

        start_dir = self._preferred_trajectories_dir()
        default_path = str(Path(start_dir) / "trajectory_samples.csv") if start_dir else "trajectory_samples.csv"
        path, _ = QFileDialog.getSaveFileName(
            self.trajectory_view,
            "Exporter la trajectoire calculée",
            default_path,
            "Fichiers CSV (*.csv);;Tous les fichiers (*.*)",
        )
        if not path:
            return

        header = self._trajectory_export_header()

        display_frame = self.config_widget.get_cartesian_display_frame()
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()

        try:
            with open(path, "w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle, delimiter=";")
                writer.writerow(header)
                for sample in self.current_samples:
                    pose = convert_pose_from_base_frame(
                        Pose6(*sample.pose[:6]),
                        ReferenceFrame.from_value(display_frame),
                        robot_base_transform,
                    ).to_list()
                    if display_frame == "WORLD":
                        cartesian_velocity = twist_base_to_world(
                            Pose6(*sample.cartesian_velocity[:6]),
                            robot_base_transform,
                        ).to_list()
                        cartesian_acceleration = twist_base_to_world(
                            Pose6(*sample.cartesian_acceleration[:6]),
                            robot_base_transform,
                        ).to_list()
                        cartesian_jerk = twist_base_to_world(Pose6(*sample.cartesian_jerk[:6]), robot_base_transform).to_list()
                    else:
                        cartesian_velocity = [float(v) for v in sample.cartesian_velocity[:6]]
                        cartesian_acceleration = [float(v) for v in sample.cartesian_acceleration[:6]]
                        cartesian_jerk = [float(v) for v in sample.cartesian_jerk[:6]]
                    status = (
                        "VALID"
                        if sample.reachable and sample.error_code == TrajectorySampleErrorCode.NONE
                        else sample.error_code.name
                    )
                    config_name = sample.configuration.name if sample.configuration is not None else ""
                    row = [
                        status,
                        config_name,
                        self._fmt_csv(sample.time),
                    ]
                    row.extend(self._fmt_csv(v) for v in sample.joints[:6])
                    row.extend(self._fmt_csv(v) for v in sample.articular_velocity[:6])
                    row.extend(self._fmt_csv(v) for v in sample.articular_acceleration[:6])
                    row.extend(self._fmt_csv(v) for v in sample.articular_jerk[:6])
                    row.extend(self._fmt_csv(v) for v in pose[:6])
                    row.extend(self._fmt_csv(v) for v in cartesian_velocity[:6])
                    row.extend(self._fmt_csv(v) for v in cartesian_acceleration[:6])
                    row.extend(self._fmt_csv(v) for v in cartesian_jerk[:6])
                    row.append(self._format_dynamic_violations(sample, TrajectoryDynamicViolationSeverity.ERROR))
                    row.append(self._format_dynamic_violations(sample, TrajectoryDynamicViolationSeverity.WARNING))
                    row.extend(
                        [
                            str(bool(sample.articular_velocity_valid)).upper(),
                            str(bool(sample.articular_acceleration_valid)).upper(),
                            str(bool(sample.articular_jerk_valid)).upper(),
                            str(bool(sample.cartesian_velocity_valid)).upper(),
                            str(bool(sample.cartesian_acceleration_valid)).upper(),
                            str(bool(sample.cartesian_jerk_valid)).upper(),
                        ]
                    )
                    writer.writerow(row)
        except Exception as exc:
            QMessageBox.warning(
                self.trajectory_view,
                "Export trajectoire",
                f"Impossible d'exporter la trajectoire.\n{exc}",
            )

    def _recompute_trajectory(self, keypoints_override: list[TrajectoryKeypoint] | None = None) -> None:
        self._trajectory_generation_sequence += 1
        traj_id = self._trajectory_generation_sequence
        total_start_s = time.perf_counter()
        running_worker_count = len(self._collision_workers)
        worker_free = running_worker_count == 0
        self._log_benchmark("---------------")
        self._log_benchmark(f"desir de faire une traj traj_id={traj_id}")
        self._log_benchmark(
            "worker free="
            f"{worker_free} active_job_id={self._active_collision_job_id} running_workers={running_worker_count}"
        )
        self._cancel_collision_analysis()
        self._stop_playback()
        self.trajectory_builder.set_time_smoothing_enabled(self.config_widget.is_time_smoothing_enabled())
        self.trajectory_builder.set_jerk_check_enabled(self.config_widget.is_jerk_check_enabled())
        self.trajectory_builder.set_bezier_degree(self.config_widget.get_bezier_degree())
        if keypoints_override is None:
            keypoints = self.config_widget.get_keypoints()
        else:
            keypoints = [keypoint.clone() for keypoint in keypoints_override]
        self._displayed_keypoints = [keypoint.clone() for keypoint in keypoints]
        generation_start_s = time.perf_counter()
        self._log_benchmark(f"generating traj... traj_id={traj_id}")
        if not keypoints:
            self.current_trajectory = TrajectoryResult()
            self.config_widget.set_trajectory_context(self.current_trajectory)
            self.current_samples = []
            self.current_sample_times = []
            self._reset_trajectory_visuals()
            self._update_trajectory_issue_messages()
            generation_ms = self._elapsed_ms(generation_start_s)
            self._log_benchmark(f"done : {generation_ms:.3f} ms traj_id={traj_id} samples=0")
            self._log_benchmark(f"detect collision skipped... traj_id={traj_id} reason=no_samples")
            total_end_s = time.perf_counter()
            total_ms = self._duration_ms(total_start_s, total_end_s)
            self._log_benchmark(
                "temps total start->end : "
                f"{total_ms:.3f} ms traj_id={traj_id} "
                f"total_start_perf_s={total_start_s:.9f} total_end_perf_s={total_end_s:.9f} "
                f"trajectory_object_id={id(self.current_trajectory)}"
            )
            return

        current_joints = self.robot_model.get_joints()
        if len(keypoints) == 1:
            first_segment = self.trajectory_builder.compute_first_segment(current_joints, keypoints[0], 0.0)
            self.current_trajectory = TrajectoryResult()
            self.current_trajectory.segments.append(first_segment)
            if first_segment.status != TrajectoryComputationStatus.SUCCESS:
                self.current_trajectory.status = first_segment.status
                self.current_trajectory.first_error_segment_index = 0
        else:
            segments = self._build_segments(keypoints)
            self.current_trajectory = self.trajectory_builder.compute_trajectory(current_joints, segments)

        self.config_widget.set_trajectory_context(self.current_trajectory)
        self.current_samples = self._flatten_samples(self.current_trajectory)
        self.current_sample_times = [sample.time for sample in self.current_samples]
        generation_ms = self._elapsed_ms(generation_start_s)
        self._log_benchmark(
            f"done : {generation_ms:.3f} ms traj_id={traj_id} "
            f"trajectory_object_id={id(self.current_trajectory)} samples={len(self.current_samples)}"
        )
        self._update_graphs()
        self._update_3d_trajectory_path()
        self._update_3d_keypoint_overlays()
        self._update_timeline()
        self._update_trajectory_issue_messages()
        self._apply_time_value(0.0, force_real_robot=False)
        self._start_collision_analysis(traj_id, total_start_s)

    @staticmethod
    def _elapsed_ms(start_s: float) -> float:
        return (time.perf_counter() - float(start_s)) * 1000.0

    @staticmethod
    def _duration_ms(start_s: float, end_s: float) -> float:
        return (float(end_s) - float(start_s)) * 1000.0

    @staticmethod
    def _log_benchmark(message: str) -> None:
        print(f"[trajectory-benchmark] {message}", flush=True)

    @staticmethod
    def _build_segments(keypoints: list[TrajectoryKeypoint]) -> list[TrajectorySegment]:
        if len(keypoints) < 2:
            return []
        return [TrajectorySegment(keypoints[i], keypoints[i + 1]) for i in range(len(keypoints) - 1)]

    @staticmethod
    def _flatten_samples(trajectory: TrajectoryResult) -> list[TrajectorySample]:
        samples: list[TrajectorySample] = []
        for segment in trajectory.segments:
            samples.extend(segment.samples)
        return samples

    def _cancel_collision_analysis(self) -> None:
        if self._collision_workers:
            self._log_benchmark(
                f"requete de cancel active_job_id={self._active_collision_job_id} "
                f"worker_count={len(self._collision_workers)}"
            )
        self._active_collision_job_id = None
        for worker in list(self._collision_workers):
            worker.request_cancel()

    def _start_collision_analysis(self, traj_id: int, total_start_s: float) -> None:
        if not self.current_samples:
            self._log_benchmark(f"detect collision skipped... traj_id={traj_id} reason=no_samples")
            total_end_s = time.perf_counter()
            total_ms = self._duration_ms(total_start_s, total_end_s)
            self._log_benchmark(
                "temps total start->end : "
                f"{total_ms:.3f} ms traj_id={traj_id} "
                f"total_start_perf_s={total_start_s:.9f} total_end_perf_s={total_end_s:.9f} "
                f"trajectory_object_id={id(self.current_trajectory)}"
            )
            return

        self._collision_job_sequence += 1
        job_id = self._collision_job_sequence
        self._active_collision_job_id = job_id
        self._collision_job_traj_ids[job_id] = int(traj_id)
        self._collision_job_total_start_s[job_id] = float(total_start_s)
        self._collision_job_detect_start_s[job_id] = time.perf_counter()
        self._log_benchmark(
            f"detect collision started... traj_id={traj_id} job_id={job_id} "
            f"trajectory_object_id={id(self.current_trajectory)} samples={len(self.current_samples)}"
        )

        context = TrajectoryCollisionContext.from_models(
            self.robot_model,
            self.tool_model,
            self.workspace_model,
        )
        cancel_token = TrajectoryCollisionCancelToken()
        worker = TrajectoryCollisionWorker(
            job_id=job_id,
            trajectory=self.current_trajectory,
            context=context,
            cancel_token=cancel_token,
        )
        thread = QThread(self)
        worker.moveToThread(thread)

        thread.started.connect(worker.run)
        worker.completed.connect(self._on_collision_worker_completed)
        worker.cancelled.connect(self._on_collision_worker_cancelled)
        worker.finished.connect(worker.deleteLater)
        worker.finished.connect(thread.quit)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(
            lambda thread_ref=thread, worker_ref=worker: self._forget_collision_worker(thread_ref, worker_ref)
        )

        self._collision_threads.append(thread)
        self._collision_workers.append(worker)
        thread.start()

    def _on_collision_worker_completed(self, job_id: int, payload: object) -> None:
        if job_id != self._active_collision_job_id:
            self._log_benchmark(
                f"detect collision done ignored stale job_id={job_id} active_job_id={self._active_collision_job_id}"
            )
            self._clear_collision_job_benchmark(job_id)
            return
        self._active_collision_job_id = None
        if not isinstance(payload, TrajectoryCollisionAnalysisResult):
            self._clear_collision_job_benchmark(job_id)
            return
        detect_start_s = self._collision_job_detect_start_s.get(job_id)
        total_start_s = self._collision_job_total_start_s.get(job_id)
        traj_id = self._collision_job_traj_ids.get(job_id)
        if detect_start_s is not None:
            self._log_benchmark(
                f"done : {self._elapsed_ms(detect_start_s):.3f} ms "
                f"traj_id={traj_id} job_id={job_id} has_collision={payload.has_collision}"
            )
        if total_start_s is not None:
            total_end_s = time.perf_counter()
            self._log_benchmark(
                "temps total start->end : "
                f"{self._duration_ms(total_start_s, total_end_s):.3f} ms traj_id={traj_id} "
                f"job_id={job_id} total_start_perf_s={total_start_s:.9f} "
                f"total_end_perf_s={total_end_s:.9f} "
                f"trajectory_object_id={id(self.current_trajectory)}"
            )
        self._clear_collision_job_benchmark(job_id)

        applied = apply_trajectory_collision_result(self.current_trajectory, payload)
        if not applied:
            return

        self.config_widget.set_trajectory_context(self.current_trajectory)
        self._update_graphs()
        self._update_3d_trajectory_path()
        self._update_trajectory_issue_messages()

    def _on_collision_worker_cancelled(self, job_id: int) -> None:
        traj_id = self._collision_job_traj_ids.get(job_id)
        detect_start_s = self._collision_job_detect_start_s.get(job_id)
        if detect_start_s is None:
            self._log_benchmark(f"worker canceled traj_id={traj_id} job_id={job_id}")
        else:
            self._log_benchmark(
                f"worker canceled traj_id={traj_id} job_id={job_id} "
                f"after={self._elapsed_ms(detect_start_s):.3f} ms"
            )
        self._clear_collision_job_benchmark(job_id)
        if job_id == self._active_collision_job_id:
            self._active_collision_job_id = None

    def _clear_collision_job_benchmark(self, job_id: int) -> None:
        self._collision_job_traj_ids.pop(job_id, None)
        self._collision_job_total_start_s.pop(job_id, None)
        self._collision_job_detect_start_s.pop(job_id, None)

    def _forget_collision_worker(
        self,
        thread: QThread,
        worker: TrajectoryCollisionWorker,
    ) -> None:
        if thread in self._collision_threads:
            self._collision_threads.remove(thread)
        if worker in self._collision_workers:
            self._collision_workers.remove(worker)

    def _update_graphs(self) -> None:
        articular_panel = self.graphs_widget.get_articular_panel()
        cartesian_panel = self.graphs_widget.get_cartesian_panel()
        config_timeline = self.graphs_widget.get_configuration_timeline_widget()

        if not self.current_samples:
            empty_series = [[] for _ in range(6)]
            articular_panel.set_trajectories([], empty_series, empty_series, empty_series, empty_series)
            cartesian_panel.set_trajectories([], empty_series, empty_series, empty_series, empty_series)
            config_timeline.set_configuration_data([], [])
            articular_panel.set_key_times([])
            cartesian_panel.set_key_times([])
            config_timeline.set_key_times([])
            articular_panel.set_time_indicator(None)
            cartesian_panel.set_time_indicator(None)
            config_timeline.set_time_indicator(None)
            return

        times = self.current_sample_times
        cartesian_samples = self._cartesian_samples_for_display()
        cart_positions = [[sample[0][axis] for sample in cartesian_samples] for axis in range(6)]
        cart_velocities = [[sample[1][axis] for sample in cartesian_samples] for axis in range(6)]
        cart_accelerations = [[sample[2][axis] for sample in cartesian_samples] for axis in range(6)]
        cart_jerks = [[sample[3][axis] for sample in cartesian_samples] for axis in range(6)]
        art_positions = [[sample.joints[axis] for sample in self.current_samples] for axis in range(6)]
        art_velocities = [[sample.articular_velocity[axis] for sample in self.current_samples] for axis in range(6)]
        art_accelerations = [[sample.articular_acceleration[axis] for sample in self.current_samples] for axis in range(6)]
        art_jerks = [[sample.articular_jerk[axis] for sample in self.current_samples] for axis in range(6)]
        key_times = [segment.last_time for segment in self.current_trajectory.segments if segment.last_time > 0.0]

        cartesian_panel.set_trajectories(times, cart_positions, cart_velocities, cart_accelerations, cart_jerks)
        articular_panel.set_trajectories(times, art_positions, art_velocities, art_accelerations, art_jerks)
        config_timeline.set_configuration_data(times, self.current_samples)
        cartesian_panel.set_key_times(key_times)
        articular_panel.set_key_times(key_times)
        config_timeline.set_key_times(key_times)

    def _update_3d_trajectory_path(self) -> None:
        if not self.current_samples:
            self.viewer3d_controller.clear_trajectory_path()
            if not self._is_keypoint_preview_active:
                self.viewer3d_controller.hide_robot_ghost()
            return

        colored_segments = self._build_colored_3d_trajectory_path_segments()
        if colored_segments:
            self.viewer3d_controller.set_trajectory_path_segments(colored_segments)
        else:
            self.viewer3d_controller.clear_trajectory_path()

    @staticmethod
    def _sample_xyz(sample: TrajectorySample) -> list[float]:
        return [float(sample.pose[0]), float(sample.pose[1]), float(sample.pose[2])]

    def _cartesian_samples_for_display(self) -> list[tuple[list[float], list[float], list[float], list[float]]]:
        display_frame = self.config_widget.get_cartesian_display_frame()
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()
        out: list[tuple[list[float], list[float], list[float], list[float]]] = []
        for sample in self.current_samples:
            pose = convert_pose_from_base_frame(
                Pose6(*sample.pose[:6]),
                ReferenceFrame.from_value(display_frame),
                robot_base_transform,
            ).to_list()
            if display_frame == "WORLD":
                velocity = twist_base_to_world(Pose6(*sample.cartesian_velocity[:6]), robot_base_transform).to_list()
                acceleration = twist_base_to_world(Pose6(*sample.cartesian_acceleration[:6]), robot_base_transform).to_list()
                jerk = twist_base_to_world(Pose6(*sample.cartesian_jerk[:6]), robot_base_transform).to_list()
            else:
                velocity = [float(v) for v in sample.cartesian_velocity[:6]]
                acceleration = [float(v) for v in sample.cartesian_acceleration[:6]]
                jerk = [float(v) for v in sample.cartesian_jerk[:6]]
            out.append((pose, velocity, acceleration, jerk))
        return out

    def _base_path_color_for_mode(self, mode: KeypointMotionMode) -> tuple[float, float, float, float]:
        if mode == KeypointMotionMode.PTP:
            return self._PATH_COLOR_PTP
        return self._PATH_COLOR_LIN_CUBIC

    def _edge_color_for_samples(
        self,
        sample_a: TrajectorySample,
        sample_b: TrajectorySample,
        base_color: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        if sample_a.error_code != TrajectorySampleErrorCode.NONE or sample_b.error_code != TrajectorySampleErrorCode.NONE:
            return self._PATH_COLOR_ERROR
        return base_color

    @staticmethod
    def _append_colored_edge(
        chunks: list[tuple[list[list[float]], tuple[float, float, float, float]]],
        start_xyz: list[float],
        end_xyz: list[float],
        color: tuple[float, float, float, float],
    ) -> None:
        if not chunks:
            chunks.append(([start_xyz, end_xyz], color))
            return

        last_points, last_color = chunks[-1]
        if last_color == color and last_points[-1] == start_xyz:
            last_points.append(end_xyz)
            return

        chunks.append(([start_xyz, end_xyz], color))

    def _build_colored_3d_trajectory_path_segments(
        self,
    ) -> list[tuple[list[list[float]], tuple[float, float, float, float]]]:
        chunks: list[tuple[list[list[float]], tuple[float, float, float, float]]] = []
        previous_last_sample: TrajectorySample | None = None

        for segment in self.current_trajectory.segments:
            segment_samples = segment.samples
            if not segment_samples:
                continue
            base_color = self._base_path_color_for_mode(segment.mode)

            if previous_last_sample is not None:
                start_sample = previous_last_sample
                end_sample = segment_samples[0]
                edge_color = self._edge_color_for_samples(start_sample, end_sample, base_color)
                self._append_colored_edge(
                    chunks,
                    self._sample_xyz(start_sample),
                    self._sample_xyz(end_sample),
                    edge_color,
                )

            for sample_index in range(1, len(segment_samples)):
                start_sample = segment_samples[sample_index - 1]
                end_sample = segment_samples[sample_index]
                edge_color = self._edge_color_for_samples(start_sample, end_sample, base_color)
                self._append_colored_edge(
                    chunks,
                    self._sample_xyz(start_sample),
                    self._sample_xyz(end_sample),
                    edge_color,
                )

            previous_last_sample = segment_samples[-1]

        return chunks

    def _build_edit_tangent_segments(
        self,
    ) -> tuple[list[TangentSegment] | None, list[TangentSegment] | None]:
        if self._editing_keypoint_index is None:
            return None, None
        keypoints = self._displayed_keypoints
        if not keypoints or not self.current_trajectory.segments:
            return None, None

        tcp_pose = self.robot_model.get_tcp_pose()
        first_start_anchor = XYZ3(tcp_pose.x, tcp_pose.y, tcp_pose.z)

        tangent_out_segments: list[TangentSegment] = []
        tangent_in_segments: list[TangentSegment] = []
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()
        count = min(len(self.current_trajectory.segments), len(keypoints))
        for segment_index in range(count):
            segment_result = self.current_trajectory.segments[segment_index]
            end_anchor = resolve_keypoint_xyz(
                self.robot_model,
                keypoints[segment_index],
                tool=self.tool_model.get_tool(),
                robot_base_pose_world=robot_base_transform,
            )
            if end_anchor is None:
                continue

            if segment_index == 0:
                start_anchor = first_start_anchor
            else:
                start_anchor = resolve_keypoint_xyz(
                    self.robot_model,
                    keypoints[segment_index - 1],
                    tool=self.tool_model.get_tool(),
                    robot_base_pose_world=robot_base_transform,
                )
                if start_anchor is None:
                    continue

            start_anchor_xyz = start_anchor.copy()
            end_anchor_xyz = end_anchor.copy()
            out_direction = XYZ3(*segment_result.out_direction[:3])
            in_direction = XYZ3(*segment_result.in_direction[:3])

            if not math_utils.is_near_zero_vector_xyz(out_direction.to_list()):
                tangent_out_segments.append(
                    (
                        start_anchor_xyz,
                        XYZ3(
                            start_anchor_xyz.x + out_direction.x,
                            start_anchor_xyz.y + out_direction.y,
                            start_anchor_xyz.z + out_direction.z,
                        ),
                    )
                )

            if not math_utils.is_near_zero_vector_xyz(in_direction.to_list()):
                tangent_in_segments.append(
                    (
                        end_anchor_xyz,
                        XYZ3(
                            end_anchor_xyz.x + in_direction.x,
                            end_anchor_xyz.y + in_direction.y,
                            end_anchor_xyz.z + in_direction.z,
                        ),
                    )
                )

        return (
            tangent_out_segments if tangent_out_segments else None,
            tangent_in_segments if tangent_in_segments else None,
        )

    def _update_3d_keypoint_overlays(self) -> None:
        if not self._displayed_keypoints:
            self.viewer3d_controller.clear_trajectory_keypoints()
            self.viewer3d_controller.clear_trajectory_edit_tangents()
            return

        points_xyz: list[list[float]] = []
        index_map: list[int] = []
        robot_base_transform = self.workspace_model.get_robot_base_transform_world()
        for idx, keypoint in enumerate(self._displayed_keypoints):
            xyz = resolve_keypoint_xyz(
                self.robot_model,
                keypoint,
                tool=self.tool_model.get_tool(),
                robot_base_pose_world=robot_base_transform,
            )
            if xyz is None:
                continue
            points_xyz.append(xyz.to_list())
            index_map.append(idx)

        def _mapped_index(source_idx: int | None) -> int | None:
            if source_idx is None:
                return None
            try:
                return index_map.index(source_idx)
            except ValueError:
                return None

        self.viewer3d_controller.set_trajectory_keypoints(
            points_xyz,
            selected_index=_mapped_index(self._selected_keypoint_index),
            editing_index=_mapped_index(self._editing_keypoint_index),
        )

        tangent_out_segments, tangent_in_segments = self._build_edit_tangent_segments()
        self.viewer3d_controller.set_trajectory_edit_tangents(tangent_out_segments, tangent_in_segments)

    def _update_timeline(self) -> None:
        if not self.current_samples:
            self.actions_widget.set_time_range(0.0, 0.0)
            return
        end_time = self.current_samples[-1].time
        self.actions_widget.set_time_range(0.0, end_time)

    def _reset_trajectory_visuals(self) -> None:
        self._current_time_s = 0.0
        self._update_graphs()
        self.actions_widget.set_time_range(0.0, 0.0)
        self.actions_widget.set_issue_messages([])
        self.actions_widget.set_warning_messages([])
        self.viewer3d_controller.clear_trajectory_path()
        self.viewer3d_controller.clear_trajectory_keypoints()
        self.viewer3d_controller.clear_trajectory_edit_tangents()
        if not self._is_keypoint_preview_active:
            self.viewer3d_controller.hide_robot_ghost()

    def _update_trajectory_issue_messages(self) -> None:
        issues = build_trajectory_issue_messages(self.current_trajectory)
        warnings = build_trajectory_warning_messages(self.current_trajectory)
        self.actions_widget.set_issue_messages(issues)
        self.actions_widget.set_warning_messages(warnings)

    def _on_time_value_changed(self, time_s: float) -> None:
        self._apply_time_value(time_s, force_real_robot=True)
        if self._is_playing:
            self._playback_index = self._sample_index_at_time(time_s)
            self._playback_sim_start_s = float(time_s)
            self._playback_wall_start_s = time.perf_counter()

    def _apply_time_value(self, time_s: float, force_real_robot: bool) -> None:
        self._current_time_s = float(time_s)
        articular_panel = self.graphs_widget.get_articular_panel()
        cartesian_panel = self.graphs_widget.get_cartesian_panel()
        config_timeline = self.graphs_widget.get_configuration_timeline_widget()
        articular_panel.set_time_indicator(time_s)
        cartesian_panel.set_time_indicator(time_s)
        config_timeline.set_time_indicator(time_s)

        sample = self._sample_at_time(time_s)
        if sample is None:
            if not self._is_keypoint_preview_active:
                self.viewer3d_controller.hide_robot_ghost()
            return

        if sample.reachable:
            if force_real_robot:
                self.viewer3d_controller.hide_robot_ghost()
                self.robot_model.set_joints(sample.joints)
            elif not self._is_keypoint_preview_active:
                # Simulation timeline should not spawn the ghost outside edition mode.
                self.viewer3d_controller.hide_robot_ghost()
        else:
            if not self._is_keypoint_preview_active:
                self.viewer3d_controller.hide_robot_ghost()

    def _sample_index_at_time(self, time_s: float) -> int:
        if not self.current_sample_times:
            return 0
        times = self.current_sample_times
        idx = bisect_left(times, time_s)
        if idx <= 0:
            return 0
        if idx >= len(times):
            return len(times) - 1
        previous_time = times[idx - 1]
        next_time = times[idx]
        return idx - 1 if (time_s - previous_time) <= (next_time - time_s) else idx

    def _sample_at_time(self, time_s: float) -> TrajectorySample | None:
        if not self.current_samples:
            return None
        return self.current_samples[self._sample_index_at_time(time_s)]

    def _stop_playback(self) -> None:
        self._is_playing = False
        self._playback_wall_start_s = None
        self._playback_timer.stop()

    def _on_play_requested(self) -> None:
        if not self.current_samples:
            return
        self._is_playing = True
        self._playback_index = self._sample_index_at_time(self._current_time_s)
        self._playback_sim_start_s = float(self._current_time_s)
        self._playback_wall_start_s = time.perf_counter()
        timer_interval_ms = max(1, int(round(self.trajectory_builder.sample_dt_s * 1000.0)))
        self._playback_timer.start(timer_interval_ms)
        self._on_playback_tick()

    def _on_pause_requested(self) -> None:
        self._stop_playback()

    def _on_stop_requested(self) -> None:
        self._stop_playback()
        self._playback_index = 0
        self.actions_widget.set_time_value(0.0)
        self._apply_time_value(0.0, force_real_robot=True)

    def _on_playback_tick(self) -> None:
        if not self._is_playing:
            return

        if not self.current_samples:
            self._stop_playback()
            return

        wall_start = self._playback_wall_start_s
        if wall_start is None:
            self._playback_wall_start_s = time.perf_counter()
            self._playback_sim_start_s = float(self._current_time_s)
            wall_start = self._playback_wall_start_s

        elapsed_s = max(0.0, time.perf_counter() - wall_start)
        target_time_s = self._playback_sim_start_s + elapsed_s
        end_time_s = self.current_samples[-1].time

        if target_time_s >= end_time_s:
            self._stop_playback()
            self.actions_widget.set_time_value(end_time_s)
            self._apply_time_value(end_time_s, force_real_robot=True)
            return

        self._playback_index = self._sample_index_at_time(target_time_s)
        self.actions_widget.set_time_value(target_time_s)
        self._apply_time_value(target_time_s, force_real_robot=True)

