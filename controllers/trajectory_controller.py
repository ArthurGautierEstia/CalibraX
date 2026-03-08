import csv
from bisect import bisect_left
from pathlib import Path
import time
import os

from PyQt6.QtCore import QObject, QTimer, Qt
from PyQt6.QtWidgets import QFileDialog, QMessageBox

from models.robot_model import RobotModel
from models.trajectory_result import (
    TrajectoryComputationStatus,
    TrajectoryResult,
    TrajectorySample,
    TrajectorySampleErrorCode,
    TrajectorySegment,
)
from models.trajectory_keypoint import KeypointMotionMode, KeypointTargetType, TrajectoryKeypoint
from utils.trajectory_builder import TrajectoryBuilder
from utils.trajectory_keypoint_utils import resolve_keypoint_xyz
from utils.trajectory_status import build_trajectory_issue_messages
from views.trajectory_view import TrajectoryView
from controllers.viewer3d_controller import Viewer3DController
import utils.math_utils as math_utils


class TrajectoryController(QObject):
    _PATH_COLOR_LIN_CUBIC = (1.0, 0.84, 0.1, 0.85)
    _PATH_COLOR_PTP = (0.25, 0.65, 1.0, 0.9)
    _PATH_COLOR_ERROR = (1.0, 0.2, 0.2, 0.95)

    def __init__(
        self,
        robot_model: RobotModel,
        trajectory_view: TrajectoryView,
        viewer3d_controller: Viewer3DController,
        parent: QObject = None,
    ):
        super().__init__(parent)

        self.robot_model = robot_model
        self.trajectory_view = trajectory_view
        self.viewer3d_controller = viewer3d_controller
        self.config_widget = self.trajectory_view.get_config_widget()
        self.actions_widget = self.trajectory_view.get_actions_widget()
        self.graphs_widget = self.trajectory_view.get_graphs_widget()

        self.trajectory_builder = TrajectoryBuilder(self.robot_model)
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
        self.actions_widget.compute_requested.connect(self._on_compute_requested)
        self.actions_widget.export_trajectory_requested.connect(self._on_export_trajectory_requested)
        self.actions_widget.home_position_requested.connect(self._on_home_position_requested)
        self.actions_widget.play_requested.connect(self._on_play_requested)
        self.actions_widget.pause_requested.connect(self._on_pause_requested)
        self.actions_widget.stop_requested.connect(self._on_stop_requested)
        self.actions_widget.time_value_changed.connect(self._on_time_value_changed)

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
            fk_result = self.robot_model.compute_fk_joints(joints)
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

        mgi_result = self.robot_model.compute_ik_target(keypoint.cartesian_target[:6])
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
    def _preferred_trajectories_dir() -> str:
        current_dir = os.getcwd()
        trajectories_dir = os.path.join(current_dir, "trajectories")
        return trajectories_dir if os.path.exists(trajectories_dir) else current_dir

    def _on_export_trajectory_requested(self) -> None:
        if not self.current_samples:
            QMessageBox.warning(
                self.trajectory_view,
                "Export trajectoire",
                "Aucune trajectoire calculee a exporter.",
            )
            return

        start_dir = self._preferred_trajectories_dir()
        default_path = str(Path(start_dir) / "trajectory_samples.csv") if start_dir else "trajectory_samples.csv"
        path, _ = QFileDialog.getSaveFileName(
            self.trajectory_view,
            "Exporter la trajectoire calculee",
            default_path,
            "Fichiers CSV (*.csv);;Tous les fichiers (*.*)",
        )
        if not path:
            return

        header = [
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
        ]

        try:
            with open(path, "w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle, delimiter=";")
                writer.writerow(header)
                for sample in self.current_samples:
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
                    row.extend(self._fmt_csv(v) for v in sample.pose[:6])
                    row.extend(self._fmt_csv(v) for v in sample.cartesian_velocity[:6])
                    row.extend(self._fmt_csv(v) for v in sample.cartesian_acceleration[:6])
                    writer.writerow(row)
        except Exception as exc:
            QMessageBox.warning(
                self.trajectory_view,
                "Export trajectoire",
                f"Impossible d'exporter la trajectoire.\n{exc}",
            )

    def _recompute_trajectory(self, keypoints_override: list[TrajectoryKeypoint] | None = None) -> None:
        self._stop_playback()
        if keypoints_override is None:
            keypoints = self.config_widget.get_keypoints()
        else:
            keypoints = [keypoint.clone() for keypoint in keypoints_override]
        self._displayed_keypoints = [keypoint.clone() for keypoint in keypoints]
        if not keypoints:
            self.current_trajectory = TrajectoryResult()
            self.config_widget.set_trajectory_context(self.current_trajectory)
            self.current_samples = []
            self.current_sample_times = []
            self._reset_trajectory_visuals()
            self._update_trajectory_issue_messages()
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
        self._update_graphs()
        self._update_3d_trajectory_path()
        self._update_3d_keypoint_overlays()
        self._update_timeline()
        self._update_trajectory_issue_messages()
        self._apply_time_value(0.0, force_real_robot=False)

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

    def _update_graphs(self) -> None:
        articular_panel = self.graphs_widget.get_articular_panel()
        cartesian_panel = self.graphs_widget.get_cartesian_panel()
        config_timeline = self.graphs_widget.get_configuration_timeline_widget()

        if not self.current_samples:
            empty_series = [[] for _ in range(6)]
            articular_panel.set_trajectories([], empty_series, empty_series, empty_series)
            cartesian_panel.set_trajectories([], empty_series, empty_series, empty_series)
            config_timeline.set_configuration_data([], [])
            articular_panel.set_key_times([])
            cartesian_panel.set_key_times([])
            config_timeline.set_key_times([])
            articular_panel.set_time_indicator(None)
            cartesian_panel.set_time_indicator(None)
            config_timeline.set_time_indicator(None)
            return

        times = self.current_sample_times
        cart_positions = [[sample.pose[axis] for sample in self.current_samples] for axis in range(6)]
        cart_velocities = [[sample.cartesian_velocity[axis] for sample in self.current_samples] for axis in range(6)]
        cart_accelerations = [[sample.cartesian_acceleration[axis] for sample in self.current_samples] for axis in range(6)]
        art_positions = [[sample.joints[axis] for sample in self.current_samples] for axis in range(6)]
        art_velocities = [[sample.articular_velocity[axis] for sample in self.current_samples] for axis in range(6)]
        art_accelerations = [[sample.articular_acceleration[axis] for sample in self.current_samples] for axis in range(6)]
        key_times = [segment.last_time for segment in self.current_trajectory.segments if segment.last_time > 0.0]

        cartesian_panel.set_trajectories(times, cart_positions, cart_velocities, cart_accelerations)
        articular_panel.set_trajectories(times, art_positions, art_velocities, art_accelerations)
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
    ) -> tuple[list[list[list[float]]] | None, list[list[list[float]]] | None]:
        if self._editing_keypoint_index is None:
            return None, None
        keypoints = self._displayed_keypoints
        if not keypoints or not self.current_trajectory.segments:
            return None, None

        tcp_pose = self.robot_model.get_tcp_pose()
        if len(tcp_pose) < 3:
            return None, None
        first_start_anchor = [float(v) for v in tcp_pose[:3]]

        tangent_out_segments: list[list[list[float]]] = []
        tangent_in_segments: list[list[list[float]]] = []
        count = min(len(self.current_trajectory.segments), len(keypoints))
        for segment_index in range(count):
            segment_result = self.current_trajectory.segments[segment_index]
            end_anchor = resolve_keypoint_xyz(self.robot_model, keypoints[segment_index])
            if end_anchor is None:
                continue

            if segment_index == 0:
                start_anchor = first_start_anchor
            else:
                start_anchor = resolve_keypoint_xyz(self.robot_model, keypoints[segment_index - 1])
                if start_anchor is None:
                    continue

            out_direction = [float(v) for v in segment_result.out_direction[:3]]
            in_direction = [float(v) for v in segment_result.in_direction[:3]]

            if not math_utils.is_near_zero_vector_xyz(out_direction):
                tangent_out_segments.append(
                    [
                        start_anchor,
                        [
                            start_anchor[0] + out_direction[0],
                            start_anchor[1] + out_direction[1],
                            start_anchor[2] + out_direction[2],
                        ],
                    ]
                )

            if not math_utils.is_near_zero_vector_xyz(in_direction):
                tangent_in_segments.append(
                    [
                        end_anchor,
                        [
                            end_anchor[0] + in_direction[0],
                            end_anchor[1] + in_direction[1],
                            end_anchor[2] + in_direction[2],
                        ],
                    ]
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
        for idx, keypoint in enumerate(self._displayed_keypoints):
            xyz = resolve_keypoint_xyz(self.robot_model, keypoint)
            if xyz is None:
                continue
            points_xyz.append(xyz)
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
        self.viewer3d_controller.clear_trajectory_path()
        self.viewer3d_controller.clear_trajectory_keypoints()
        self.viewer3d_controller.clear_trajectory_edit_tangents()
        if not self._is_keypoint_preview_active:
            self.viewer3d_controller.hide_robot_ghost()

    def _update_trajectory_issue_messages(self) -> None:
        issues = build_trajectory_issue_messages(self.current_trajectory)
        self.actions_widget.set_issue_messages(issues)

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
