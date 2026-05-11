from __future__ import annotations

from PyQt6.QtCore import QObject, pyqtSignal

from models.types import JointAngles6
from trajectory_engine.adapters.legacy_converters import to_legacy_preview, to_legacy_trajectory
from trajectory_engine.managers.trajectory_build_manager import TrajectoryBuildManager
from trajectory_engine.models import TrajectoryBuildRequest, TrajectoryBuildTriggerMode


class TrajectoryControllerBuildBridge(QObject):
    preview_ready = pyqtSignal(object)
    result_ready = pyqtSignal(object)
    build_failed = pyqtSignal(str, str)

    def __init__(
        self,
        build_manager: TrajectoryBuildManager,
        parent: QObject | None = None,
    ) -> None:
        super().__init__(parent)
        self._build_manager = build_manager
        self._build_manager.preview_ready.connect(self._on_preview_ready)
        self._build_manager.result_ready.connect(self._on_result_ready)
        self._build_manager.build_failed.connect(self._on_build_failed)

    def submit(
        self,
        current_joints: JointAngles6,
        keypoints: list,
        sample_dt_s: float,
        jerk_check_enabled: bool,
        cartesian_accel_limit_mm_s2: float,
        cartesian_jerk_limit_mm_s3: float,
        trigger_mode: TrajectoryBuildTriggerMode,
    ) -> int:
        request = TrajectoryBuildRequest(
            revision_id=0,
            current_joints=current_joints.copy(),
            keypoints=[keypoint.clone() for keypoint in keypoints],
            sample_dt_s=float(sample_dt_s),
            smooth_time_enabled=True,
            bezier_degree="BEZIER7",
            jerk_check_enabled=bool(jerk_check_enabled),
            cartesian_accel_limit_mm_s2=float(cartesian_accel_limit_mm_s2),
            cartesian_jerk_limit_mm_s3=float(cartesian_jerk_limit_mm_s3),
            trigger_mode=trigger_mode,
        )
        return self._build_manager.submit(request)

    def cancel_active(self) -> None:
        self._build_manager.cancel_active()

    def shutdown(self) -> None:
        self._build_manager.shutdown()

    def _on_preview_ready(self, _revision_id: int, payload: object) -> None:
        self.preview_ready.emit(to_legacy_preview(payload))

    def _on_result_ready(self, _revision_id: int, payload: object) -> None:
        self.result_ready.emit(to_legacy_trajectory(payload))

    def _on_build_failed(self, _revision_id: int, stage: str, message: str) -> None:
        self.build_failed.emit(stage, message)
