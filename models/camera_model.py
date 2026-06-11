from __future__ import annotations

import os
import json
from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal

from models.types import Pose6
from utils import math_utils


class CameraVisibilityState(str, Enum):
    DISABLED = "disabled"
    VISIBLE = "visible"
    PARTIAL = "partial"
    NOT_VISIBLE = "not_visible"
    OUT_OF_FOV = "out_of_fov"
    OUT_OF_RANGE = "out_of_range"
    OCCLUDED = "occluded"
    INVALID = "invalid"


@dataclass(frozen=True)
class CameraFov:
    horizontal_deg: float = 60.0
    vertical_deg: float = 45.0
    range_mm: float = 4000.0

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "CameraFov":
        values = data if isinstance(data, dict) else {}
        range_value = values.get("range_mm", values.get("range_m", 4.0))
        if "range_mm" not in values and "range_m" in values:
            range_value = float(range_value) * 1000.0
        return cls(
            horizontal_deg=float(values.get("horizontal_deg", 60.0)),
            vertical_deg=float(values.get("vertical_deg", 45.0)),
            range_mm=float(range_value),
        )

    def to_dict(self) -> dict[str, float]:
        return {
            "horizontal_deg": float(self.horizontal_deg),
            "vertical_deg": float(self.vertical_deg),
            "range_mm": float(self.range_mm),
        }

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.horizontal_deg <= 0.0 or self.horizontal_deg >= 180.0:
            errors.append("FOV horizontal doit etre dans ]0, 180[ degres")
        if self.vertical_deg <= 0.0 or self.vertical_deg >= 180.0:
            errors.append("FOV vertical doit etre dans ]0, 180[ degres")
        if self.range_mm <= 0.0:
            errors.append("Portee FOV doit etre strictement positive")
        return errors


@dataclass(frozen=True)
class CameraStl:
    path: str = ""
    color: str = "#2B8CBE"

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "CameraStl":
        values = data if isinstance(data, dict) else {}
        return cls(
            path=str(values.get("path", "")).strip(),
            color=str(values.get("color", "#2B8CBE")).strip() or "#2B8CBE",
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "path": self.path,
            "color": self.color,
        }

    def validate(self) -> list[str]:
        return []


@dataclass(frozen=True)
class CameraVisual:
    color: str = "#00AEEF"
    show_frustum: bool = True
    show_lines_to_markers: bool = True
    verify_markers_in_fov: bool = True
    verify_line_of_sight: bool = True

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "CameraVisual":
        values = data if isinstance(data, dict) else {}
        return cls(
            color=str(values.get("color", "#00AEEF")).strip() or "#00AEEF",
            show_frustum=bool(values.get("show_frustum", True)),
            show_lines_to_markers=bool(
                values.get(
                    "show_lines_to_markers",
                    values.get("show_lines_to_target_points", values.get("show_line_to_tcp", True)),
                )
            ),
            verify_markers_in_fov=bool(
                values.get(
                    "verify_markers_in_fov",
                    values.get("verify_target_points_in_fov", values.get("verify_tcp_in_fov", True)),
                )
            ),
            verify_line_of_sight=bool(values.get("verify_line_of_sight", True)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "color": self.color,
            "show_frustum": bool(self.show_frustum),
            "show_lines_to_markers": bool(self.show_lines_to_markers),
            "verify_markers_in_fov": bool(self.verify_markers_in_fov),
            "verify_line_of_sight": bool(self.verify_line_of_sight),
        }

    @property
    def show_line_to_tcp(self) -> bool:
        return self.show_lines_to_markers

    @property
    def verify_tcp_in_fov(self) -> bool:
        return self.verify_markers_in_fov

    @property
    def show_lines_to_target_points(self) -> bool:
        return self.show_lines_to_markers

    @property
    def verify_target_points_in_fov(self) -> bool:
        return self.verify_markers_in_fov


@dataclass(frozen=True)
class CameraTargetPoint:
    point_id: str
    name: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    diameter_mm: float = 20.0
    enabled: bool = True

    @classmethod
    def default(cls, index: int = 1) -> "CameraTargetPoint":
        return cls(point_id=f"M{index}", name=f"Marker {index}")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CameraTargetPoint":
        if not isinstance(data, dict):
            raise ValueError("Un marker Rigid Body doit etre un objet JSON")
        point_id = str(data.get("id", data.get("marker_id", data.get("point_id", "")))).strip()
        position = data.get("position", data.get("position_mm"))
        if isinstance(position, (list, tuple)) and len(position) >= 3:
            x, y, z = float(position[0]), float(position[1]), float(position[2])
        else:
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            z = float(data.get("z", 0.0))
        return cls(
            point_id=point_id,
            name=str(data.get("name", point_id or "Marker")).strip() or "Marker",
            x=x,
            y=y,
            z=z,
            diameter_mm=float(data.get("diameter_mm", data.get("diameter", 20.0))),
            enabled=bool(data.get("enabled", True)),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.point_id,
            "name": self.name,
            "enabled": bool(self.enabled),
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "diameter_mm": float(self.diameter_mm),
        }

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.point_id.strip() == "":
            errors.append("ID marker manquant")
        if self.diameter_mm <= 0.0:
            errors.append("Diametre marker doit etre strictement positif")
        return errors

    def xyz(self) -> np.ndarray:
        return np.array([float(self.x), float(self.y), float(self.z)], dtype=float)


@dataclass(frozen=True)
class CameraTargetBody:
    name: str = "Rigid Body"
    parent_frame: str = "frame_6"
    pose: Pose6 = field(default_factory=Pose6.zeros)
    stl: CameraStl = CameraStl(color="#4F8CFF")
    points: tuple[CameraTargetPoint, ...] = field(default_factory=tuple)

    @classmethod
    def default(cls) -> "CameraTargetBody":
        return cls(points=(CameraTargetPoint.default(1),))

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "CameraTargetBody":
        values = data if isinstance(data, dict) else {}
        points_data = values.get("markers", values.get("points", []))
        if not isinstance(points_data, list):
            raise ValueError("Le champ 'target_body.markers' doit etre une liste")
        pose_values = values.get("pose", values.get("rigid_body_pose"))
        return cls(
            name=str(values.get("name", "Rigid Body")).strip() or "Rigid Body",
            parent_frame=str(values.get("parent_frame", "frame_6")).strip().lower() or "frame_6",
            pose=Pose6.from_values(pose_values),
            stl=CameraStl.from_dict(values.get("stl")),
            points=tuple(CameraTargetPoint.from_dict(item) for item in points_data),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "parent_frame": self.parent_frame,
            "pose": list(self.pose.to_tuple()),
            "stl": self.stl.to_dict(),
            "markers": [point.to_dict() for point in self.points],
        }

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.parent_frame not in {"frame_6", "tool"}:
            errors.append("target_body.parent_frame doit etre 'frame_6' ou 'tool'")
        errors.extend(self.stl.validate())
        seen_ids: set[str] = set()
        for index, point in enumerate(self.points):
            prefix = f"Marker Rigid Body {index + 1}"
            for error in point.validate():
                errors.append(f"{prefix}: {error}")
            point_id = point.point_id.strip()
            if point_id in seen_ids:
                errors.append(f"{prefix}: ID marker duplique '{point_id}'")
            seen_ids.add(point_id)
        return errors

    def pose_matrix(self) -> np.ndarray:
        return math_utils.pose_zyx_to_matrix(self.pose)

    def enabled_points(self) -> list[CameraTargetPoint]:
        return [point for point in self.points if point.enabled]


@dataclass(frozen=True)
class CameraConfiguration:
    camera_id: str
    name: str
    enabled: bool = True
    parent_frame: str = "truss"
    mount_pose: Pose6 = field(default_factory=Pose6.zeros)
    optical_pose: Pose6 = field(default_factory=Pose6.zeros)
    fov: CameraFov = CameraFov()
    stl: CameraStl = CameraStl()
    visual: CameraVisual = CameraVisual()

    @classmethod
    def default(cls, index: int = 1) -> "CameraConfiguration":
        return cls(
            camera_id=f"cam_{index:02d}",
            name=f"Camera {index}",
            mount_pose=Pose6(0.0, 0.0, 2500.0, 0.0, -35.0, 0.0),
        )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CameraConfiguration":
        if not isinstance(data, dict):
            raise ValueError("Une camera doit etre un objet JSON")
        mount = data.get("mount", {})
        optical = data.get("optical_frame", {})
        parent_frame = "truss"
        mount_pose_values: Any = None
        if isinstance(mount, dict):
            parent_frame = str(mount.get("parent_frame", "truss")).strip() or "truss"
            mount_pose_values = mount.get("pose")
            if mount_pose_values is None:
                position = mount.get("position_mm", mount.get("position_m", [0.0, 0.0, 0.0]))
                if "position_mm" not in mount and "position_m" in mount:
                    position = [float(v) * 1000.0 for v in position[:3]]
                rotation = mount.get("rotation_deg", [0.0, 0.0, 0.0])
                mount_pose_values = list(position[:3]) + list(rotation[:3])

        optical_pose_values: Any = None
        if isinstance(optical, dict):
            optical_pose_values = optical.get("pose")
            if optical_pose_values is None:
                position = optical.get("position_mm", optical.get("position_m", [0.0, 0.0, 0.0]))
                if "position_mm" not in optical and "position_m" in optical:
                    position = [float(v) * 1000.0 for v in position[:3]]
                rotation = optical.get("rotation_deg", [0.0, 0.0, 0.0])
                optical_pose_values = list(position[:3]) + list(rotation[:3])

        camera_id = str(data.get("id", data.get("camera_id", ""))).strip()
        return cls(
            camera_id=camera_id,
            name=str(data.get("name", camera_id or "Camera")).strip() or "Camera",
            enabled=bool(data.get("enabled", True)),
            parent_frame=parent_frame,
            mount_pose=Pose6.from_values(mount_pose_values),
            optical_pose=Pose6.from_values(optical_pose_values),
            fov=CameraFov.from_dict(data.get("fov")),
            stl=CameraStl.from_dict(data.get("stl")),
            visual=CameraVisual.from_dict(data.get("visual")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.camera_id,
            "name": self.name,
            "enabled": bool(self.enabled),
            "stl": self.stl.to_dict(),
            "mount": {
                "parent_frame": self.parent_frame,
                "pose": list(self.mount_pose.to_tuple()),
            },
            "optical_frame": {
                "pose": list(self.optical_pose.to_tuple()),
            },
            "fov": self.fov.to_dict(),
            "visual": self.visual.to_dict(),
        }

    def validate(self) -> list[str]:
        errors: list[str] = []
        if self.camera_id.strip() == "":
            errors.append("ID camera manquant")
        if self.parent_frame.strip().lower() not in {"truss", "world"}:
            errors.append("parent_frame doit etre 'truss' ou 'world'")
        errors.extend(self.fov.validate())
        errors.extend(self.stl.validate())
        return errors

    def mount_matrix(self) -> np.ndarray:
        return math_utils.pose_zyx_to_matrix(self.mount_pose)

    def optical_matrix(self) -> np.ndarray:
        return self.mount_matrix() @ math_utils.pose_zyx_to_matrix(self.optical_pose)


@dataclass(frozen=True)
class CameraVisibilityResult:
    camera_id: str
    state: CameraVisibilityState
    distance_mm: float = 0.0
    horizontal_angle_deg: float = 0.0
    vertical_angle_deg: float = 0.0
    occluder_name: str = ""
    visible_points: int = 0
    total_points: int = 0
    point_results: tuple["CameraPointVisibilityResult", ...] = field(default_factory=tuple)

    def to_dict(self) -> dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "state": self.state.value,
            "distance_mm": float(self.distance_mm),
            "horizontal_angle_deg": float(self.horizontal_angle_deg),
            "vertical_angle_deg": float(self.vertical_angle_deg),
            "occluder_name": self.occluder_name,
            "visible_points": int(self.visible_points),
            "total_points": int(self.total_points),
            "point_results": [result.to_dict() for result in self.point_results],
        }


@dataclass(frozen=True)
class CameraPointVisibilityResult:
    camera_id: str
    point_id: str
    point_name: str
    state: CameraVisibilityState
    world_xyz: tuple[float, float, float]
    distance_mm: float = 0.0
    horizontal_angle_deg: float = 0.0
    vertical_angle_deg: float = 0.0
    occluder_name: str = ""

    @classmethod
    def from_camera_result(
        cls,
        point: CameraTargetPoint,
        world_xyz: np.ndarray,
        result: CameraVisibilityResult,
    ) -> "CameraPointVisibilityResult":
        return cls(
            camera_id=result.camera_id,
            point_id=point.point_id,
            point_name=point.name,
            state=result.state,
            world_xyz=(float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2])),
            distance_mm=result.distance_mm,
            horizontal_angle_deg=result.horizontal_angle_deg,
            vertical_angle_deg=result.vertical_angle_deg,
            occluder_name=result.occluder_name,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "point_id": self.point_id,
            "point_name": self.point_name,
            "state": self.state.value,
            "world_xyz": [float(value) for value in self.world_xyz],
            "distance_mm": float(self.distance_mm),
            "horizontal_angle_deg": float(self.horizontal_angle_deg),
            "vertical_angle_deg": float(self.vertical_angle_deg),
            "occluder_name": self.occluder_name,
        }


def evaluate_camera_fov(
    camera: CameraConfiguration,
    tcp_world_xyz: np.ndarray,
    camera_world_matrix: np.ndarray,
) -> CameraVisibilityResult:
    if not camera.enabled:
        return CameraVisibilityResult(camera.camera_id, CameraVisibilityState.DISABLED)

    tcp_h = np.array([float(tcp_world_xyz[0]), float(tcp_world_xyz[1]), float(tcp_world_xyz[2]), 1.0])
    try:
        tcp_camera = math_utils.invert_homogeneous_transform(camera_world_matrix) @ tcp_h
    except ValueError:
        return CameraVisibilityResult(camera.camera_id, CameraVisibilityState.INVALID)

    x, y, z = float(tcp_camera[0]), float(tcp_camera[1]), float(tcp_camera[2])
    distance = float(np.linalg.norm([x, y, z]))
    if distance <= 1e-9:
        return CameraVisibilityResult(camera.camera_id, CameraVisibilityState.VISIBLE, distance)
    if z <= 0.0:
        return CameraVisibilityResult(camera.camera_id, CameraVisibilityState.OUT_OF_FOV, distance)
    if distance > camera.fov.range_mm:
        return CameraVisibilityResult(camera.camera_id, CameraVisibilityState.OUT_OF_RANGE, distance)

    horizontal = float(np.degrees(np.arctan2(x, z)))
    vertical = float(np.degrees(np.arctan2(y, z)))
    if abs(horizontal) > camera.fov.horizontal_deg * 0.5 or abs(vertical) > camera.fov.vertical_deg * 0.5:
        return CameraVisibilityResult(
            camera.camera_id,
            CameraVisibilityState.OUT_OF_FOV,
            distance,
            horizontal,
            vertical,
        )
    return CameraVisibilityResult(
        camera.camera_id,
        CameraVisibilityState.VISIBLE,
        distance,
        horizontal,
        vertical,
    )


class CameraConfigurationFile:
    def __init__(
        self,
        cameras: list[CameraConfiguration] | None = None,
        name: str = "Camera setup",
        target_body: CameraTargetBody | None = None,
    ) -> None:
        self.name = str(name).strip() or "Camera setup"
        self.cameras = [camera for camera in (cameras or [])]
        self.target_body = target_body or CameraTargetBody.default()

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CameraConfigurationFile":
        if not isinstance(data, dict):
            raise ValueError("La configuration camera doit etre un objet JSON")
        cameras_data = data.get("cameras", [])
        if not isinstance(cameras_data, list):
            raise ValueError("Le champ 'cameras' doit etre une liste")
        cameras = [CameraConfiguration.from_dict(item) for item in cameras_data]
        config = cls(
            cameras=cameras,
            name=str(data.get("name", "Camera setup")),
            target_body=CameraTargetBody.from_dict(data.get("target_body")),
        )
        errors = config.validate()
        if errors:
            raise ValueError("\n".join(errors))
        return config

    @classmethod
    def load(cls, file_path: str) -> "CameraConfigurationFile":
        with open(file_path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        return cls.from_dict(data)

    @classmethod
    def from_camera_model(cls, model: "CameraModel") -> "CameraConfigurationFile":
        return cls(
            cameras=model.get_cameras(),
            name=model.get_setup_name(),
            target_body=model.get_target_body(),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "target_body": self.target_body.to_dict(),
            "cameras": [camera.to_dict() for camera in self.cameras],
        }

    def save(self, file_path: str) -> None:
        directory = os.path.dirname(os.path.abspath(file_path))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(file_path, "w", encoding="utf-8") as handle:
            json.dump(self.to_dict(), handle, indent=2, ensure_ascii=False)

    def validate(self) -> list[str]:
        errors: list[str] = []
        for error in self.target_body.validate():
            errors.append(f"Rigid Body: {error}")
        seen_ids: set[str] = set()
        for index, camera in enumerate(self.cameras):
            prefix = f"Camera {index + 1}"
            for error in camera.validate():
                errors.append(f"{prefix}: {error}")
            camera_id = camera.camera_id.strip()
            if camera_id in seen_ids:
                errors.append(f"{prefix}: ID camera duplique '{camera_id}'")
            seen_ids.add(camera_id)
        return errors


class CameraModel(QObject):
    cameras_changed = pyqtSignal()
    visibility_changed = pyqtSignal()

    DEFAULT_CAMERA_DIRECTORY = "./user_data/cameras"

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._setup_name = "Camera setup"
        self._current_file_path = ""
        self._cameras: list[CameraConfiguration] = []
        self._target_body = CameraTargetBody.default()
        self._visibility_results: dict[str, CameraVisibilityResult] = {}

    def get_setup_name(self) -> str:
        return self._setup_name

    def set_setup_name(self, value: str) -> None:
        normalized = str(value).strip() or "Camera setup"
        if normalized == self._setup_name:
            return
        self._setup_name = normalized
        self.cameras_changed.emit()

    def get_current_file_path(self) -> str:
        return self._current_file_path

    def set_current_file_path(self, value: str | None) -> None:
        self._current_file_path = "" if value is None else str(value).strip()

    def get_cameras(self) -> list[CameraConfiguration]:
        return list(self._cameras)

    def get_target_body(self) -> CameraTargetBody:
        return self._target_body

    def set_target_body(self, value: CameraTargetBody) -> None:
        if not isinstance(value, CameraTargetBody):
            raise TypeError("value must be CameraTargetBody")
        if value == self._target_body:
            return
        self._target_body = value
        self.cameras_changed.emit()

    def set_cameras(self, values: list[CameraConfiguration]) -> None:
        if not all(isinstance(camera, CameraConfiguration) for camera in values):
            raise TypeError("values must contain CameraConfiguration")
        self._cameras = list(values)
        self._visibility_results = {
            camera_id: result
            for camera_id, result in self._visibility_results.items()
            if any(camera.camera_id == camera_id for camera in self._cameras)
        }
        self.cameras_changed.emit()

    def add_camera(self, camera: CameraConfiguration | None = None) -> None:
        next_index = len(self._cameras) + 1
        candidate = camera or CameraConfiguration.default(next_index)
        existing_ids = {cam.camera_id for cam in self._cameras}
        if candidate.camera_id in existing_ids:
            base = candidate.camera_id or "cam"
            suffix = 1
            while f"{base}_{suffix}" in existing_ids:
                suffix += 1
            candidate = CameraConfiguration.from_dict({**candidate.to_dict(), "id": f"{base}_{suffix}"})
        self._cameras.append(candidate)
        self.cameras_changed.emit()

    def update_camera(self, index: int, camera: CameraConfiguration) -> None:
        if not isinstance(camera, CameraConfiguration):
            raise TypeError("camera must be CameraConfiguration")
        if not (0 <= index < len(self._cameras)):
            return
        self._cameras[index] = camera
        self.cameras_changed.emit()

    def remove_camera(self, index: int) -> None:
        if not (0 <= index < len(self._cameras)):
            return
        removed = self._cameras.pop(index)
        self._visibility_results.pop(removed.camera_id, None)
        self.cameras_changed.emit()

    def load_configuration_file(self, config: CameraConfigurationFile, file_path: str | None = None) -> None:
        self._setup_name = config.name
        self._cameras = list(config.cameras)
        self._target_body = config.target_body
        self._current_file_path = "" if file_path is None else str(file_path).strip()
        self._visibility_results = {}
        self.cameras_changed.emit()

    def get_visibility_results(self) -> dict[str, CameraVisibilityResult]:
        return dict(self._visibility_results)

    def set_visibility_results(self, results: list[CameraVisibilityResult]) -> None:
        self._visibility_results = {result.camera_id: result for result in results}
        self.visibility_changed.emit()
