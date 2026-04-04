from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from typing import Any

from models.reference_frame import ReferenceFrame


@dataclass
class ViewerDisplayState:
    cad_visible: bool = True
    transparency_enabled: bool = False
    show_axes: bool = True
    frames_visibility: list[bool] = field(default_factory=list)
    workspace_frames_visibility: list[bool] = field(default_factory=list)
    workspace_tcp_zones_visible: bool = True
    workspace_collision_zones_visible: bool = True
    robot_colliders_visible: bool = True
    tool_colliders_visible: bool = True

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "ViewerDisplayState":
        payload = data if isinstance(data, dict) else {}
        raw_frames = payload.get("frames_visibility", [])
        frames_visibility = [bool(value) for value in raw_frames] if isinstance(raw_frames, list) else []
        raw_workspace_frames = payload.get("workspace_frames_visibility", [])
        workspace_frames_visibility = [bool(value) for value in raw_workspace_frames] if isinstance(raw_workspace_frames, list) else []
        return cls(
            cad_visible=bool(payload.get("cad_visible", True)),
            transparency_enabled=bool(payload.get("transparency_enabled", False)),
            show_axes=bool(payload.get("show_axes", True)),
            frames_visibility=frames_visibility,
            workspace_frames_visibility=workspace_frames_visibility,
            workspace_tcp_zones_visible=bool(payload.get("workspace_tcp_zones_visible", True)),
            workspace_collision_zones_visible=bool(payload.get("workspace_collision_zones_visible", True)),
            robot_colliders_visible=bool(payload.get("robot_colliders_visible", True)),
            tool_colliders_visible=bool(payload.get("tool_colliders_visible", True)),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class AppSessionFile:
    robot_config_path: str = ""
    tool_profile_path: str = ""
    workspace_path: str = ""
    viewer_state: ViewerDisplayState = field(default_factory=ViewerDisplayState)
    cartesian_control_frame: str = ReferenceFrame.BASE.value
    jog_tcp_display_frame: str = ReferenceFrame.BASE.value
    trajectory_display_frame: str = ReferenceFrame.BASE.value

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "AppSessionFile":
        if not isinstance(data, dict):
            raise TypeError("La session applicative doit etre un dictionnaire JSON.")
        return cls(
            robot_config_path="" if data.get("robot_config_path") is None else str(data.get("robot_config_path")),
            tool_profile_path="" if data.get("tool_profile_path") is None else str(data.get("tool_profile_path")),
            workspace_path="" if data.get("workspace_path") is None else str(data.get("workspace_path")),
            viewer_state=ViewerDisplayState.from_dict(data.get("viewer_state")),
            cartesian_control_frame=ReferenceFrame.from_value(data.get("cartesian_control_frame")).value,
            jog_tcp_display_frame=ReferenceFrame.from_value(data.get("jog_tcp_display_frame")).value,
            trajectory_display_frame=ReferenceFrame.from_value(data.get("trajectory_display_frame")).value,
        )

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["viewer_state"] = self.viewer_state.to_dict()
        return payload

    def save(self, file_path: str) -> None:
        with open(file_path, "w", encoding="utf-8") as file:
            json.dump(self.to_dict(), file, indent=4)

    @classmethod
    def load(cls, file_path: str) -> "AppSessionFile":
        with open(file_path, "r", encoding="utf-8") as file:
            data = json.load(file)
        return cls.from_dict(data)
