from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from typing import Any

from models.program_generation_settings import ProgramGenerationSettings
from models.reference_frame import ReferenceFrame


@dataclass
class ProgramBaseConfigState:
    base_source: str = "MANUAL"
    base_offset: list[float] = field(default_factory=lambda: [0.0] * 6)

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "ProgramBaseConfigState":
        payload = data if isinstance(data, dict) else {}
        raw_offset = payload.get("base_offset", [0.0] * 6)
        base_offset = (
            [float(v) for v in raw_offset]
            if isinstance(raw_offset, list) and len(raw_offset) == 6
            else [0.0] * 6
        )
        return cls(
            base_source=str(payload.get("base_source", "MANUAL")),
            base_offset=base_offset,
        )

    def to_dict(self) -> dict[str, Any]:
        return {"base_source": self.base_source, "base_offset": list(self.base_offset)}


@dataclass
class ViewerThemeState:
    background_mode: str = "solid"
    background_primary_color: str = "#2D2D30FF"
    background_secondary_color: str = "#0F0F12FF"
    background_gradient_direction: str = "vertical"
    text_color: str = "#E6E6E6FF"
    accent_color: str = "#FF8C00FF"
    grid_size: int = 4000
    grid_spacing: int = 200
    grid_color: str = "#96969664"

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> "ViewerThemeState":
        payload = data if isinstance(data, dict) else {}
        return cls(
            background_mode=str(payload.get("background_mode", "solid")),
            background_primary_color=str(payload.get("background_primary_color", "#2D2D30FF")),
            background_secondary_color=str(payload.get("background_secondary_color", "#0F0F12FF")),
            background_gradient_direction=str(payload.get("background_gradient_direction", "vertical")),
            text_color=str(payload.get("text_color", "#E6E6E6FF")),
            accent_color=str(payload.get("accent_color", "#FF8C00FF")),
            grid_size=max(1, int(payload.get("grid_size", 4000))),
            grid_spacing=max(1, int(payload.get("grid_spacing", 200))),
            grid_color=str(payload.get("grid_color", "#96969664")),
        )

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


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
    ext_axes_transparency_enabled: bool = False
    workspace_transparency_enabled: bool = True
    tooling_frames_visible: bool = True
    workpiece_frame_visible: bool = True
    theme: ViewerThemeState = field(default_factory=ViewerThemeState)
    selected_theme_name: str = ""

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
            ext_axes_transparency_enabled=bool(payload.get("ext_axes_transparency_enabled", False)),
            workspace_transparency_enabled=bool(payload.get("workspace_transparency_enabled", True)),
            tooling_frames_visible=bool(payload.get("tooling_frames_visible", True)),
            workpiece_frame_visible=bool(payload.get("workpiece_frame_visible", True)),
            theme=ViewerThemeState.from_dict(payload.get("theme")),
            selected_theme_name="" if payload.get("selected_theme_name") is None else str(payload.get("selected_theme_name")),
        )

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["theme"] = self.theme.to_dict()
        return payload


@dataclass
class AppSessionFile:
    robot_config_path: str = ""
    tool_profile_path: str = ""
    workspace_path: str = ""
    external_axes_config_path: str = ""
    camera_config_path: str = ""
    viewer_state: ViewerDisplayState = field(default_factory=ViewerDisplayState)
    external_axes_data: dict = field(default_factory=dict)
    workpiece_data: dict = field(default_factory=dict)
    tooling_data: dict = field(default_factory=dict)
    program_base_config: ProgramBaseConfigState = field(default_factory=ProgramBaseConfigState)
    program_generation_settings: ProgramGenerationSettings = field(
        default_factory=ProgramGenerationSettings
    )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "AppSessionFile":
        if not isinstance(data, dict):
            raise TypeError("La session applicative doit etre un dictionnaire JSON.")
        return cls(
            robot_config_path="" if data.get("robot_config_path") is None else str(data.get("robot_config_path")),
            tool_profile_path="" if data.get("tool_profile_path") is None else str(data.get("tool_profile_path")),
            workspace_path="" if data.get("workspace_path") is None else str(data.get("workspace_path")),
            external_axes_config_path=""
            if data.get("external_axes_config_path") is None
            else str(data.get("external_axes_config_path")),
            camera_config_path="" if data.get("camera_config_path") is None else str(data.get("camera_config_path")),
            viewer_state=ViewerDisplayState.from_dict(data.get("viewer_state")),
            external_axes_data=data.get("external_axes_data") or {},
            workpiece_data=data.get("workpiece_data") or {},
            tooling_data=data.get("tooling_data") or {},
            program_base_config=ProgramBaseConfigState.from_dict(data.get("program_base_config")),
            program_generation_settings=ProgramGenerationSettings.from_dict(
                data.get("program_generation_settings")
            ),
        )

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["viewer_state"] = self.viewer_state.to_dict()
        payload["program_base_config"] = self.program_base_config.to_dict()
        payload["program_generation_settings"] = self.program_generation_settings.to_dict()
        return payload

    def save(self, file_path: str) -> None:
        with open(file_path, "w", encoding="utf-8") as file:
            json.dump(self.to_dict(), file, indent=4)

    @classmethod
    def load(cls, file_path: str) -> "AppSessionFile":
        with open(file_path, "r", encoding="utf-8") as file:
            data = json.load(file)
        return cls.from_dict(data)
