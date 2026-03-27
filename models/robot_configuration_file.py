from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Any, TYPE_CHECKING

from utils.mgi import MgiConfigKey

if TYPE_CHECKING:
    from models.robot_model import RobotModel

DEFAULT_AXIS_LIMITS: list[tuple[float, float]] = [
    (-170.0, 170.0),
    (-190.0, 45.0),
    (-120.0, 156.0),
    (-185.0, 185.0),
    (-120.0, 120.0),
    (-350.0, 350.0),
]
DEFAULT_AXIS_SPEED_LIMITS: list[float] = [300.0, 225.0, 255.0, 381.0, 311.0, 492.0]
DEFAULT_AXIS_JERK_LIMITS: list[float] = [6000.0, 5000.0, 5000.0, 7500.0, 6500.0, 9000.0]
DEFAULT_ROBOT_CAD_MODELS: list[str] = [f"./robot_stl/rocky{i}.stl" for i in range(7)]
DEFAULT_TOOL_PROFILES_DIRECTORY: str = "./configurations/tools"
DEFAULT_SELECTED_TOOL_PROFILE: str = ""


@dataclass
class RobotConfigurationFile:
    """Representation d'un fichier de configuration robot."""

    name: str = ""
    dh: list[list[float]] = field(default_factory=lambda: [[0.0, 0.0, 0.0, 0.0] for _ in range(6)])
    corr: list[list[float]] = field(default_factory=lambda: [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(6)])
    axis_limits: list[tuple[float, float]] = field(default_factory=lambda: list(DEFAULT_AXIS_LIMITS))
    axis_speed_limits: list[float] = field(default_factory=lambda: list(DEFAULT_AXIS_SPEED_LIMITS))
    axis_jerk_limits: list[float] = field(default_factory=lambda: list(DEFAULT_AXIS_JERK_LIMITS))
    axis_reversed: list[int] = field(default_factory=lambda: [1] * 6)
    joint_weights: list[float] = field(default_factory=lambda: [1.0] * 6)
    allowed_configs: set[MgiConfigKey] = field(default_factory=lambda: set(MgiConfigKey))
    home_position: list[float] = field(default_factory=lambda: [0.0, -90.0, 90.0, 0.0, 90.0, 0.0])
    position_zero: list[float] = field(default_factory=lambda: [0.0, -90.0, 90.0, 0.0, 0.0, 0.0])
    position_transport: list[float] = field(default_factory=lambda: [0.0, -105.0, 156.0, 0.0, 120.0, 0.0])
    robot_cad_models: list[str] = field(default_factory=lambda: list(DEFAULT_ROBOT_CAD_MODELS))
    tool_profiles_directory: str = DEFAULT_TOOL_PROFILES_DIRECTORY
    selected_tool_profile: str = DEFAULT_SELECTED_TOOL_PROFILE

    # Tracks which fields were present when loaded from JSON.
    present_fields: set[str] = field(default_factory=set, repr=False)

    @staticmethod
    def _safe_float(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @classmethod
    def _parse_matrix(
        cls,
        rows: Any,
        row_count: int,
        col_count: int,
        default_value: float = 0.0,
    ) -> list[list[float]]:
        matrix: list[list[float]] = []
        if not isinstance(rows, list):
            rows = []

        for row in rows[:row_count]:
            if not isinstance(row, list):
                row = []
            parsed_row = [cls._safe_float(v, default_value) for v in row[:col_count]]
            while len(parsed_row) < col_count:
                parsed_row.append(default_value)
            matrix.append(parsed_row)

        while len(matrix) < row_count:
            matrix.append([default_value] * col_count)
        return matrix

    @classmethod
    def _parse_float_list(cls, values: Any, length: int, default_value: float = 0.0) -> list[float]:
        parsed: list[float] = []
        if not isinstance(values, list):
            values = []

        for value in values[:length]:
            parsed.append(cls._safe_float(value, default_value))

        while len(parsed) < length:
            parsed.append(default_value)
        return parsed

    @classmethod
    def _parse_string_list(cls, values: Any, default_values: list[str]) -> list[str]:
        if not isinstance(values, list):
            return list(default_values)
        parsed = [str(value) for value in values]
        return parsed if parsed else list(default_values)

    @classmethod
    def _parse_axis_limits(cls, values: Any) -> list[tuple[float, float]]:
        limits: list[tuple[float, float]] = []
        if not isinstance(values, list):
            values = []

        for i, item in enumerate(values[:6]):
            default_min, default_max = DEFAULT_AXIS_LIMITS[i]
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                limits.append((cls._safe_float(item[0], default_min), cls._safe_float(item[1], default_max)))
            else:
                limits.append((default_min, default_max))

        while len(limits) < 6:
            limits.append(DEFAULT_AXIS_LIMITS[len(limits)])
        return limits

    @classmethod
    def _parse_axis_speed_limits(cls, values: Any) -> list[float]:
        speed_limits: list[float] = []
        if not isinstance(values, list):
            values = []

        for i, item in enumerate(values[:6]):
            speed_limits.append(cls._safe_float(item, DEFAULT_AXIS_SPEED_LIMITS[i]))

        while len(speed_limits) < 6:
            speed_limits.append(DEFAULT_AXIS_SPEED_LIMITS[len(speed_limits)])
        return speed_limits

    @classmethod
    def _parse_axis_jerk_limits(cls, values: Any) -> list[float]:
        jerk_limits: list[float] = []
        if not isinstance(values, list):
            values = []

        for i, item in enumerate(values[:6]):
            jerk_limits.append(cls._safe_float(item, DEFAULT_AXIS_JERK_LIMITS[i]))

        while len(jerk_limits) < 6:
            jerk_limits.append(DEFAULT_AXIS_JERK_LIMITS[len(jerk_limits)])
        return jerk_limits

    @classmethod
    def _parse_axis_reversed(cls, values: Any) -> list[int]:
        parsed = cls._parse_float_list(values, 6, 1.0)
        return [-1 if int(v) == -1 else 1 for v in parsed]

    @staticmethod
    def _parse_mgi_config_key(raw_value: Any) -> MgiConfigKey | None:
        if isinstance(raw_value, MgiConfigKey):
            return raw_value

        if isinstance(raw_value, str):
            key_name = raw_value.strip().upper()
            if key_name in MgiConfigKey.__members__:
                return MgiConfigKey[key_name]
            try:
                return MgiConfigKey(int(key_name))
            except (ValueError, TypeError):
                return None

        if isinstance(raw_value, (int, float)):
            try:
                return MgiConfigKey(int(raw_value))
            except (ValueError, TypeError):
                return None

        return None

    @classmethod
    def _parse_allowed_configs(cls, raw_values: Any) -> set[MgiConfigKey]:
        parsed: set[MgiConfigKey] = set()

        if isinstance(raw_values, dict):
            for key, is_allowed in raw_values.items():
                if is_allowed:
                    parsed_key = cls._parse_mgi_config_key(key)
                    if parsed_key is not None:
                        parsed.add(parsed_key)
            return parsed

        if isinstance(raw_values, list):
            for key in raw_values:
                parsed_key = cls._parse_mgi_config_key(key)
                if parsed_key is not None:
                    parsed.add(parsed_key)
            return parsed

        return set(MgiConfigKey)

    @classmethod
    def from_robot_model(cls, robot_model: RobotModel) -> RobotConfigurationFile:
        return cls(
            name=robot_model.get_robot_name(),
            dh=[row[:] for row in robot_model.get_dh_params()[:6]],
            corr=[row[:] for row in robot_model.get_corrections()],
            axis_limits=[tuple(v) for v in robot_model.get_axis_limits()],
            axis_speed_limits=robot_model.get_axis_speed_limits(),
            axis_jerk_limits=robot_model.get_axis_jerk_limits(),
            axis_reversed=robot_model.get_axis_reversed(),
            joint_weights=robot_model.get_joint_weights(),
            allowed_configs=robot_model.get_allowed_configurations(),
            home_position=robot_model.get_home_position(),
            position_zero=robot_model.get_position_zero(),
            position_transport=robot_model.get_position_transport(),
            robot_cad_models=robot_model.get_robot_cad_models(),
            tool_profiles_directory=robot_model.get_tool_profiles_directory(),
            selected_tool_profile=robot_model.get_selected_tool_profile(),
            present_fields={
                "name",
                "dh",
                "corr",
                "axis_limits",
                "axis_speed_limits",
                "axis_jerk_limits",
                "axis_reversed",
                "joint_weights",
                "allowed_configs",
                "home_position",
                "position_zero",
                "position_transport",
                "robot_cad_models",
                "tool_profiles_directory",
                "selected_tool_profile",
            },
        )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RobotConfigurationFile:
        if not isinstance(data, dict):
            raise TypeError("La configuration robot doit etre un dictionnaire JSON.")

        present_fields = set(data.keys())
        name_raw = data.get("name", "")
        if isinstance(name_raw, list) and name_raw:
            name = str(name_raw[0])
        elif isinstance(name_raw, str):
            name = name_raw
        else:
            name = ""

        allowed_raw_key = None
        if "allowed_configs" in data:
            allowed_raw_key = "allowed_configs"
        elif "allowed_configurations" in data:
            allowed_raw_key = "allowed_configurations"
            present_fields.add("allowed_configs")

        if "robot_cad_models" not in data and "robot_cad_files" in data:
            present_fields.add("robot_cad_models")
        present_fields.add("tool_profiles_directory")
        present_fields.add("selected_tool_profile")

        allowed_configs = cls._parse_allowed_configs(data.get(allowed_raw_key)) if allowed_raw_key else set(MgiConfigKey)
        tool_profiles_directory = data.get("tool_profiles_directory", DEFAULT_TOOL_PROFILES_DIRECTORY)
        selected_tool_profile = data.get("selected_tool_profile", DEFAULT_SELECTED_TOOL_PROFILE)

        return cls(
            name=name,
            dh=cls._parse_matrix(data.get("dh"), 6, 4, 0.0),
            corr=cls._parse_matrix(data.get("corr"), 6, 6, 0.0),
            axis_limits=cls._parse_axis_limits(data.get("axis_limits")),
            axis_speed_limits=cls._parse_axis_speed_limits(data.get("axis_speed_limits")),
            axis_jerk_limits=cls._parse_axis_jerk_limits(data.get("axis_jerk_limits")),
            axis_reversed=cls._parse_axis_reversed(data.get("axis_reversed")),
            joint_weights=cls._parse_float_list(data.get("joint_weights"), 6, 1.0),
            allowed_configs=allowed_configs,
            home_position=cls._parse_float_list(data.get("home_position"), 6, 0.0),
            position_zero=cls._parse_float_list(data.get("position_zero"), 6, 0.0),
            position_transport=cls._parse_float_list(data.get("position_transport"), 6, 0.0),
            robot_cad_models=cls._parse_string_list(
                data.get("robot_cad_models", data.get("robot_cad_files")),
                DEFAULT_ROBOT_CAD_MODELS,
            ),
            tool_profiles_directory="" if tool_profiles_directory is None else str(tool_profiles_directory),
            selected_tool_profile="" if selected_tool_profile is None else str(selected_tool_profile),
            present_fields=present_fields,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": [self.name],
            "dh": [[str(val) for val in row] for row in self.dh[:6]],
            "corr": [[str(val) for val in row] for row in self.corr[:6]],
            "axis_limits": self.axis_limits[:6],
            "axis_speed_limits": self.axis_speed_limits[:6],
            "axis_jerk_limits": self.axis_jerk_limits[:6],
            "axis_reversed": self.axis_reversed[:6],
            "joint_weights": self.joint_weights[:6],
            "allowed_configs": [cfg.name for cfg in MgiConfigKey if cfg in self.allowed_configs],
            "home_position": self.home_position[:6],
            "position_zero": self.position_zero[:6],
            "position_transport": self.position_transport[:6],
            "robot_cad_models": [str(path) for path in self.robot_cad_models],
            "tool_profiles_directory": str(self.tool_profiles_directory),
            "selected_tool_profile": str(self.selected_tool_profile),
        }

    def apply_to_robot_model(self, robot_model: RobotModel) -> None:
        if "name" in self.present_fields:
            robot_model.robot_name = self.name
        if "dh" in self.present_fields:
            robot_model.set_dh_params(self.dh)
        if "corr" in self.present_fields:
            robot_model._set_corrections(self.corr)
        if "axis_reversed" in self.present_fields:
            robot_model.set_axis_reversed(self.axis_reversed)
        if "joint_weights" in self.present_fields:
            robot_model.set_joint_weights(self.joint_weights)
        if "allowed_configs" in self.present_fields:
            robot_model.set_allowed_configurations(self.allowed_configs)
        if "axis_limits" in self.present_fields:
            robot_model.set_axis_limits(self.axis_limits)
        if "axis_speed_limits" in self.present_fields:
            robot_model.set_axis_speed_limits(self.axis_speed_limits)
        if "axis_jerk_limits" in self.present_fields:
            robot_model.set_axis_jerk_limits(self.axis_jerk_limits)
        if "home_position" in self.present_fields:
            robot_model.set_home_position(self.home_position)
        if "position_zero" in self.present_fields:
            robot_model.set_position_zero(self.position_zero)
        if "position_transport" in self.present_fields:
            robot_model.set_position_transport(self.position_transport)
        if "robot_cad_models" in self.present_fields:
            robot_model.set_robot_cad_models(self.robot_cad_models)
        if "tool_profiles_directory" in self.present_fields:
            robot_model.set_tool_profiles_directory(self.tool_profiles_directory)
        if "selected_tool_profile" in self.present_fields:
            robot_model.set_selected_tool_profile(self.selected_tool_profile)

    def save(self, file_path: str) -> None:
        with open(file_path, "w") as file:
            json.dump(self.to_dict(), file, indent=4)

    @classmethod
    def load(cls, file_path: str) -> RobotConfigurationFile:
        with open(file_path, "r") as file:
            data = json.load(file)
        return cls.from_dict(data)
