from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from typing import Any, TYPE_CHECKING

from models.collider_models import default_axis_colliders
from models.primitive_collider_models import (
    AxisDirection,
    RobotAxisColliderData,
)
from models.types import CadColorPalette, XYZ3
from utils.math_utils import safe_float
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
DEFAULT_AXIS_ACCEL_LIMITS: list[float] = [
    round(math.sqrt(speed * jerk), 3)
    for speed, jerk in zip(DEFAULT_AXIS_SPEED_LIMITS, DEFAULT_AXIS_JERK_LIMITS)
]
DEFAULT_AXIS_COLLIDERS: list[RobotAxisColliderData] = default_axis_colliders(6)
DEFAULT_CARTESIAN_SLIDER_LIMITS_XYZ: list[tuple[float, float]] = [
    (-1000.0, 1000.0),
    (-1000.0, 1000.0),
    (-1000.0, 1000.0),
]
DEFAULT_ROBOT_CAD_MODELS: list[str] = [f"./default_data/robots_stl/rocky{i}.stl" for i in range(7)]
DEFAULT_ROBOT_CAD_COLORS: list[str] = [""] * 7


def _parse_axis_direction(value: Any) -> AxisDirection:
    if not isinstance(value, str):
        raise TypeError("axis collider direction_axis must be a string")
    return AxisDirection(value)


def _parse_xyz3_list(value: Any) -> XYZ3:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("axis collider offset_xyz must be a list of 3 values")
    return XYZ3(safe_float(value[0], 0.0), safe_float(value[1], 0.0), safe_float(value[2], 0.0))


def _parse_axis_collider(value: Any, index: int) -> RobotAxisColliderData:
    if not isinstance(value, dict):
        raise TypeError("axis collider must be a JSON object")
    required = ("axis", "enabled", "radius", "height", "direction_axis", "offset_xyz")
    missing = [key for key in required if key not in value]
    if missing:
        raise ValueError(f"axis collider is missing keys: {', '.join(missing)}")
    return RobotAxisColliderData(
        axis_index=int(value["axis"]),
        enabled=bool(value["enabled"]),
        radius=safe_float(value["radius"], 0.0),
        height=safe_float(value["height"], 0.0),
        direction_axis=_parse_axis_direction(value["direction_axis"]),
        offset_xyz=_parse_xyz3_list(value["offset_xyz"]),
    )


def _parse_axis_colliders(value: Any) -> list[RobotAxisColliderData]:
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError("axis_colliders must be a list of 6 colliders")
    return [_parse_axis_collider(collider, index) for index, collider in enumerate(value)]


def _axis_collider_to_dict(collider: RobotAxisColliderData) -> dict[str, Any]:
    return {
        "axis": collider.axis_index,
        "enabled": collider.enabled,
        "radius": float(collider.radius),
        "height": float(collider.height),
        "direction_axis": collider.direction_axis.value,
        "offset_xyz": collider.offset_xyz.to_list(),
    }


@dataclass
class RobotConfigurationFile:
    """Representation d'un fichier de configuration robot."""

    name: str = ""
    dh: list[list[float]] = field(default_factory=lambda: [[0.0, 0.0, 0.0, 0.0] for _ in range(6)])
    dh_measured: list[list[float]] = field(default_factory=lambda: [[0.0, 0.0, 0.0, 0.0] for _ in range(6)])
    dh_measured_enabled: bool = False
    corr: list[list[float]] = field(default_factory=lambda: [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] for _ in range(6)])
    axis_limits: list[tuple[float, float]] = field(default_factory=lambda: list(DEFAULT_AXIS_LIMITS))
    cartesian_slider_limits_xyz: list[tuple[float, float]] = field(
        default_factory=lambda: list(DEFAULT_CARTESIAN_SLIDER_LIMITS_XYZ)
    )
    axis_speed_limits: list[float] = field(default_factory=lambda: list(DEFAULT_AXIS_SPEED_LIMITS))
    axis_accel_limits: list[float] = field(default_factory=lambda: list(DEFAULT_AXIS_ACCEL_LIMITS))
    axis_jerk_limits: list[float] = field(default_factory=lambda: list(DEFAULT_AXIS_JERK_LIMITS))
    axis_colliders: list[RobotAxisColliderData] = field(
        default_factory=lambda: [collider.copy() for collider in DEFAULT_AXIS_COLLIDERS]
    )
    axis_reversed: list[int] = field(default_factory=lambda: [1] * 6)
    joint_weights: list[float] = field(default_factory=lambda: [1.0] * 6)
    allowed_configs: set[MgiConfigKey] = field(default_factory=lambda: set(MgiConfigKey))
    home_position: list[float] = field(default_factory=lambda: [0.0, -90.0, 90.0, 0.0, 90.0, 0.0])
    position_zero: list[float] = field(default_factory=lambda: [0.0, -90.0, 90.0, 0.0, 0.0, 0.0])
    position_calibration: list[float] = field(default_factory=lambda: [0.0, -105.0, 156.0, 0.0, 120.0, 0.0])
    robot_cad_models: list[str] = field(default_factory=lambda: list(DEFAULT_ROBOT_CAD_MODELS))
    robot_cad_colors: list[str] = field(default_factory=lambda: list(DEFAULT_ROBOT_CAD_COLORS))

    # Tracks which fields were present when loaded from JSON.
    present_fields: set[str] = field(default_factory=set, repr=False)

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
            parsed_row = [safe_float(v, default_value) for v in row[:col_count]]
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
            parsed.append(safe_float(value, default_value))

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
                limits.append((safe_float(item[0], default_min), safe_float(item[1], default_max)))
            else:
                limits.append((default_min, default_max))

        while len(limits) < 6:
            limits.append(DEFAULT_AXIS_LIMITS[len(limits)])
        return limits

    @classmethod
    def _parse_cartesian_slider_limits_xyz(cls, values: Any) -> list[tuple[float, float]]:
        limits: list[tuple[float, float]] = []
        if not isinstance(values, list):
            values = []

        for i, item in enumerate(values[:3]):
            default_min, default_max = DEFAULT_CARTESIAN_SLIDER_LIMITS_XYZ[i]
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                limits.append((safe_float(item[0], default_min), safe_float(item[1], default_max)))
            else:
                limits.append((default_min, default_max))

        while len(limits) < 3:
            limits.append(DEFAULT_CARTESIAN_SLIDER_LIMITS_XYZ[len(limits)])
        return limits

    @classmethod
    def _parse_axis_speed_limits(cls, values: Any) -> list[float]:
        speed_limits: list[float] = []
        if not isinstance(values, list):
            values = []

        for i, item in enumerate(values[:6]):
            speed_limits.append(safe_float(item, DEFAULT_AXIS_SPEED_LIMITS[i]))

        while len(speed_limits) < 6:
            speed_limits.append(DEFAULT_AXIS_SPEED_LIMITS[len(speed_limits)])
        return speed_limits

    @classmethod
    def _calculate_axis_accel_limits(cls, speed_limits: list[float], jerk_limits: list[float]) -> list[float]:
        accel_limits: list[float] = []
        for index in range(6):
            speed = safe_float(speed_limits[index] if index < len(speed_limits) else 0.0, 0.0)
            jerk = safe_float(jerk_limits[index] if index < len(jerk_limits) else 0.0, 0.0)
            accel_limits.append(round(math.sqrt(max(0.0, speed) * max(0.0, jerk)), 3))
        return accel_limits

    @classmethod
    def _parse_axis_accel_limits(
        cls,
        values: Any,
        speed_limits: list[float],
        jerk_limits: list[float],
    ) -> list[float]:
        defaults = cls._calculate_axis_accel_limits(speed_limits, jerk_limits)
        accel_limits: list[float] = []
        if not isinstance(values, list):
            values = []

        for i, item in enumerate(values[:6]):
            accel_limits.append(max(0.0, safe_float(item, defaults[i])))

        while len(accel_limits) < 6:
            accel_limits.append(defaults[len(accel_limits)])
        return accel_limits

    @classmethod
    def _parse_axis_jerk_limits(cls, values: Any) -> list[float]:
        jerk_limits: list[float] = []
        if not isinstance(values, list):
            values = []

        for i, item in enumerate(values[:6]):
            jerk_limits.append(safe_float(item, DEFAULT_AXIS_JERK_LIMITS[i]))

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
            cartesian_slider_limits_xyz=[tuple(v) for v in robot_model.get_cartesian_slider_limits_xyz()],
            axis_speed_limits=robot_model.get_axis_speed_limits(),
            axis_accel_limits=robot_model.get_axis_accel_limits(),
            axis_jerk_limits=robot_model.get_axis_jerk_limits(),
            axis_colliders=robot_model.get_axis_collider_data(),
            axis_reversed=robot_model.get_axis_reversed(),
            joint_weights=robot_model.get_joint_weights(),
            allowed_configs=robot_model.get_allowed_configurations(),
            home_position=robot_model.get_home_position(),
            position_zero=robot_model.get_position_zero(),
            position_calibration=robot_model.get_position_calibration(),
            robot_cad_models=robot_model.get_robot_cad_models(),
            robot_cad_colors=CadColorPalette(robot_model.get_robot_cad_colors()).to_hex_list(),
            dh_measured=[row[:] for row in robot_model.get_measured_dh_params()[:6]],
            dh_measured_enabled=robot_model.get_measured_dh_enabled(),
            present_fields={
                "name",
                "dh",
                "dh_measured",
                "dh_measured_enabled",
                "corr",
                "axis_limits",
                "cartesian_slider_limits_xyz",
                "axis_speed_limits",
                "axis_accel_limits",
                "axis_jerk_limits",
                "axis_colliders",
                "axis_reversed",
                "joint_weights",
                "allowed_configs",
                "home_position",
                "position_zero",
                "position_calibration",
                "robot_cad_models",
                "robot_cad_colors",
            },
        )

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RobotConfigurationFile:
        if not isinstance(data, dict):
            raise TypeError("La configuration robot doit etre un dictionnaire JSON.")

        present_fields = set(data.keys())
        required_fields = {
            "name",
            "dh",
            "dh_measured",
            "dh_measured_enabled",
            "corr",
            "axis_limits",
            "cartesian_slider_limits_xyz",
            "axis_speed_limits",
            "axis_accel_limits",
            "axis_jerk_limits",
            "axis_colliders",
            "axis_reversed",
            "joint_weights",
            "allowed_configs",
            "home_position",
            "position_zero",
            "position_calibration",
            "robot_cad_models",
        }
        missing = sorted(required_fields - present_fields)
        if missing:
            raise ValueError(f"La configuration robot est incomplete: {', '.join(missing)}")

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
        if "robot_cad_colors" not in present_fields:
            present_fields.add("robot_cad_colors")

        allowed_configs = cls._parse_allowed_configs(data.get(allowed_raw_key)) if allowed_raw_key else set(MgiConfigKey)
        axis_speed_limits = cls._parse_axis_speed_limits(data.get("axis_speed_limits"))
        axis_jerk_limits = cls._parse_axis_jerk_limits(data.get("axis_jerk_limits"))
        if "axis_accel_limits" not in present_fields:
            present_fields.add("axis_accel_limits")

        return cls(
            name=name,
            dh=cls._parse_matrix(data.get("dh"), 6, 4, 0.0),
            corr=cls._parse_matrix(data.get("corr"), 6, 6, 0.0),
            axis_limits=cls._parse_axis_limits(data.get("axis_limits")),
            cartesian_slider_limits_xyz=cls._parse_cartesian_slider_limits_xyz(
                data.get("cartesian_slider_limits_xyz")
            ),
            axis_speed_limits=axis_speed_limits,
            axis_accel_limits=cls._parse_axis_accel_limits(
                data.get("axis_accel_limits"),
                axis_speed_limits,
                axis_jerk_limits,
            ),
            axis_jerk_limits=axis_jerk_limits,
            axis_colliders=_parse_axis_colliders(data.get("axis_colliders")),
            axis_reversed=cls._parse_axis_reversed(data.get("axis_reversed")),
            joint_weights=cls._parse_float_list(data.get("joint_weights"), 6, 1.0),
            allowed_configs=allowed_configs,
            home_position=cls._parse_float_list(data.get("home_position"), 6, 0.0),
            position_zero=cls._parse_float_list(data.get("position_zero"), 6, 0.0),
            position_calibration=cls._parse_float_list(data.get("position_calibration"), 6, 0.0),
            robot_cad_models=cls._parse_string_list(
                data.get("robot_cad_models", data.get("robot_cad_files")),
                DEFAULT_ROBOT_CAD_MODELS,
            ),
            robot_cad_colors=CadColorPalette.from_values(
                cls._parse_string_list(data.get("robot_cad_colors"), DEFAULT_ROBOT_CAD_COLORS),
                len(DEFAULT_ROBOT_CAD_COLORS),
                DEFAULT_ROBOT_CAD_COLORS,
            ).to_hex_list(),
            dh_measured=cls._parse_matrix(data.get("dh_measured"), 6, 4, 0.0),
            dh_measured_enabled=bool(data.get("dh_measured_enabled", False)),
            present_fields=present_fields,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": [self.name],
            "dh": [[str(val) for val in row] for row in self.dh[:6]],
            "dh_measured": [[str(val) for val in row] for row in self.dh_measured[:6]],
            "dh_measured_enabled": self.dh_measured_enabled,
            "corr": [[str(val) for val in row] for row in self.corr[:6]],
            "axis_limits": self.axis_limits[:6],
            "cartesian_slider_limits_xyz": self.cartesian_slider_limits_xyz[:3],
            "axis_speed_limits": self.axis_speed_limits[:6],
            "axis_accel_limits": self.axis_accel_limits[:6],
            "axis_jerk_limits": self.axis_jerk_limits[:6],
            "axis_colliders": [_axis_collider_to_dict(collider) for collider in self.axis_colliders[:6]],
            "axis_reversed": self.axis_reversed[:6],
            "joint_weights": self.joint_weights[:6],
            "allowed_configs": [cfg.name for cfg in MgiConfigKey if cfg in self.allowed_configs],
            "home_position": self.home_position[:6],
            "position_zero": self.position_zero[:6],
            "position_calibration": self.position_calibration[:6],
            "robot_cad_models": [str(path) for path in self.robot_cad_models],
            "robot_cad_colors": list(self.robot_cad_colors[:7]),
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
        if "cartesian_slider_limits_xyz" in self.present_fields:
            robot_model.set_cartesian_slider_limits_xyz(self.cartesian_slider_limits_xyz)
        if "axis_speed_limits" in self.present_fields:
            robot_model.set_axis_speed_limits(self.axis_speed_limits)
        if "axis_jerk_limits" in self.present_fields:
            robot_model.set_axis_jerk_limits(self.axis_jerk_limits)
        if "axis_accel_limits" in self.present_fields:
            robot_model.set_axis_accel_limits(self.axis_accel_limits)
        if "axis_colliders" in self.present_fields:
            robot_model.set_axis_colliders(self.axis_colliders)
        if "home_position" in self.present_fields:
            robot_model.set_home_position(self.home_position)
        if "position_zero" in self.present_fields:
            robot_model.set_position_zero(self.position_zero)
        if "position_calibration" in self.present_fields:
            robot_model.set_position_calibration(self.position_calibration)
        if "robot_cad_models" in self.present_fields:
            robot_model.set_robot_cad_models(self.robot_cad_models)
        if "robot_cad_colors" in self.present_fields:
            robot_model.set_robot_cad_colors(self.robot_cad_colors)
        if "dh_measured" in self.present_fields:
            robot_model.set_measured_dh_params(self.dh_measured)
        if "dh_measured_enabled" in self.present_fields:
            robot_model.set_measured_dh_enabled(self.dh_measured_enabled)

    def save(self, file_path: str) -> None:
        with open(file_path, "w") as file:
            json.dump(self.to_dict(), file, indent=4)

    @classmethod
    def load(cls, file_path: str) -> RobotConfigurationFile:
        with open(file_path, "r") as file:
            data = json.load(file)
        return cls.from_dict(data)
