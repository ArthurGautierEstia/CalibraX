from __future__ import annotations

from enum import Enum
import math

from mgi import ConfigurationIdentifier, MgiConfigKey


class KeypointTargetType(Enum):
    CARTESIAN = "CARTESIAN"
    JOINT = "JOINT"


class KeypointMotionMode(Enum):
    PTP = "PTP"
    LINEAR = "LINEAR"
    CUBIC = "CUBIC"


class TrajectoryKeypoint:
    def __init__(
        self,
        target_type: KeypointTargetType = KeypointTargetType.CARTESIAN,
        cartesian_target: list[float] | None = None,
        joint_target: list[float] | None = None,
        mode: KeypointMotionMode = KeypointMotionMode.PTP,
        cubic_vectors: list[list[float]] | None = None,
        allowed_configs: list[MgiConfigKey] | None = None,
        favorite_config: MgiConfigKey = MgiConfigKey.FUN,
        ptp_speed_percent: float = 75.0,
        linear_speed_mps: float = 0.5,
    ) -> None:
        self.target_type = target_type
        self.mode = mode

        self.cartesian_target = self._normalize_float_list(
            [0.0] * 6 if cartesian_target is None else list(cartesian_target),
            6,
            0.0,
        )
        self.joint_target = self._normalize_float_list(
            [0.0] * 6 if joint_target is None else list(joint_target),
            6,
            0.0,
        )

        vectors = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] if cubic_vectors is None else list(cubic_vectors)
        vec1 = vectors[0] if len(vectors) > 0 else []
        vec2 = vectors[1] if len(vectors) > 1 else []
        self.cubic_vectors = [
            self._normalize_float_list(list(vec1), 3, 0.0),
            self._normalize_float_list(list(vec2), 3, 0.0),
        ]

        configs = list(MgiConfigKey) if allowed_configs is None else list(allowed_configs)
        self.allowed_configs = self._unique_configs(configs)
        self.favorite_config = favorite_config

        self.ptp_speed_percent = self._clamp(float(ptp_speed_percent), 0.0, 100.0)
        self.linear_speed_mps = self._clamp(float(linear_speed_mps), 0.0, 2.0)

        self._normalize_configuration_rules()

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _normalize_float_list(values: list[float], size: int, default: float) -> list[float]:
        out: list[float] = []
        for value in values[:size]:
            try:
                out.append(float(value))
            except (TypeError, ValueError):
                out.append(default)
        while len(out) < size:
            out.append(default)
        return out

    @staticmethod
    def _unique_configs(values: list[MgiConfigKey]) -> list[MgiConfigKey]:
        out: list[MgiConfigKey] = []
        for key in values:
            if key in MgiConfigKey and key not in out:
                out.append(key)
        return out

    def _normalize_configuration_rules(self) -> None:
        if self.target_type == KeypointTargetType.JOINT:
            if not self.allowed_configs:
                self.allowed_configs = [self.favorite_config]
            if self.favorite_config not in self.allowed_configs:
                self.favorite_config = self.allowed_configs[0]
            self.allowed_configs = [self.favorite_config]
            return

        if not self.allowed_configs:
            self.allowed_configs = list(MgiConfigKey)

        if self.favorite_config not in self.allowed_configs:
            self.favorite_config = self.allowed_configs[0]

    @property
    def speed(self) -> float:
        return self.get_speed_for_mode(self.mode)

    def get_speed_for_mode(self, mode: KeypointMotionMode | None = None) -> float:
        use_mode = self.mode if mode is None else mode
        if use_mode == KeypointMotionMode.PTP:
            return self.ptp_speed_percent
        return self.linear_speed_mps

    def clone(self) -> TrajectoryKeypoint:
        return TrajectoryKeypoint(
            target_type=self.target_type,
            cartesian_target=list(self.cartesian_target),
            joint_target=list(self.joint_target),
            mode=self.mode,
            cubic_vectors=[list(self.cubic_vectors[0]), list(self.cubic_vectors[1])],
            allowed_configs=list(self.allowed_configs),
            favorite_config=self.favorite_config,
            ptp_speed_percent=self.ptp_speed_percent,
            linear_speed_mps=self.linear_speed_mps,
        )

    @staticmethod
    def identify_config_from_joint_target(
        joint_target_deg: list[float],
        config_identifier: ConfigurationIdentifier,
    ) -> MgiConfigKey:
        joints_rad = [math.radians(v) for v in joint_target_deg[:6]]
        while len(joints_rad) < 6:
            joints_rad.append(0.0)
        return MgiConfigKey.identify_configuration(joints_rad, config_identifier)
