from __future__ import annotations

from enum import Enum
import math

from utils.mgi import ConfigurationIdentifier, MgiConfigKey


class KeypointTargetType(Enum):
    CARTESIAN = "CARTESIAN"
    JOINT = "JOINT"


class KeypointMotionMode(Enum):
    PTP = "PTP"
    LINEAR = "LINEAR"
    CUBIC = "CUBIC"


class TrajectoryKeypoint:
    DEFAULT_CUBIC_AMPLITUDE_PERCENT = 30.0

    def __init__(
        self,
        target_type: KeypointTargetType = KeypointTargetType.CARTESIAN,
        cartesian_target: list[float] | None = None,
        joint_target: list[float] | None = None,
        mode: KeypointMotionMode = KeypointMotionMode.PTP,
        cubic_vectors: list[list[float]] | None = None,
        cubic_amplitudes_percent: list[float] | None = None,
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

        # Segment-in semantics (for the segment that ends at this keypoint):
        # cubic_vectors[0] = direction at segment start (previous point side)
        # cubic_vectors[1] = direction at segment end (current point side)
        vectors = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]] if cubic_vectors is None else list(cubic_vectors)
        vec1 = vectors[0] if len(vectors) > 0 else []
        vec2 = vectors[1] if len(vectors) > 1 else []
        self.cubic_vectors = [
            self._normalize_direction_vector(self._normalize_float_list(list(vec1), 3, 0.0)),
            self._normalize_direction_vector(self._normalize_float_list(list(vec2), 3, 0.0)),
        ]
        amplitudes = (
            [self.DEFAULT_CUBIC_AMPLITUDE_PERCENT, self.DEFAULT_CUBIC_AMPLITUDE_PERCENT]
            if cubic_amplitudes_percent is None
            else list(cubic_amplitudes_percent)
        )
        amp1 = amplitudes[0] if len(amplitudes) > 0 else self.DEFAULT_CUBIC_AMPLITUDE_PERCENT
        amp2 = amplitudes[1] if len(amplitudes) > 1 else self.DEFAULT_CUBIC_AMPLITUDE_PERCENT
        amp_values = self._normalize_float_list([amp1, amp2], 2, self.DEFAULT_CUBIC_AMPLITUDE_PERCENT)
        self.cubic_amplitudes_percent = [
            self._clamp_min(amp_values[0], 0.0),
            self._clamp_min(amp_values[1], 0.0),
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
    def _clamp_min(value: float, minimum: float) -> float:
        return max(minimum, value)

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

    @staticmethod
    def _normalize_direction_vector(vector_xyz: list[float]) -> list[float]:
        x = float(vector_xyz[0]) if len(vector_xyz) > 0 else 0.0
        y = float(vector_xyz[1]) if len(vector_xyz) > 1 else 0.0
        z = float(vector_xyz[2]) if len(vector_xyz) > 2 else 0.0
        norm = math.sqrt(x * x + y * y + z * z)
        if norm <= 1e-12:
            return [0.0, 0.0, 0.0]
        return [x / norm, y / norm, z / norm]

    def resolve_cubic_tangent_vectors(self, segment_length_mm: float) -> tuple[list[float], list[float]]:
        distance = max(0.0, float(segment_length_mm))
        tangents: list[list[float]] = []
        for idx in range(2):
            direction = self.cubic_vectors[idx]
            amplitude_percent = self._clamp_min(float(self.cubic_amplitudes_percent[idx]), 0.0)
            amplitude_mm = distance * (amplitude_percent / 100.0)
            tangents.append(
                [
                    float(direction[0]) * amplitude_mm,
                    float(direction[1]) * amplitude_mm,
                    float(direction[2]) * amplitude_mm,
                ]
            )
        return tangents[0], tangents[1]

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
            cubic_amplitudes_percent=list(self.cubic_amplitudes_percent),
            allowed_configs=list(self.allowed_configs),
            favorite_config=self.favorite_config,
            ptp_speed_percent=self.ptp_speed_percent,
            linear_speed_mps=self.linear_speed_mps,
        )

    def to_dict(self) -> dict:
        return {
            "target_type": self.target_type.value,
            "cartesian_target": [float(v) for v in self.cartesian_target[:6]],
            "joint_target": [float(v) for v in self.joint_target[:6]],
            "mode": self.mode.value,
            "cubic_vectors": [
                [float(v) for v in self.cubic_vectors[0][:3]],
                [float(v) for v in self.cubic_vectors[1][:3]],
            ],
            "cubic_amplitudes_percent": [
                float(self.cubic_amplitudes_percent[0]),
                float(self.cubic_amplitudes_percent[1]),
            ],
            "allowed_configs": [cfg.name for cfg in self.allowed_configs],
            "favorite_config": self.favorite_config.name,
            "ptp_speed_percent": float(self.ptp_speed_percent),
            "linear_speed_mps": float(self.linear_speed_mps),
        }

    @staticmethod
    def from_dict(data: dict) -> TrajectoryKeypoint:
        raw = data if isinstance(data, dict) else {}

        try:
            target_type = KeypointTargetType(str(raw.get("target_type", KeypointTargetType.CARTESIAN.value)))
        except ValueError:
            target_type = KeypointTargetType.CARTESIAN

        try:
            mode = KeypointMotionMode(str(raw.get("mode", KeypointMotionMode.PTP.value)))
        except ValueError:
            mode = KeypointMotionMode.PTP

        allowed_configs: list[MgiConfigKey] = []
        for name in raw.get("allowed_configs", []):
            try:
                allowed_configs.append(MgiConfigKey[str(name)])
            except (KeyError, TypeError):
                continue

        favorite_config = MgiConfigKey.FUN
        try:
            favorite_config = MgiConfigKey[str(raw.get("favorite_config", MgiConfigKey.FUN.name))]
        except (KeyError, TypeError):
            if allowed_configs:
                favorite_config = allowed_configs[0]

        cubic_vectors = raw.get("cubic_vectors")
        if not isinstance(cubic_vectors, list):
            cubic_vectors = None
        cubic_amplitudes_percent = raw.get("cubic_amplitudes_percent")
        if not isinstance(cubic_amplitudes_percent, list) or len(cubic_amplitudes_percent) < 2:
            raise ValueError(
                "Format invalide: 'cubic_amplitudes_percent' doit etre une liste "
                "de 2 valeurs (nouveau format requis)."
            )

        return TrajectoryKeypoint(
            target_type=target_type,
            cartesian_target=raw.get("cartesian_target"),
            joint_target=raw.get("joint_target"),
            mode=mode,
            cubic_vectors=cubic_vectors,
            cubic_amplitudes_percent=cubic_amplitudes_percent,
            allowed_configs=allowed_configs if allowed_configs else None,
            favorite_config=favorite_config,
            ptp_speed_percent=float(raw.get("ptp_speed_percent", 75.0)),
            linear_speed_mps=float(raw.get("linear_speed_mps", 0.5)),
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
