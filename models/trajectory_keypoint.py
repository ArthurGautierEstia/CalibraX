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


class ConfigurationPolicy(Enum):
    AUTO = "AUTO"
    CURRENT_BRANCH = "CURRENT_BRANCH"
    FORCED = "FORCED"


class TrajectoryKeypoint:
    DEFAULT_CUBIC_AMPLITUDE_MM = 30.0
    DEFAULT_LINEAR_TANGENT_RATIO = 0.3

    def __init__(
        self,
        target_type: KeypointTargetType = KeypointTargetType.CARTESIAN,
        cartesian_target: list[float] | None = None,
        joint_target: list[float] | None = None,
        mode: KeypointMotionMode = KeypointMotionMode.PTP,
        cubic_vectors: list[list[float]] | None = None,
        cubic_amplitudes_mm: list[float] | None = None,
        linear_tangent_ratios: list[float] | None = None,
        linear_tangent_ratios_linked: bool = True,
        configuration_policy: ConfigurationPolicy = ConfigurationPolicy.AUTO,
        forced_config: MgiConfigKey | None = None,
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
            [self.DEFAULT_CUBIC_AMPLITUDE_MM, self.DEFAULT_CUBIC_AMPLITUDE_MM]
            if cubic_amplitudes_mm is None
            else list(cubic_amplitudes_mm)
        )
        amp1 = amplitudes[0] if len(amplitudes) > 0 else self.DEFAULT_CUBIC_AMPLITUDE_MM
        amp2 = amplitudes[1] if len(amplitudes) > 1 else self.DEFAULT_CUBIC_AMPLITUDE_MM
        amp_values = self._normalize_float_list([amp1, amp2], 2, self.DEFAULT_CUBIC_AMPLITUDE_MM)
        self.cubic_amplitudes_mm = [
            self._clamp_min(amp_values[0], 0.0),
            self._clamp_min(amp_values[1], 0.0),
        ]
        linear_ratios = (
            [self.DEFAULT_LINEAR_TANGENT_RATIO, self.DEFAULT_LINEAR_TANGENT_RATIO]
            if linear_tangent_ratios is None
            else list(linear_tangent_ratios)
        )
        ratio1 = linear_ratios[0] if len(linear_ratios) > 0 else self.DEFAULT_LINEAR_TANGENT_RATIO
        ratio2 = linear_ratios[1] if len(linear_ratios) > 1 else self.DEFAULT_LINEAR_TANGENT_RATIO
        ratio_values = self._normalize_float_list([ratio1, ratio2], 2, self.DEFAULT_LINEAR_TANGENT_RATIO)
        self.linear_tangent_ratios = [
            self._clamp_min(ratio_values[0], 0.0),
            self._clamp_min(ratio_values[1], 0.0),
        ]
        self.linear_tangent_ratios_linked = bool(linear_tangent_ratios_linked)

        self.configuration_policy = configuration_policy
        self.forced_config = forced_config if forced_config in MgiConfigKey else None

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
        _ = segment_length_mm
        tangents: list[list[float]] = []
        for idx in range(2):
            direction = self.cubic_vectors[idx]
            amplitude_mm = self._clamp_min(float(self.cubic_amplitudes_mm[idx]), 0.0)
            tangents.append(
                [
                    float(direction[0]) * amplitude_mm,
                    float(direction[1]) * amplitude_mm,
                    float(direction[2]) * amplitude_mm,
                ]
            )
        return tangents[0], tangents[1]

    def resolve_linear_tangent_vectors(
        self,
        start_xyz: list[float],
        end_xyz: list[float],
    ) -> tuple[list[float], list[float]]:
        if len(start_xyz) < 3 or len(end_xyz) < 3:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        dx = float(end_xyz[0]) - float(start_xyz[0])
        dy = float(end_xyz[1]) - float(start_xyz[1])
        dz = float(end_xyz[2]) - float(start_xyz[2])
        start_ratio = self._clamp_min(float(self.linear_tangent_ratios[0]), 0.0)
        end_ratio = self._clamp_min(float(self.linear_tangent_ratios[1]), 0.0)
        return (
            [dx * start_ratio, dy * start_ratio, dz * start_ratio],
            [-dx * end_ratio, -dy * end_ratio, -dz * end_ratio],
        )

    def _normalize_configuration_rules(self) -> None:
        if self.configuration_policy not in ConfigurationPolicy:
            self.configuration_policy = ConfigurationPolicy.AUTO
        if self.configuration_policy == ConfigurationPolicy.FORCED and self.forced_config is None:
            self.forced_config = MgiConfigKey.FUN
        if self.configuration_policy != ConfigurationPolicy.FORCED:
            self.forced_config = None

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
            cubic_amplitudes_mm=list(self.cubic_amplitudes_mm),
            linear_tangent_ratios=list(self.linear_tangent_ratios),
            linear_tangent_ratios_linked=self.linear_tangent_ratios_linked,
            configuration_policy=self.configuration_policy,
            forced_config=self.forced_config,
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
            "cubic_amplitudes_mm": [
                float(self.cubic_amplitudes_mm[0]),
                float(self.cubic_amplitudes_mm[1]),
            ],
            "linear_tangent_ratios": [
                float(self.linear_tangent_ratios[0]),
                float(self.linear_tangent_ratios[1]),
            ],
            "linear_tangent_ratios_linked": bool(self.linear_tangent_ratios_linked),
            "configuration_policy": self.configuration_policy.value,
            "forced_config": self.forced_config.name if self.forced_config is not None else None,
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

        try:
            configuration_policy = ConfigurationPolicy(
                str(raw.get("configuration_policy", ConfigurationPolicy.AUTO.value))
            )
        except ValueError:
            configuration_policy = ConfigurationPolicy.AUTO

        forced_config: MgiConfigKey | None = None
        forced_raw = raw.get("forced_config")
        if isinstance(forced_raw, str):
            try:
                forced_config = MgiConfigKey[forced_raw]
            except KeyError:
                forced_config = None

        cubic_vectors = raw.get("cubic_vectors")
        if not isinstance(cubic_vectors, list):
            cubic_vectors = None
        cubic_amplitudes_mm = raw.get("cubic_amplitudes_mm")
        if not isinstance(cubic_amplitudes_mm, list) or len(cubic_amplitudes_mm) < 2:
            raise ValueError(
                "Format invalide: 'cubic_amplitudes_mm' doit etre une liste "
                "de 2 valeurs."
            )
        linear_tangent_ratios = raw.get("linear_tangent_ratios")
        if not isinstance(linear_tangent_ratios, list) or len(linear_tangent_ratios) < 2:
            raise ValueError(
                "Format invalide: 'linear_tangent_ratios' doit etre une liste "
                "de 2 valeurs."
            )
        linear_tangent_ratios_linked = raw.get("linear_tangent_ratios_linked")
        if not isinstance(linear_tangent_ratios_linked, bool):
            raise ValueError(
                "Format invalide: 'linear_tangent_ratios_linked' doit etre un booleen."
            )

        return TrajectoryKeypoint(
            target_type=target_type,
            cartesian_target=raw.get("cartesian_target"),
            joint_target=raw.get("joint_target"),
            mode=mode,
            cubic_vectors=cubic_vectors,
            cubic_amplitudes_mm=cubic_amplitudes_mm,
            linear_tangent_ratios=linear_tangent_ratios,
            linear_tangent_ratios_linked=linear_tangent_ratios_linked,
            configuration_policy=configuration_policy,
            forced_config=forced_config,
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
