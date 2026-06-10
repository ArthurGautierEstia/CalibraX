from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from models.types.approach_retract import ApproachAxisRef, ApproachRetractConfig, ApproachRetractStep
from models.types.motion_approximation import ApproximationMode, MotionApproximation


@dataclass
class ProgramGenerationSettings:
    home_enabled: bool = True
    approach: ApproachRetractConfig = field(
        default_factory=ApproachRetractConfig.default_approach
    )
    retract: ApproachRetractConfig = field(
        default_factory=ApproachRetractConfig.default_retract
    )
    header_text: str = ""
    default_approximation: MotionApproximation = field(
        default_factory=MotionApproximation.none
    )

    def to_dict(self) -> dict[str, Any]:
        return {
            "home_enabled": self.home_enabled,
            "approach": _approach_retract_to_dict(self.approach),
            "retract": _approach_retract_to_dict(self.retract),
            "header_text": self.header_text,
            "default_approximation": _approximation_to_dict(self.default_approximation),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> ProgramGenerationSettings:
        payload = data if isinstance(data, dict) else {}
        return cls(
            home_enabled=bool(payload.get("home_enabled", True)),
            approach=_approach_retract_from_dict(
                payload.get("approach"), default=ApproachRetractConfig.default_approach()
            ),
            retract=_approach_retract_from_dict(
                payload.get("retract"), default=ApproachRetractConfig.default_retract()
            ),
            header_text=str(payload.get("header_text", "")),
            default_approximation=_approximation_from_dict(payload.get("default_approximation")),
        )


def _approach_retract_to_dict(cfg: ApproachRetractConfig) -> dict[str, Any]:
    return {
        "enabled": cfg.enabled,
        "steps": [s.to_dict() for s in cfg.steps],
    }


def _approach_retract_from_dict(
    data: Any, default: ApproachRetractConfig
) -> ApproachRetractConfig:
    if not isinstance(data, dict):
        return default
    # Rétrocompatibilité : ancien format avec axis_ref/distance_mm/speed_mps directement
    if "steps" in data and isinstance(data["steps"], list):
        steps = tuple(
            ApproachRetractStep.from_dict(s)
            for s in data["steps"]
            if isinstance(s, dict)
        )
        if not steps:
            steps = default.steps
    elif "axis_ref" in data:
        steps = (ApproachRetractStep.from_dict(data),)
    else:
        steps = default.steps
    return ApproachRetractConfig(
        enabled=bool(data.get("enabled", default.enabled)),
        steps=steps,
    )


def _approximation_to_dict(approx: MotionApproximation) -> dict[str, Any]:
    return {"mode": approx.mode.value, "value": approx.value}


def _approximation_from_dict(data: Any) -> MotionApproximation:
    if not isinstance(data, dict):
        return MotionApproximation.none()
    try:
        mode = ApproximationMode(data.get("mode", "NONE"))
    except ValueError:
        mode = ApproximationMode.NONE
    return MotionApproximation(mode=mode, value=float(data.get("value", 0.0)))
