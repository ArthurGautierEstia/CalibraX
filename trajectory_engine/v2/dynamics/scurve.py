from __future__ import annotations

from dataclasses import dataclass

from trajectory_engine.v2.models import (
    MotionScalarState,
    SegmentDynamicPhaseKind,
    SegmentDynamicProfileKind,
    SegmentDynamicResolution,
)


S_CURVE_PEAK_SPEED_SCALE = 2.1875
S_CURVE_ACCEL_PEAK_SCALE = 2.1875
S_CURVE_JERK_FROM_VELOCITY_SCALE = 7.5131884044
_EPS = 1e-9


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def normalized_s_curve(u: float) -> float:
    u = _clamp01(u)
    u2 = u * u
    u3 = u2 * u
    u4 = u3 * u
    return 35.0 * u4 - 84.0 * u4 * u + 70.0 * u3 * u3 - 20.0 * u4 * u3


def normalized_s_curve_derivative(u: float) -> float:
    u = _clamp01(u)
    u2 = u * u
    u3 = u2 * u
    return 140.0 * u3 - 420.0 * u3 * u + 420.0 * u3 * u2 - 140.0 * u3 * u3


def normalized_s_curve_second_derivative(u: float) -> float:
    u = _clamp01(u)
    u2 = u * u
    u3 = u2 * u
    u4 = u2 * u2
    return 420.0 * u2 - 1680.0 * u3 + 2100.0 * u4 - 840.0 * u4 * u


def normalized_s_curve_third_derivative(u: float) -> float:
    u = _clamp01(u)
    u2 = u * u
    u3 = u2 * u
    u4 = u2 * u2
    return 840.0 * u - 5040.0 * u2 + 8400.0 * u3 - 4200.0 * u4


def normalized_s_curve_integral(u: float) -> float:
    u = _clamp01(u)
    u2 = u * u
    u3 = u2 * u
    u4 = u2 * u2
    u5 = u4 * u
    u6 = u3 * u3
    u7 = u6 * u
    u8 = u4 * u4
    return 7.0 * u5 - 14.0 * u6 + 10.0 * u7 - 2.5 * u8


def ptp_duration_s(delta_abs: float, velocity_limit: float) -> float:
    if abs(delta_abs) <= _EPS:
        return 0.0
    if velocity_limit <= _EPS:
        return 0.0
    return abs(float(delta_abs)) * S_CURVE_PEAK_SPEED_SCALE / float(velocity_limit)


@dataclass(frozen=True)
class ScalarMotionPhase:
    kind: SegmentDynamicPhaseKind
    segment_index: int
    start_time_s: float
    duration_s: float
    start_position: float
    start_speed: float
    end_speed: float

    def distance(self) -> float:
        if self.duration_s <= _EPS:
            return 0.0
        if self.kind == SegmentDynamicPhaseKind.CRUISE:
            return self.start_speed * self.duration_s
        return 0.5 * (self.start_speed + self.end_speed) * self.duration_s

    def end_time_s(self) -> float:
        return self.start_time_s + self.duration_s

    def end_position(self) -> float:
        return self.start_position + self.distance()

    def evaluate(self, time_s: float) -> MotionScalarState:
        if self.duration_s <= _EPS:
            return MotionScalarState(
                time_s=float(time_s),
                position=self.start_position,
                velocity=self.end_speed,
                acceleration=0.0,
                jerk=0.0,
                phase=self.kind,
                segment_index=self.segment_index,
            )

        local_t = max(0.0, min(self.duration_s, float(time_s) - self.start_time_s))
        if self.kind == SegmentDynamicPhaseKind.CRUISE:
            return MotionScalarState(
                time_s=float(time_s),
                position=self.start_position + self.start_speed * local_t,
                velocity=self.start_speed,
                acceleration=0.0,
                jerk=0.0,
                phase=self.kind,
                segment_index=self.segment_index,
            )

        u = local_t / self.duration_s
        dv = self.end_speed - self.start_speed
        position = self.start_position + self.start_speed * local_t + dv * self.duration_s * normalized_s_curve_integral(u)
        velocity = self.start_speed + dv * normalized_s_curve(u)
        acceleration = dv * normalized_s_curve_derivative(u) / self.duration_s
        jerk = dv * normalized_s_curve_second_derivative(u) / (self.duration_s * self.duration_s)
        return MotionScalarState(
            time_s=float(time_s),
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            jerk=jerk,
            phase=self.kind,
            segment_index=self.segment_index,
        )


class ScalarMotionProfile:
    def __init__(self, phases: list[ScalarMotionPhase]) -> None:
        self.phases = list(phases)
        self.duration_s = self.phases[-1].end_time_s() if self.phases else 0.0
        self.distance = self.phases[-1].end_position() if self.phases else 0.0

    def evaluate(self, time_s: float) -> MotionScalarState:
        if not self.phases:
            return MotionScalarState(0.0, 0.0, 0.0, 0.0, 0.0, SegmentDynamicPhaseKind.CRUISE, 0)
        if time_s <= self.phases[0].start_time_s:
            return self.phases[0].evaluate(time_s)
        for phase in self.phases:
            if time_s <= phase.end_time_s() + 1e-12:
                return phase.evaluate(time_s)
        return self.phases[-1].evaluate(self.phases[-1].end_time_s())


def _transition_duration_s(delta_speed: float, accel_limit: float, jerk_limit: float) -> float:
    delta = abs(float(delta_speed))
    if delta <= _EPS:
        return 0.0
    accel_duration = S_CURVE_ACCEL_PEAK_SCALE * delta / max(float(accel_limit), _EPS)
    jerk_duration = (S_CURVE_JERK_FROM_VELOCITY_SCALE * delta / max(float(jerk_limit), _EPS)) ** 0.5
    return max(accel_duration, jerk_duration)


def _transition_distance(start_speed: float, end_speed: float, accel_limit: float, jerk_limit: float) -> float:
    duration = _transition_duration_s(end_speed - start_speed, accel_limit, jerk_limit)
    return 0.5 * (start_speed + end_speed) * duration


def _required_distance(entry_speed: float, cruise_speed: float, exit_speed: float, accel_limit: float, jerk_limit: float) -> float:
    return (
        _transition_distance(entry_speed, cruise_speed, accel_limit, jerk_limit)
        + _transition_distance(cruise_speed, exit_speed, accel_limit, jerk_limit)
    )


def _fit_peak_speed(
    length_mm: float,
    target_speed: float,
    entry_speed: float,
    exit_speed: float,
    accel_limit: float,
    jerk_limit: float,
) -> float:
    low = max(entry_speed, exit_speed, 0.0)
    high = max(low, target_speed)
    if _required_distance(entry_speed, high, exit_speed, accel_limit, jerk_limit) <= length_mm:
        return high
    for _ in range(64):
        mid = 0.5 * (low + high)
        if _required_distance(entry_speed, mid, exit_speed, accel_limit, jerk_limit) <= length_mm:
            low = mid
        else:
            high = mid
    return low


def resolve_segment_dynamic_profile(
    length_mm: float,
    target_speed_mm_s: float,
    entry_speed_mm_s: float,
    exit_speed_mm_s: float,
    accel_limit_mm_s2: float,
    jerk_limit_mm_s3: float,
) -> SegmentDynamicResolution:
    length = max(0.0, float(length_mm))
    target_speed = max(0.0, float(target_speed_mm_s))
    entry_speed = max(0.0, min(float(entry_speed_mm_s), target_speed))
    exit_speed = max(0.0, min(float(exit_speed_mm_s), target_speed))
    accel_limit = max(float(accel_limit_mm_s2), _EPS)
    jerk_limit = max(float(jerk_limit_mm_s3), _EPS)

    if length <= _EPS or target_speed <= _EPS:
        return SegmentDynamicResolution(
            profile_kind=SegmentDynamicProfileKind.TRIANGULAR,
            peak_speed_mm_s=0.0,
            target_speed_reached=target_speed <= _EPS,
            accel_distance_mm=0.0,
            cruise_distance_mm=0.0,
            decel_distance_mm=0.0,
        )

    peak_speed = _fit_peak_speed(length, target_speed, entry_speed, exit_speed, accel_limit, jerk_limit)
    accel_distance = _transition_distance(entry_speed, peak_speed, accel_limit, jerk_limit)
    decel_distance = _transition_distance(peak_speed, exit_speed, accel_limit, jerk_limit)
    cruise_distance = max(0.0, length - accel_distance - decel_distance)
    target_speed_reached = peak_speed >= target_speed - _EPS
    profile_kind = (
        SegmentDynamicProfileKind.TRAPEZOIDAL
        if target_speed_reached and cruise_distance > _EPS
        else SegmentDynamicProfileKind.TRIANGULAR
    )

    return SegmentDynamicResolution(
        profile_kind=profile_kind,
        peak_speed_mm_s=peak_speed,
        target_speed_reached=target_speed_reached,
        accel_distance_mm=accel_distance,
        cruise_distance_mm=cruise_distance if profile_kind == SegmentDynamicProfileKind.TRAPEZOIDAL else 0.0,
        decel_distance_mm=decel_distance,
    )


def build_distance_profile(
    segment_index: int,
    length_mm: float,
    target_speed_mm_s: float,
    entry_speed_mm_s: float,
    exit_speed_mm_s: float,
    accel_limit_mm_s2: float,
    jerk_limit_mm_s3: float,
    start_time_s: float = 0.0,
    start_position_mm: float = 0.0,
) -> ScalarMotionProfile:
    length = max(0.0, float(length_mm))
    target_speed = max(0.0, float(target_speed_mm_s))
    entry_speed = max(0.0, min(float(entry_speed_mm_s), target_speed))
    exit_speed = max(0.0, min(float(exit_speed_mm_s), target_speed))
    accel_limit = max(float(accel_limit_mm_s2), _EPS)
    jerk_limit = max(float(jerk_limit_mm_s3), _EPS)

    if length <= _EPS or target_speed <= _EPS:
        phase = ScalarMotionPhase(
            kind=SegmentDynamicPhaseKind.TRANSITION,
            segment_index=segment_index,
            start_time_s=start_time_s,
            duration_s=0.0,
            start_position=start_position_mm,
            start_speed=0.0,
            end_speed=0.0,
        )
        return ScalarMotionProfile([phase])

    resolution = resolve_segment_dynamic_profile(
        length_mm=length,
        target_speed_mm_s=target_speed,
        entry_speed_mm_s=entry_speed,
        exit_speed_mm_s=exit_speed,
        accel_limit_mm_s2=accel_limit,
        jerk_limit_mm_s3=jerk_limit,
    )
    peak_speed = resolution.peak_speed_mm_s
    accel_duration = _transition_duration_s(peak_speed - entry_speed, accel_limit, jerk_limit)
    decel_duration = _transition_duration_s(exit_speed - peak_speed, accel_limit, jerk_limit)
    cruise_distance = resolution.cruise_distance_mm
    cruise_duration = cruise_distance / peak_speed if peak_speed > _EPS else 0.0

    phases: list[ScalarMotionPhase] = []
    cursor_time = float(start_time_s)
    cursor_position = float(start_position_mm)
    if accel_duration > _EPS:
        phase = ScalarMotionPhase(
            kind=SegmentDynamicPhaseKind.ACCEL if peak_speed >= entry_speed else SegmentDynamicPhaseKind.DECEL,
            segment_index=segment_index,
            start_time_s=cursor_time,
            duration_s=accel_duration,
            start_position=cursor_position,
            start_speed=entry_speed,
            end_speed=peak_speed,
        )
        phases.append(phase)
        cursor_time = phase.end_time_s()
        cursor_position = phase.end_position()

    if resolution.profile_kind == SegmentDynamicProfileKind.TRAPEZOIDAL and cruise_duration > _EPS:
        phase = ScalarMotionPhase(
            kind=SegmentDynamicPhaseKind.CRUISE,
            segment_index=segment_index,
            start_time_s=cursor_time,
            duration_s=cruise_duration,
            start_position=cursor_position,
            start_speed=peak_speed,
            end_speed=peak_speed,
        )
        phases.append(phase)
        cursor_time = phase.end_time_s()
        cursor_position = phase.end_position()

    if decel_duration > _EPS:
        phase = ScalarMotionPhase(
            kind=SegmentDynamicPhaseKind.DECEL if exit_speed <= peak_speed else SegmentDynamicPhaseKind.ACCEL,
            segment_index=segment_index,
            start_time_s=cursor_time,
            duration_s=decel_duration,
            start_position=cursor_position,
            start_speed=peak_speed,
            end_speed=exit_speed,
        )
        phases.append(phase)

    if not phases:
        phases.append(
            ScalarMotionPhase(
                kind=SegmentDynamicPhaseKind.TRANSITION,
                segment_index=segment_index,
                start_time_s=start_time_s,
                duration_s=0.0,
                start_position=start_position_mm,
                start_speed=0.0,
                end_speed=0.0,
            )
        )

    return ScalarMotionProfile(phases)
