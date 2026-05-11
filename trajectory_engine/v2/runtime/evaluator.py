from __future__ import annotations

from models.types import Pose6, XYZ3
from trajectory_engine.v2.arc_length import parameter_at_distance
from trajectory_engine.v2.dynamics import ScalarMotionProfile, normalized_s_curve
from trajectory_engine.v2.models import RuntimeSegment


def _wrap_angle_deg(angle_deg: float) -> float:
    wrapped = (float(angle_deg) + 180.0) % 360.0 - 180.0
    if wrapped == -180.0 and angle_deg > 0.0:
        return 180.0
    return wrapped


def _shortest_angle_delta_deg(from_deg: float, to_deg: float) -> float:
    delta = (float(to_deg) - float(from_deg) + 180.0) % 360.0 - 180.0
    if delta == -180.0 and (float(to_deg) - float(from_deg)) > 0.0:
        return 180.0
    return delta


class RuntimeEvaluator:
    def __init__(self, runtime_segment: RuntimeSegment, profile: ScalarMotionProfile) -> None:
        self.runtime_segment = runtime_segment
        self.profile = profile

    def evaluate_pose(self, time_s: float) -> Pose6:
        state = self.profile.evaluate(time_s)
        segment = self.runtime_segment
        local_distance_mm = max(0.0, min(segment.speed_profile.length_mm, state.position))
        if segment.curve is not None and segment.arc_lut is not None:
            u = parameter_at_distance(segment.arc_lut, local_distance_mm)
            point: XYZ3 = segment.curve.point(u)
        else:
            length = max(1e-9, segment.speed_profile.length_mm)
            u = max(0.0, min(1.0, local_distance_mm / length))
            point = XYZ3(
                segment.start_pose.x + (segment.end_pose.x - segment.start_pose.x) * u,
                segment.start_pose.y + (segment.end_pose.y - segment.start_pose.y) * u,
                segment.start_pose.z + (segment.end_pose.z - segment.start_pose.z) * u,
            )

        orientation_u = normalized_s_curve(
            0.0 if segment.speed_profile.length_mm <= 1e-9 else local_distance_mm / segment.speed_profile.length_mm
        )
        d_a = _shortest_angle_delta_deg(segment.start_pose.a, segment.end_pose.a)
        d_b = _shortest_angle_delta_deg(segment.start_pose.b, segment.end_pose.b)
        d_c = _shortest_angle_delta_deg(segment.start_pose.c, segment.end_pose.c)
        return Pose6(
            point.x,
            point.y,
            point.z,
            _wrap_angle_deg(segment.start_pose.a + d_a * orientation_u),
            _wrap_angle_deg(segment.start_pose.b + d_b * orientation_u),
            _wrap_angle_deg(segment.start_pose.c + d_c * orientation_u),
        )
