from __future__ import annotations

from trajectory_engine.models.pipeline import TrajectorySample


def _norm3(x: float, y: float, z: float) -> float:
    return float((x * x + y * y + z * z) ** 0.5)


def _shortest_angle_delta_deg(from_deg: float, to_deg: float) -> float:
    delta = (float(to_deg) - float(from_deg) + 180.0) % 360.0 - 180.0
    if delta == -180.0 and (float(to_deg) - float(from_deg)) > 0.0:
        return 180.0
    return delta


def reset_cartesian_dynamics(sample: TrajectorySample) -> None:
    sample.cartesian_velocity = [0.0] * 6
    sample.cartesian_acceleration = [0.0] * 6
    sample.cartesian_jerk = [0.0] * 6
    sample.cartesian_velocity_valid = False
    sample.cartesian_acceleration_valid = False
    sample.cartesian_jerk_valid = False
    sample.velocity = 0.0
    sample.acceleration = 0.0


def reset_articular_dynamics(sample: TrajectorySample) -> None:
    sample.articular_velocity = [0.0] * 6
    sample.articular_acceleration = [0.0] * 6
    sample.articular_jerk = [0.0] * 6
    sample.articular_velocity_valid = False
    sample.articular_acceleration_valid = False
    sample.articular_jerk_valid = False


def update_cartesian_dynamics(sample: TrajectorySample, previous_sample: TrajectorySample | None, dt_s: float) -> None:
    reset_cartesian_dynamics(sample)
    if previous_sample is None or not sample.reachable or not previous_sample.reachable:
        return
    dt = max(1e-9, float(dt_s))
    sample.cartesian_velocity[0] = (sample.pose[0] - previous_sample.pose[0]) / dt
    sample.cartesian_velocity[1] = (sample.pose[1] - previous_sample.pose[1]) / dt
    sample.cartesian_velocity[2] = (sample.pose[2] - previous_sample.pose[2]) / dt
    sample.cartesian_velocity[3] = _shortest_angle_delta_deg(previous_sample.pose[3], sample.pose[3]) / dt
    sample.cartesian_velocity[4] = _shortest_angle_delta_deg(previous_sample.pose[4], sample.pose[4]) / dt
    sample.cartesian_velocity[5] = _shortest_angle_delta_deg(previous_sample.pose[5], sample.pose[5]) / dt
    sample.cartesian_velocity_valid = True
    sample.velocity = _norm3(sample.cartesian_velocity[0], sample.cartesian_velocity[1], sample.cartesian_velocity[2])

    if not previous_sample.cartesian_velocity_valid:
        return
    for axis in range(6):
        sample.cartesian_acceleration[axis] = (
            sample.cartesian_velocity[axis] - previous_sample.cartesian_velocity[axis]
        ) / dt
    sample.cartesian_acceleration_valid = True
    sample.acceleration = _norm3(
        sample.cartesian_acceleration[0],
        sample.cartesian_acceleration[1],
        sample.cartesian_acceleration[2],
    )

    if not previous_sample.cartesian_acceleration_valid:
        return
    for axis in range(6):
        sample.cartesian_jerk[axis] = (
            sample.cartesian_acceleration[axis] - previous_sample.cartesian_acceleration[axis]
        ) / dt
    sample.cartesian_jerk_valid = True


def update_articular_dynamics(sample: TrajectorySample, previous_sample: TrajectorySample | None, dt_s: float) -> None:
    reset_articular_dynamics(sample)
    if previous_sample is None or not sample.reachable or not previous_sample.reachable:
        return
    dt = max(1e-9, float(dt_s))
    for axis in range(6):
        sample.articular_velocity[axis] = (sample.joints[axis] - previous_sample.joints[axis]) / dt
    sample.articular_velocity_valid = True

    if not previous_sample.articular_velocity_valid:
        return
    for axis in range(6):
        sample.articular_acceleration[axis] = (
            sample.articular_velocity[axis] - previous_sample.articular_velocity[axis]
        ) / dt
    sample.articular_acceleration_valid = True

    if not previous_sample.articular_acceleration_valid:
        return
    for axis in range(6):
        sample.articular_jerk[axis] = (
            sample.articular_acceleration[axis] - previous_sample.articular_acceleration[axis]
        ) / dt
    sample.articular_jerk_valid = True
