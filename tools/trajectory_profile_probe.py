from __future__ import annotations

import csv
from pathlib import Path

from trajectory_engine.v2.dynamics import build_distance_profile
from trajectory_engine.v2.models import TrajectoryPassMode


def _read_float(prompt: str, default: float) -> float:
    raw = input(f"{prompt} [{default}]: ").strip()
    if not raw:
        return float(default)
    return float(raw)


def _read_pass_mode(prompt: str) -> TrajectoryPassMode:
    raw = input(f"{prompt} STOP/FLY_BY [STOP]: ").strip().upper()
    if raw == "FLY_BY":
        return TrajectoryPassMode.FLY_BY
    return TrajectoryPassMode.STOP


def main() -> None:
    segment_count = int(_read_float("Nombre de segments", 1.0))
    accel_limit = _read_float("Acceleration max cartesienne mm/s2", 1000.0)
    jerk_limit = _read_float("Jerk max cartesien mm/s3", 10000.0)
    sample_dt_s = _read_float("Pas d'echantillonnage s", 0.004)

    lengths: list[float] = []
    speeds: list[float] = []
    pass_modes: list[TrajectoryPassMode] = []
    for index in range(segment_count):
        print(f"\nSegment {index + 1}")
        lengths.append(_read_float("  Longueur mm", 500.0))
        speeds.append(_read_float("  Vitesse cible mm/s", 500.0))
        pass_modes.append(_read_pass_mode("  Passage au point final"))

    output_dir = Path("user_data") / "trajectory_profile_exports"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "profile_probe.csv"

    rows: list[list[object]] = []
    start_time = 0.0
    for index in range(segment_count):
        next_speed = speeds[index + 1] if index + 1 < segment_count else 0.0
        entry_speed = rows[-1][4] if rows else 0.0
        exit_speed = min(speeds[index], next_speed) if pass_modes[index] == TrajectoryPassMode.FLY_BY else 0.0
        profile = build_distance_profile(
            segment_index=index,
            length_mm=lengths[index],
            target_speed_mm_s=speeds[index],
            entry_speed_mm_s=float(entry_speed),
            exit_speed_mm_s=exit_speed,
            accel_limit_mm_s2=accel_limit,
            jerk_limit_mm_s3=jerk_limit,
            start_time_s=start_time,
            start_position_mm=0.0,
        )
        print(f"\nSegment {index + 1}: duree={profile.duration_s - start_time:.6f}s")
        for phase in profile.phases:
            print(
                f"  {phase.kind.value}: t={phase.start_time_s:.6f}->{phase.end_time_s():.6f}, "
                f"v={phase.start_speed:.3f}->{phase.end_speed:.3f}, "
                f"s={phase.start_position:.3f}->{phase.end_position():.3f}"
            )
        step_count = max(1, int((profile.duration_s - start_time) / sample_dt_s) + 1)
        for step in range(step_count + 1):
            t = min(profile.duration_s, start_time + step * sample_dt_s)
            state = profile.evaluate(t)
            rows.append(
                [
                    f"{state.time_s:.6f}",
                    state.segment_index,
                    state.phase.value,
                    f"{state.position:.6f}",
                    f"{state.velocity:.6f}",
                    f"{state.acceleration:.6f}",
                    f"{state.jerk:.6f}",
                ]
            )
            if t >= profile.duration_s:
                break
        start_time = profile.duration_s

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["time_s", "segment_index", "phase", "s_mm", "v_mm_s", "a_mm_s2", "j_mm_s3"])
        writer.writerows(rows)

    print(f"\nCSV ecrit: {output_path}")


if __name__ == "__main__":
    main()
