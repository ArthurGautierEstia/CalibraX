from __future__ import annotations

from trajectory_engine.models import TrajectoryResult, ValidationTaskSample


def build_validation_task_samples(trajectory: TrajectoryResult) -> list[ValidationTaskSample]:
    entries: list[ValidationTaskSample] = []
    global_sample_index = 0
    for segment_index, segment in enumerate(trajectory.segments):
        for sample_index, sample in enumerate(segment.samples):
            entries.append(
                ValidationTaskSample(
                    global_sample_index=global_sample_index,
                    segment_index=segment_index,
                    sample_index=sample_index,
                    sample=sample,
                )
            )
            global_sample_index += 1
    return entries
