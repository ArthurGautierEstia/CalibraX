# Trajectory Builder And C++ Migration Notes

## Purpose

This document captures:

- the current Python architecture for trajectory preview, full trajectory generation, and validity analysis
- the reasons behind the current and planned design choices
- the limits imposed by Python and the current implementation
- the target architecture we want to converge to before a future C++ migration
- the migration guidance for rebuilding the same system in C++

This file is intended to be a living reference. It should be readable by a future human developer, Codex, Claude, or any other assistant that needs to understand both:

- how the current Python system works
- how it should evolve
- how it should later be ported to C++


## Recommended Usage

This document should exist both:

- before the refactor, to freeze the current architecture and the refactor intent
- after the refactor, updated to describe what was actually implemented

Conclusion:

- write the first version now
- update it during and after the refactor

If we wait until after coding, we risk losing the exact reasons behind the design decisions. If we write it only now and never update it, it will become stale. The correct approach is to maintain it as an architecture log.


## Current Python Architecture

### Main entry point

The main orchestration currently happens in:

- `controllers/trajectory_controller.py`

When keypoints change, the controller currently does two separate things:

1. It computes a fast preview trajectory for visualization.
2. It launches a worker that recomputes the full trajectory and then runs validity analysis.


### Current preview pipeline

The preview pipeline uses:

- `utils/trajectory_preview_builder.py`
- `models/trajectory_preview.py`

Goal:

- provide a trajectory that can be drawn quickly in the UI
- avoid the full cost of complete analysis during immediate editing

Preview properties:

- it stores lightweight preview segments and preview samples
- it is suitable for drawing graphs and 3D path previews
- it does not always contain full joint-space information
- for LIN / CUBIC segments, preview samples often only carry cartesian pose data

Important consequence:

- preview results are not currently reused as the base for full analysis
- the full worker recomputes everything from keypoints again


### Current full trajectory pipeline

The full computation currently uses:

- `utils/trajectory_full_analysis_worker.py`
- `utils/trajectory_builder.py`
- `models/trajectory_result.py`

Current behavior:

1. The worker rebuilds a full `TrajectoryResult` from keypoints.
2. It does not reuse the previously computed preview samples.
3. It then runs the validity analyzer.
4. It applies validity results back onto the trajectory.

The full builder is responsible for producing rich samples containing:

- time
- joints
- pose
- kinematics cache
- selected configuration
- dynamic data
- generation errors


### Current validity pipeline

The validity pipeline currently uses:

- `utils/trajectory_validity_analyzer.py`
- `utils/collision_utils.py`

Current behavior:

- the analyzer iterates over all trajectory samples
- for each sample it checks:
  - collision with workspace collision zones
  - collision between robot colliders and tool colliders
  - TCP workspace inclusion
- validity results are then applied back onto the trajectory result

Important detail:

- the analyzer can reuse `sample.kinematics.corrected_matrices` when present
- this is already a useful cache boundary between generation and validity


## Current Domain Model

### Keypoints

Trajectory definition starts from:

- `models/trajectory_keypoint.py`

Keypoints encode:

- target type
- motion mode
- speed settings
- configuration policy
- cubic tangent data
- linear tangent ratios

Important detail for CUBIC:

- tangent semantics are segment-in semantics
- the tangents stored on a keypoint describe the segment that ends at that keypoint

This impacts which segments must be recalculated after editing a keypoint.


### Segment and sample results

The full result model is in:

- `models/trajectory_result.py`

Current important objects:

- `TrajectoryResult`
- `SegmentResult`
- `TrajectorySample`
- `TrajectorySegment`

This model already stores enough information to support future reuse of already built segments, but the current pipeline does not exploit that yet.


## Important Current Behaviors And Constraints

### The system is currently full rebuild oriented

Even though `TrajectoryResult` stores computed segments and samples, the current controller behavior is still:

- regenerate preview from scratch
- launch a worker that rebuilds the full trajectory from scratch
- rerun validity analysis across the full trajectory

So the current architecture is still logically a full rebuild pipeline.


### The preview and full pipelines are duplicated

Today there is a split between:

- `TrajectoryPreviewBuilder`
- `TrajectoryBuilder`

This is useful for UI responsiveness, but it also means:

- duplicated orchestration logic
- duplicated segment traversal logic
- limited reuse between preview and full generation


### Some dependencies are inherently sequential

Not everything can be parallelized safely at sample granularity.

Important sequential dependencies include:

- full sample time progression
- IK solution selection for cartesian samples based on the previous sample
- configuration continuity
- articular velocity / acceleration / jerk
- cartesian velocity / acceleration / jerk
- configuration jump detection

This means the complete sample generation phase should still be considered mostly sequential.


### LIN and CUBIC may be grouped into chained super-segments

In the current builder, LIN and CUBIC segments can be grouped into larger chained units when continuity conditions are met.

Implications:

- local editing is not always limited to one or two segments
- touching one keypoint can affect a whole chain
- future incremental rebuild must operate on a dirty range, not on a single segment replacement rule


### Current cancellation is incomplete

The current full analysis worker supports cancellation tokens, but the cancellation is only checked:

- before full trajectory computation
- after full trajectory computation
- during validity analysis

The heavy full generation step itself is not yet deeply cancel-aware.

Implication:

- obsolete workers may continue computing even when their result will later be ignored
- this wastes CPU and can still create lag


## Current Python Performance Constraints

### The GIL

CPython uses a Global Interpreter Lock (GIL).

Practical consequences:

- multiple Python threads do not automatically mean true CPU parallelism
- if the heavy work is mostly Python bytecode, threads will compete for the GIL
- threads are still useful for responsiveness and offloading work away from the UI thread


### Collision code is not a large vectorized NumPy kernel

The collision system uses NumPy arrays and linear algebra operations, but the global flow is still heavily driven by Python loops and Python-level logic.

Current collision characteristics:

- pairwise collision loops are written in Python
- GJK orchestration is written in Python
- many small NumPy operations are used inside that algorithm

Implication:

- Python threads may not provide ideal CPU scaling for collision analysis
- if real parallel CPU scaling is required in Python, process-based execution may eventually be preferable
- in C++, this limitation disappears because the algorithm can run in native threads


### QThread is not the core problem

The main current performance issue is not simply that Qt threads are used.

The real issues are:

- full rebuild frequency
- duplicated work between preview and full generation
- incomplete cancellation
- full validity analysis being rerun too often

So the refactor should first improve architecture and work partitioning, not only replace `QThread`.


## Target Refactor Direction In Python

The target architecture should converge to a pipeline with explicit stages and long-lived workers.

### High-level target

We want:

1. a fast preview stage
2. a full trajectory generation stage
3. a parallel validity analysis stage
4. a central manager that owns revisioning, cancellation, and result aggregation


### Target logical components

#### `TrajectoryBuildManager`

Responsibilities:

- single orchestration entry point
- job revision tracking
- cancellation management
- latest-result-wins policy
- merging of preview, full trajectory, and validity outputs


#### `PreviewWorker`

Responsibilities:

- generate lightweight preview data only
- remain cancel-aware
- avoid heavy validation logic
- prioritize low latency for UI feedback


#### `FullTrajectoryWorker`

Responsibilities:

- generate complete trajectory samples
- remain cancel-aware
- keep sequential logic where required
- produce chunks of fully built samples that are ready for validity analysis


#### `ValidityAnalyzerManager`

Responsibilities:

- own a pool of long-lived validity workers
- receive validation tasks for already built samples
- distribute collision / TCP work
- return immutable validation results


#### `ValidityWorker`

Responsibilities:

- read a validation task
- evaluate collisions and TCP safety only
- return results without mutating the shared trajectory in place


## Recommended Data Flow

Target logical flow:

1. UI requests a new build.
2. `TrajectoryBuildManager` creates a new `revision_id`.
3. Previous active jobs are marked obsolete / cancelled.
4. `PreviewWorker` starts computing preview.
5. `FullTrajectoryWorker` starts computing full samples.
6. As chunks of complete samples become available, validation tasks are emitted.
7. `ValidityAnalyzerManager` dispatches those tasks to validity workers.
8. Validation results are returned to the manager.
9. Only the manager applies results if the `revision_id` still matches the current build.


## Recommended Concurrency Rules

### One writer rule

Strong recommendation:

- workers compute results
- only the manager mutates the shared current trajectory state

Why:

- easier reasoning
- fewer mutexes
- easier stale-result rejection
- easier C++ migration


### Use revision ids everywhere

Every important produced artifact should carry a `revision_id`.

Examples:

- preview result
- full trajectory result
- validation task
- validation result

This enables a strict latest-revision-wins policy.


### Prefer chunk-based validation tasks

Even if validation is logically sample-based, workers should preferably consume chunks instead of single samples.

Why:

- lower queue overhead
- better amortization of setup costs
- easier future C++ implementation
- better scalability than one tiny task per sample


## Future Incremental Rebuild Strategy

Incremental rebuild should not start from the assumption:

- "replace only segments n-1, n, n+1"

That assumption is too weak because of:

- chained LIN / CUBIC segments
- tangent propagation to neighbors
- time propagation
- IK / configuration continuity
- dynamic continuity

Recommended future approach:

1. identify a dirty start segment
2. expand left if needed for continuity or chain boundaries
3. reuse a known-safe prefix
4. rebuild the full suffix from that point
5. revalidate only the rebuilt suffix

This is more robust and maps well to both Python and C++.


## What Must Stay Stable For The C++ Migration

The most important thing is not the exact Python threading backend.
The most important thing is to stabilize the architecture and domain boundaries.

The following concepts should remain stable across the migration:

- trajectory build request
- preview trajectory
- full trajectory computation
- validation task
- validation result
- manager-based orchestration
- revision-based cancellation and stale-result rejection
- separation between sequential full generation and parallel heavy validation


## What Can Change In C++

The following implementation details are allowed to change during migration:

- Python threads / Qt threads / processes can become native C++ threads
- data transport can move from Python objects to C++ structs / vectors
- queue implementations can become lock-free or lower-overhead native queues
- collision implementation can become a fully native optimized algorithm
- thread affinity and priority management can become first-class concerns


## C++-Specific Opportunities

### Dedicated robot communication thread

The future C++ system is expected to contain a dedicated robot communication thread.

Desired properties:

- isolated from UI work
- isolated from heavy geometry / collision computation
- higher scheduling priority
- eventually bound to a dedicated core if the target platform allows it

Important note:

- this is a good C++ architecture goal
- it should not dictate premature Python complexity today


### Real native parallelism

In C++, the collision and validity pool can use real CPU parallelism without the Python GIL.

This makes the following design choices especially worthwhile today:

- long-lived worker pools
- explicit task queues
- immutable result objects
- clear manager / worker boundaries


### Better control over priorities and affinity

In C++, thread affinity and priority can become meaningful optimization tools for:

- robot communication
- deterministic scheduling separation
- heavy validity analysis isolation

However, these optimizations should only be applied after the architecture itself is correct and stable.


## What To Avoid During Migration

- Do not rewrite the system as a single monolithic worker again.
- Do not collapse preview, full build, and validity into one opaque execution stage.
- Do not let worker threads mutate shared trajectory state directly from many places.
- Do not tie the future C++ architecture to Python-specific constraints such as the GIL.
- Do not optimize affinity and low-level scheduling before the pipeline boundaries are well defined.


## Recommended Update Policy For This Document

This file should be updated whenever one of the following changes:

- preview pipeline responsibilities
- full build pipeline responsibilities
- validity analysis responsibilities
- concurrency model
- cancellation model
- incremental rebuild strategy
- C++ migration assumptions

Recommended maintenance rule:

- after each significant refactor milestone, update this document in the same development cycle


## Short Migration Summary

Current Python system:

- preview is fast but separate from full computation
- full computation still rebuilds from keypoints
- validity runs afterward
- cancellation is only partially cooperative
- collisions are expensive and still heavily driven by Python logic

Target Python system before C++ migration:

- manager-based orchestration
- long-lived workers
- explicit preview / full / validity stages
- revision-based cancellation
- parallel validity on already built sample chunks
- later, prefix reuse and suffix rebuild

Target C++ migration direction:

- keep the same logical architecture
- replace Python execution backends with native thread / task systems
- isolate robot communication in its own high-priority execution path
- use native parallelism for heavy validation workloads
