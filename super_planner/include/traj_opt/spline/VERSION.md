# SplineTrajectory integration

Source: SplineTrajectory main, commit `126525e49a43b0548bc6960e4979dd6b6258f289` (2026-09-12).
The three public headers, including comments and formatting, are byte-identical to upstream.
[Optimizer protocols](https://github.com/Bziyue/SplineTrajectory/blob/126525e49a43b0548bc6960e4979dd6b6258f289/docs/optimizer.md) · [Migration guide](https://github.com/Bziyue/SplineTrajectory/blob/126525e49a43b0548bc6960e4979dd6b6258f289/docs/api_migration.md) ·
[Examples](https://github.com/Bziyue/SplineTrajectory/blob/126525e49a43b0548bc6960e4979dd6b6258f289/examples/README.md). MIT and MINVO notices are preserved alongside this file.

| Header | SHA-256 |
| --- | --- |
| `SplineTrajectory.hpp` | `9d5c3eca996a13104e13bcc668cf73e50367db5cb53faf4713f0cd1c041106dc` |
| `SplineOptimizer.hpp` | `6f4ae9c547903264b726f7bf1f28a683a02426cdf8197ee4719b04b9d541e7bf` |
| `SplineConvexHull.hpp` | `221a30e2247f6c71218d7e02e7228e802a60c820574f11f24b442da9bbb09c33` |

The optimizer owns preparation, layout, polynomial and evaluation buffers. Concrete parameterizations
borrow immutable planner maps for one solve; ordinary named objective structs bind duration,
integral and decision costs. Preparation fixes integration options and encodes the initial coordinates once; `initialGuess()`
returns a copy without repeating inverse-map solves, and callbacks reuse storage.
`backwardInto()` writes spatial gradients directly and domain penalties accumulate in those same
slices. The public polynomial and diagnostic parameter views are read-only. Independent samplers
own their traversal state and require no source warm-up.

To synchronize, copy the three original headers from the recorded upstream commit, verify their
hashes, update this record and migrate application adapters. Preserve the existing numerical-failure,
interval and convex-hull behavior; do not introduce downstream header variants.

## Planner integration

Exploration and backup use the same named duration/integral/decision stages. The accepted vector
is reevaluated once before consuming penalty diagnostics; output and diagnostics therefore describe
the same candidate. Conversion to SUPER trajectory storage accepts the immutable polynomial directly.

Backup uniform time has only one total-duration auxiliary coordinate; inactive per-segment duration
coordinates and duplicate dimension counters are removed. The auxiliary map snapshots the reference
polynomial during preparation and uses combined derivative queries, scalar time maps and preallocated
gradient slices. The current layout supplies all variable offsets. Unused duplicate corridor setup
methods have been removed, and one segment per corridor is indexed directly.

`super_spline_auxiliary_regression` checks moving-boundary and uniform-time gradients, variable counts
and Eigen allocation reuse. `backend_equivalence backup` exercises both duration layouts and both
spatial maps with C3 seam checks. The existing stability and mapped-gradient checks remain available.

## Validation (2026-09-12)

GCC 9.4, C++17 Release and ROS1 Noetic: the planner library and backend regression programs built.
Mapped gradients and all three exploration stability cases passed. Backup solves passed for both
uniform/nonuniform time and identity/polytope spatial maps, with zero measured C3 seam residual.
The auxiliary regression passed finite differences (maximum absolute error `1.17e-8`), expected
variable counts and a prepared evaluation with Eigen allocations disabled.
No warnings originated in the synchronized public headers; existing planner signedness and
initialization-order warnings remain outside this migration.

## Paired integration benchmark

Baseline: SUPER `cbee603`, using the preceding SplineTrajectory integration. The old executable was
run with its original `libsuper.so`; both versions used the same GCC 9.4 Release settings and
`static_high_speed.yaml`, with relative cost tolerance `5e-6`.
AMD Ryzen 9 7945HX, one pinned logical CPU; one traced pair followed by 10 alternating pairs per
perturbation. Median complete `optimize()` times in microseconds:

| Perturbation | Before | After |
| --- | ---: | ---: |
| -1 | 1395.09 | 1380.25 |
| +0 | 1481.17 | 1460.39 |
| +1 | 2174.56 | 2175.15 |

Initial variables, costs and gradients, final duration and sampled P/V, and L-BFGS evaluation
counts were exactly equal in all three traced comparisons. Complete latency remained within about
1.5% of baseline; these measurements support parity rather than a substantial speedup.

After the catkin build, with its overlay loaded and a running ROS1 master:

```sh
rosrun super_planner super_spline_map_regression
rosrun super_planner super_spline_auxiliary_regression
rosrun super_planner backend_equivalence backup
SUPER_EQUIVALENCE_TRACE=1 rosrun super_planner backend_equivalence 0 5e-6
```

Use `scripts/compare_backend_equivalence.py BEFORE_LOG AFTER_LOG` for the paired traces and
`scripts/check_backend_stability.py BACKEND_EXECUTABLE` for the three exploration cases.
Raw logs are retained in the integration workspace's `.scratch/downstream-sync/paired-final/`;
they are local artifacts, not repository attachments. No flight or collision simulation was run.
