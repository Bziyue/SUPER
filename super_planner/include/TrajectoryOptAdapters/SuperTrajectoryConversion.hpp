#pragma once

#include <data_structure/base/trajectory.h>

namespace traj_opt_adapters
{
/** @brief Copy an ascending-power polynomial into SUPER's descending-power trajectory storage.
 * @param ppoly Valid polynomial, borrowed only for this call.
 * @return Owning SUPER trajectory with the same segment durations and a local time origin of zero. */
template <typename Polynomial>
inline geometry_utils::Trajectory splineToSuperTrajectory(const Polynomial &ppoly)
{
    geometry_utils::Trajectory traj;
    traj.clear();
    traj.reserve(ppoly.numSegments());
    for (int i = 0; i < ppoly.numSegments(); ++i)
    {
        const auto seg = ppoly[i];
        traj.emplace_back(seg.duration(), seg.coefficients().transpose().rowwise().reverse());
    }
    return traj;
}
} // namespace traj_opt_adapters
