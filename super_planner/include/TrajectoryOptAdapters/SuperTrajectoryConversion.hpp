#pragma once

#include <data_structure/base/trajectory.h>

namespace traj_opt_adapters
{
template <typename SplineType>
inline geometry_utils::Trajectory splineToSuperTrajectory(const SplineType &spline)
{
    geometry_utils::Trajectory traj;
    const auto &ppoly = spline.getTrajectory();
    traj.clear();
    traj.reserve(ppoly.getNumSegments());
    for (int i = 0; i < ppoly.getNumSegments(); ++i)
    {
        const auto seg = ppoly[i];
        traj.emplace_back(seg.duration(), seg.getCoeffs().transpose().rowwise().reverse());
    }
    return traj;
}
} // namespace traj_opt_adapters
