#pragma once

#include "traj_opt/spline/SplineOptimizer.hpp"

#include <TrajectoryOptAdapters/SuperBackupAuxiliaryStateMap.hpp>
#include <TrajectoryOptAdapters/SuperBackupIntegralCostAdapter.hpp>
#include <TrajectoryOptAdapters/SuperExpIntegralCostAdapter.hpp>
#include <TrajectoryOptAdapters/SuperTrajectoryConversion.hpp>
#include <TrajectoryOptComponents/LinearTimeCost.hpp>
#include <TrajectoryOptComponents/PolytopeSpatialMap.hpp>
#include <utils/header/type_utils.hpp>

namespace traj_opt::spline_opt
{
using super_utils::Mat3Df;
using super_utils::PolyhedraH;
using super_utils::PolyhedraV;
using super_utils::PolyhedronH;
using super_utils::StatePVAJ;
using super_utils::Vec3f;
using super_utils::Vec4f;
using super_utils::VecDf;

using SepticSpline = SplineTrajectory::SepticSplineND<3>;
using BoundaryConditions = SplineTrajectory::BoundaryConditions<3>;
using SepticGradients = typename SepticSpline::Gradients;
using WaypointsType = typename SepticSpline::CoefficientMatrix;
using LinearTimeCost = traj_opt_components::LinearTimeCost;
using PolytopeSpatialMap = traj_opt_components::PolytopeSpatialMap;
using ExpPenaltyIntegralCost = traj_opt_adapters::SuperExpIntegralCostAdapter;
using BackupPenaltyIntegralCost = traj_opt_adapters::SuperBackupIntegralCostAdapter;
using BackupAuxiliaryStateMap = traj_opt_adapters::SuperBackupAuxiliaryStateMap;
using traj_opt_adapters::splineToSuperTrajectory;

/** @brief Add mapped-domain penalties after the prepared spatial pullback. */
struct DomainCost
{
    const PolytopeSpatialMap &space;
    double operator()(const SplineTrajectory::DecisionView &state,
                      Eigen::Ref<Eigen::VectorXd> gradient) const
    {
        if (state.layout.waypoints.empty()) return 0.0;
        const int offset = state.layout.waypoints.front().offset;
        const int count = state.layout.boundary_offset - offset;
        double cost = 0.0;
        space.addNormPenalty(state.variables, offset, count, gradient, cost);
        return cost;
    }
};
} // namespace traj_opt::spline_opt
