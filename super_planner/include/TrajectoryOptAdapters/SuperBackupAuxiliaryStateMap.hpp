#pragma once

#include "traj_opt/spline/SplineOptimizer.hpp"

#include <TrajectoryOptComponents/TimeMapUtils.hpp>
#include <data_structure/base/trajectory.h>
#include <utils/header/type_utils.hpp>

#include <algorithm>
#include <vector>

namespace traj_opt_adapters
{
class SuperBackupAuxiliaryStateMap
{
public:
    using SepticSpline = SplineTrajectory::SepticSplineND<3>;
    using Gradients = typename SepticSpline::Gradients;
    using WaypointsType = typename SepticSpline::MatrixType;
    using BoundaryConditions = SplineTrajectory::BoundaryConditions<3>;

    void reset(const geometry_utils::Trajectory *traj,
               double min_ts,
               double max_ts,
               double weight_ts,
               bool uniform_time,
               int piece_num,
               double initial_ts)
    {
        reference_traj_ = traj;
        min_ts_ = min_ts;
        max_ts_ = max_ts;
        weight_ts_ = weight_ts;
        uniform_time_ = uniform_time;
        piece_num_ = piece_num;
        initial_ts_ = initial_ts;
    }

    int getDimension() const { return 1 + (uniform_time_ ? 1 : 0); }

    Eigen::VectorXd getInitialValue(const std::vector<double> &ref_times,
                                    const WaypointsType & /*ref_waypoints*/,
                                    double /*ref_start_time*/,
                                    const BoundaryConditions & /*ref_bc*/) const
    {
        Eigen::VectorXd z(getDimension());
        int idx = 0;
        if (uniform_time_)
        {
            Eigen::VectorXd total_time(1);
            total_time(0) = 0.0;
            for (double t : ref_times)
            {
                total_time(0) += t;
            }
            Eigen::VectorXd tau_total;
            traj_opt_components::TimeMapUtils::backwardMapTToTau(total_time, tau_total);
            z(idx++) = tau_total(0);
        }

        double tau_ts = 0.0;
        const double clamped_ts = std::min(std::max(initial_ts_, min_ts_), max_ts_);
        traj_opt_components::TimeMapUtils::mapIntervalToInf(min_ts_, max_ts_, clamped_ts, tau_ts);
        z(idx) = tau_ts;
        return z;
    }

    void apply(const Eigen::VectorXd &z,
               std::vector<double> &times,
               WaypointsType &waypoints,
               double & /*start_time*/,
               BoundaryConditions &bc) const
    {
        int idx = 0;
        if (uniform_time_)
        {
            Eigen::VectorXd tau_total(1);
            tau_total(0) = z(idx++);
            Eigen::VectorXd total_time;
            traj_opt_components::TimeMapUtils::forwardMapTauToT(tau_total, total_time);
            const double seg_time = total_time(0) / static_cast<double>(piece_num_);
            for (int i = 0; i < static_cast<int>(times.size()); ++i)
            {
                times[i] = seg_time;
            }
        }

        super_utils::StatePVAJ state;
        reference_traj_->getState(decodeStartTime(z(idx)), state);
        waypoints.row(0) = state.col(0).transpose();
        bc.start_velocity = state.col(1);
        bc.start_acceleration = state.col(2);
        bc.start_jerk = state.col(3);
    }

    double backward(const Eigen::VectorXd &z,
                    const SepticSpline & /*spline*/,
                    const std::vector<double> &times,
                    const WaypointsType & /*waypoints*/,
                    double /*start_time*/,
                    const BoundaryConditions & /*bc*/,
                    Gradients &grads,
                    Eigen::VectorXd &grad_z) const
    {
        grad_z.resize(getDimension());
        grad_z.setZero();

        int idx = 0;
        if (uniform_time_)
        {
            const double grad_total_time = grads.times.sum() / static_cast<double>(piece_num_);
            Eigen::VectorXd tau_total(1);
            tau_total(0) = z(idx);
            Eigen::VectorXd grad_total(1);
            grad_total(0) = grad_total_time;
            Eigen::VectorXd grad_tau_total;
            traj_opt_components::TimeMapUtils::propagateGradientTToTau(tau_total, grad_total, grad_tau_total);
            grad_z(idx++) = grad_tau_total(0);
            grads.times.setZero();
        }

        const double tau_ts = z(idx);
        const double ts = decodeStartTime(tau_ts);
        double grad_ts =
            grads.start.p.dot(reference_traj_->getVel(ts)) +
            grads.start.v.dot(reference_traj_->getAcc(ts)) +
            grads.start.a.dot(reference_traj_->getJer(ts)) +
            grads.start.j.dot(reference_traj_->getSnap(ts));

        double extra_cost = 0.0;
        if (weight_ts_ > 0.0)
        {
            extra_cost += weight_ts_ * (max_ts_ - ts);
            grad_ts -= weight_ts_;
        }

        traj_opt_components::TimeMapUtils::propagateGradIntervalToInf(min_ts_, max_ts_, tau_ts, grad_ts, grad_z(idx));
        return extra_cost;
    }

    double decodeStartTime(double tau_ts) const
    {
        double ts = initial_ts_;
        traj_opt_components::TimeMapUtils::mapInfToInterval(min_ts_, max_ts_, tau_ts, ts);
        return ts;
    }

private:
    const geometry_utils::Trajectory *reference_traj_ = nullptr;
    double min_ts_ = 0.0;
    double max_ts_ = 0.0;
    double weight_ts_ = 0.0;
    bool uniform_time_ = false;
    int piece_num_ = 1;
    double initial_ts_ = 0.0;
};
} // namespace traj_opt_adapters
