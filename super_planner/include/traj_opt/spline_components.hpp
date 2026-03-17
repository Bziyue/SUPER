#pragma once

#include "traj_opt/spline/SplineOptimizer.hpp"

#include <TrajectoryOptComponents/LinearTimeCost.hpp>
#include <TrajectoryOptComponents/PenaltyUtils.hpp>
#include <TrajectoryOptComponents/PolytopeSpatialMap.hpp>
#include <TrajectoryOptComponents/TimeMapUtils.hpp>
#include <data_structure/base/trajectory.h>
#include <utils/geometry/quadrotor_flatness.hpp>
#include <utils/header/type_utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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
using WaypointsType = typename SepticSpline::MatrixType;
using LinearTimeCost = traj_opt_components::LinearTimeCost;
using PolytopeSpatialMap = traj_opt_components::PolytopeSpatialMap;

inline geometry_utils::Trajectory splineToSuperTrajectory(const SepticSpline &spline)
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

class ExpPenaltyIntegralCost
{
public:
    const PolyhedraH *h_polys = nullptr;
    const Eigen::VectorXi *h_poly_idx = nullptr;
    const Mat3Df *waypoint_attractor = nullptr;
    const VecDf *waypoint_attractor_dead_d = nullptr;
    double smooth_eps = 0.0;
    VecDf magnitude_bounds;
    VecDf penalty_weights;
    flatness::FlatnessMap *flatmap = nullptr;

    void reset(const PolyhedraH *polys,
               const Eigen::VectorXi *indices,
               const Mat3Df *attractor,
               const VecDf *attractor_dead_d,
               double smoothing,
               const VecDf &magnitudeBounds,
               const VecDf &penaltyWeights,
               flatness::FlatnessMap *fm)
    {
        h_polys = polys;
        h_poly_idx = indices;
        waypoint_attractor = attractor;
        waypoint_attractor_dead_d = attractor_dead_d;
        smooth_eps = smoothing;
        magnitude_bounds = magnitudeBounds;
        penalty_weights = penaltyWeights;
        flatmap = fm;
    }

    void beginEvaluation(const std::vector<double> *times)
    {
        segment_times_ = times;
        max_violation_.resize(8);
        max_violation_.setZero();
    }

    const VecDf &getPenaltyLog() const { return max_violation_; }

    double operator()(double t,
                      double /*t_global*/,
                      int seg_idx,
                      const Eigen::Vector3d &p,
                      const Eigen::Vector3d &v,
                      const Eigen::Vector3d &a,
                      const Eigen::Vector3d &j,
                      const Eigen::Vector3d & /*s*/,
                      Eigen::Vector3d &gp,
                      Eigen::Vector3d &gv,
                      Eigen::Vector3d &ga,
                      Eigen::Vector3d &gj,
                      Eigen::Vector3d & /*gs*/,
                      double & /*gt*/) const
    {
        if (!h_polys || !h_poly_idx || !flatmap)
        {
            return 0.0;
        }

        const double vmax_sqr = magnitude_bounds(0) * magnitude_bounds(0);
        const double amax_sqr = magnitude_bounds(1) * magnitude_bounds(1);
        const double jmax_sqr = magnitude_bounds(2) * magnitude_bounds(2);
        const double omgmax_sqr = magnitude_bounds(3) * magnitude_bounds(3);
        const double accthrmin = magnitude_bounds(4);
        const double accthrmax = magnitude_bounds(5);
        const double thrust_mean = 0.5 * (accthrmax + accthrmin);
        const double thrust_radi = 0.5 * std::abs(accthrmax - accthrmin);
        const double thrust_sqr_radi = thrust_radi * thrust_radi;

        const double weight_pos = penalty_weights(0);
        const double weight_vel = penalty_weights(1);
        const double weight_acc = penalty_weights(2);
        const double weight_jer = penalty_weights(3);
        const double weight_att = penalty_weights(4);
        const double weight_omg = penalty_weights(5);
        const double weight_acc_thr = penalty_weights(6);

        double local_cost = 0.0;
        Eigen::Vector3d grad_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_jer = Eigen::Vector3d::Zero();

        const int poly_id = (*h_poly_idx)(seg_idx);
        const auto &poly = (*h_polys)[poly_id];
        if (weight_pos > 0.0)
        {
            for (int k = 0; k < poly.rows(); ++k)
            {
                const Eigen::Vector3d outer_normal = poly.block<1, 3>(k, 0);
                const double viola_pos = outer_normal.dot(p) + poly(k, 3);
                max_violation_(1) = std::max(max_violation_(1), viola_pos);
                double pena = 0.0;
                double pena_d = 0.0;
                if (traj_opt_components::smoothedL1(viola_pos, smooth_eps, pena, pena_d))
                {
                    grad_pos += weight_pos * pena_d * outer_normal;
                    local_cost += weight_pos * pena;
                }
            }
        }

        if (weight_att > 0.0 && waypoint_attractor && waypoint_attractor_dead_d && segment_times_)
        {
            const double seg_duration = (*segment_times_)[seg_idx];
            const double eps = std::max(1e-9, 1e-7 * std::max(1.0, seg_duration));
            const bool is_start_node = (seg_idx != 0) && (std::abs(t) <= eps);
            const bool is_end_node =
                (seg_idx != static_cast<int>(segment_times_->size()) - 1) && (std::abs(t - seg_duration) <= eps);
            if (is_start_node || is_end_node)
            {
                const int idx = is_end_node ? seg_idx : seg_idx - 1;
                const Eigen::Vector3d delta = p - waypoint_attractor->col(idx);
                const double dead_d = (*waypoint_attractor_dead_d)(idx);
                const double viola_att = delta.squaredNorm() - dead_d * dead_d;
                max_violation_(5) = std::max(max_violation_(5), viola_att);
                double pena = 0.0;
                double pena_d = 0.0;
                if (traj_opt_components::smoothedL1(viola_att, smooth_eps, pena, pena_d))
                {
                    grad_pos += weight_att * pena_d * 2.0 * delta;
                    local_cost += weight_att * pena;
                }
            }
        }

        const double viola_vel = v.squaredNorm() - vmax_sqr;
        double pena = 0.0;
        double pena_d = 0.0;
        if (weight_vel > 0.0 &&
            traj_opt_components::smoothedL1(viola_vel, smooth_eps, pena, pena_d))
        {
            grad_vel += weight_vel * pena_d * 2.0 * v;
            local_cost += weight_vel * pena;
            max_violation_(2) = std::max(max_violation_(2), viola_vel);
        }

        const double viola_acc = a.squaredNorm() - amax_sqr;
        if (weight_acc > 0.0 &&
            traj_opt_components::smoothedL1(viola_acc, smooth_eps, pena, pena_d))
        {
            grad_acc += weight_acc * pena_d * 2.0 * a;
            local_cost += weight_acc * pena;
            max_violation_(3) = std::max(max_violation_(3), viola_acc);
        }

        const double viola_jer = j.squaredNorm() - jmax_sqr;
        if (weight_jer > 0.0 &&
            traj_opt_components::smoothedL1(viola_jer, smooth_eps, pena, pena_d))
        {
            grad_jer += weight_jer * pena_d * 2.0 * j;
            local_cost += weight_jer * pena;
            max_violation_(4) = std::max(max_violation_(4), viola_jer);
        }

        Eigen::Vector3d total_grad_pos = grad_pos;
        Eigen::Vector3d total_grad_vel = grad_vel;
        Eigen::Vector3d total_grad_acc = grad_acc;
        Eigen::Vector3d total_grad_jer = grad_jer;

        if (weight_omg > 0.0 && weight_acc_thr > 0.0)
        {
            double thr = 0.0;
            Vec4f quat = Vec4f::Zero();
            Eigen::Vector3d omg = Eigen::Vector3d::Zero();
            flatmap->forward(v, a, j, 0.0, 0.0, thr, quat, omg);

            Eigen::Vector3d grad_omg = Eigen::Vector3d::Zero();
            double grad_thr = 0.0;

            const double viola_omg = omg.squaredNorm() - omgmax_sqr;
            if (weight_omg > 0.0 &&
                traj_opt_components::smoothedL1(viola_omg, smooth_eps, pena, pena_d))
            {
                grad_omg += weight_omg * pena_d * 2.0 * omg;
                local_cost += weight_omg * pena;
                max_violation_(6) = std::max(max_violation_(6), viola_omg);
            }

            const double viola_thrust = (thr - thrust_mean) * (thr - thrust_mean) - thrust_sqr_radi;
            if (weight_acc_thr > 0.0 &&
                traj_opt_components::smoothedL1(viola_thrust, smooth_eps, pena, pena_d))
            {
                grad_thr += weight_acc_thr * pena_d * 2.0 * (thr - thrust_mean);
                local_cost += weight_acc_thr * pena;
                max_violation_(7) = std::max(max_violation_(7), viola_thrust);
            }

            double total_grad_psi = 0.0;
            double total_grad_psid = 0.0;
            flatmap->backward(grad_pos,
                              grad_vel,
                              grad_acc,
                              grad_jer,
                              grad_thr,
                              Vec4f::Zero(),
                              grad_omg,
                              total_grad_pos,
                              total_grad_vel,
                              total_grad_acc,
                              total_grad_jer,
                              total_grad_psi,
                              total_grad_psid);
        }

        gp += total_grad_pos;
        gv += total_grad_vel;
        ga += total_grad_acc;
        gj += total_grad_jer;
        return local_cost;
    }

private:
    mutable const std::vector<double> *segment_times_ = nullptr;
    mutable VecDf max_violation_{VecDf::Zero(8)};
};

class BackupPenaltyIntegralCost
{
public:
    const PolyhedronH *h_poly = nullptr;
    double smooth_eps = 0.0;
    VecDf magnitude_bounds;
    VecDf penalty_weights;
    flatness::FlatnessMap *flatmap = nullptr;

    void reset(const PolyhedronH *poly,
               double smoothing,
               const VecDf &magnitudeBounds,
               const VecDf &penaltyWeights,
               flatness::FlatnessMap *fm)
    {
        h_poly = poly;
        smooth_eps = smoothing;
        magnitude_bounds = magnitudeBounds;
        penalty_weights = penaltyWeights;
        flatmap = fm;
    }

    void beginEvaluation()
    {
        max_violation_.resize(8);
        max_violation_.setZero();
    }

    const VecDf &getPenaltyLog() const { return max_violation_; }

    double operator()(double /*t*/,
                      double /*t_global*/,
                      int /*seg_idx*/,
                      const Eigen::Vector3d &p,
                      const Eigen::Vector3d &v,
                      const Eigen::Vector3d &a,
                      const Eigen::Vector3d &j,
                      const Eigen::Vector3d & /*s*/,
                      Eigen::Vector3d &gp,
                      Eigen::Vector3d &gv,
                      Eigen::Vector3d &ga,
                      Eigen::Vector3d &gj,
                      Eigen::Vector3d & /*gs*/,
                      double & /*gt*/) const
    {
        if (!h_poly || !flatmap)
        {
            return 0.0;
        }

        const double vmax_sqr = magnitude_bounds(0) * magnitude_bounds(0);
        const double amax_sqr = magnitude_bounds(1) * magnitude_bounds(1);
        const double jmax_sqr = magnitude_bounds(2) * magnitude_bounds(2);
        const double omgmax_sqr = magnitude_bounds(3) * magnitude_bounds(3);
        const double accthrmin = magnitude_bounds(4);
        const double accthrmax = magnitude_bounds(5);
        const double thrust_mean = 0.5 * (accthrmax + accthrmin);
        const double thrust_radi = 0.5 * std::abs(accthrmax - accthrmin);
        const double thrust_sqr_radi = thrust_radi * thrust_radi;

        const double weight_pos = penalty_weights(0);
        const double weight_vel = penalty_weights(1);
        const double weight_acc = penalty_weights(2);
        const double weight_jer = penalty_weights(3);
        const double weight_omg = penalty_weights(5);
        const double weight_acc_thr = penalty_weights(6);

        double local_cost = 0.0;
        Eigen::Vector3d grad_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d grad_jer = Eigen::Vector3d::Zero();

        if (weight_pos > 0.0)
        {
            for (int k = 0; k < h_poly->rows(); ++k)
            {
                const Eigen::Vector3d outer_normal = h_poly->block<1, 3>(k, 0);
                const double viola_pos = outer_normal.dot(p) + (*h_poly)(k, 3);
                max_violation_(1) = std::max(max_violation_(1), viola_pos);
                double pena = 0.0;
                double pena_d = 0.0;
                if (traj_opt_components::smoothedL1(viola_pos, smooth_eps, pena, pena_d))
                {
                    grad_pos += weight_pos * pena_d * outer_normal;
                    local_cost += weight_pos * pena;
                }
            }
        }

        double pena = 0.0;
        double pena_d = 0.0;
        const double viola_vel = v.squaredNorm() - vmax_sqr;
        if (weight_vel > 0.0 &&
            traj_opt_components::smoothedL1(viola_vel, smooth_eps, pena, pena_d))
        {
            grad_vel += weight_vel * pena_d * 2.0 * v;
            local_cost += weight_vel * pena;
            max_violation_(2) = std::max(max_violation_(2), viola_vel);
        }

        const double viola_acc = a.squaredNorm() - amax_sqr;
        if (weight_acc > 0.0 &&
            traj_opt_components::smoothedL1(viola_acc, smooth_eps, pena, pena_d))
        {
            grad_acc += weight_acc * pena_d * 2.0 * a;
            local_cost += weight_acc * pena;
            max_violation_(3) = std::max(max_violation_(3), viola_acc);
        }

        const double viola_jer = j.squaredNorm() - jmax_sqr;
        if (weight_jer > 0.0 &&
            traj_opt_components::smoothedL1(viola_jer, smooth_eps, pena, pena_d))
        {
            grad_jer += weight_jer * pena_d * 2.0 * j;
            local_cost += weight_jer * pena;
            max_violation_(4) = std::max(max_violation_(4), viola_jer);
        }

        Eigen::Vector3d total_grad_pos = grad_pos;
        Eigen::Vector3d total_grad_vel = grad_vel;
        Eigen::Vector3d total_grad_acc = grad_acc;
        Eigen::Vector3d total_grad_jer = grad_jer;

        if (weight_omg > 0.0 && weight_acc_thr > 0.0)
        {
            double thr = 0.0;
            Vec4f quat = Vec4f::Zero();
            Eigen::Vector3d omg = Eigen::Vector3d::Zero();
            flatmap->forward(v, a, j, 0.0, 0.0, thr, quat, omg);

            Eigen::Vector3d grad_omg = Eigen::Vector3d::Zero();
            double grad_thr = 0.0;

            const double viola_omg = omg.squaredNorm() - omgmax_sqr;
            if (weight_omg > 0.0 &&
                traj_opt_components::smoothedL1(viola_omg, smooth_eps, pena, pena_d))
            {
                grad_omg += weight_omg * pena_d * 2.0 * omg;
                local_cost += weight_omg * pena;
                max_violation_(6) = std::max(max_violation_(6), viola_omg);
            }

            const double viola_thrust = (thr - thrust_mean) * (thr - thrust_mean) - thrust_sqr_radi;
            if (weight_acc_thr > 0.0 &&
                traj_opt_components::smoothedL1(viola_thrust, smooth_eps, pena, pena_d))
            {
                grad_thr += weight_acc_thr * pena_d * 2.0 * (thr - thrust_mean);
                local_cost += weight_acc_thr * pena;
                max_violation_(7) = std::max(max_violation_(7), viola_thrust);
            }

            double total_grad_psi = 0.0;
            double total_grad_psid = 0.0;
            flatmap->backward(grad_pos,
                              grad_vel,
                              grad_acc,
                              grad_jer,
                              grad_thr,
                              Vec4f::Zero(),
                              grad_omg,
                              total_grad_pos,
                              total_grad_vel,
                              total_grad_acc,
                              total_grad_jer,
                              total_grad_psi,
                              total_grad_psid);
        }

        gp += total_grad_pos;
        gv += total_grad_vel;
        ga += total_grad_acc;
        gj += total_grad_jer;
        return local_cost;
    }

private:
    mutable VecDf max_violation_{VecDf::Zero(8)};
};

class BackupAuxiliaryStateMap
{
public:
    using Gradients = SepticGradients;

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

        StatePVAJ state;
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
} // namespace traj_opt::spline_opt
