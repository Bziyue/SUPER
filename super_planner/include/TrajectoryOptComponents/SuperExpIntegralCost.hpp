#pragma once

#include <TrajectoryOptComponents/SpatialCosts/AccelerationBoundPenalty.hpp>
#include <TrajectoryOptComponents/SpatialCosts/AngularRateBoundPenalty.hpp>
#include <TrajectoryOptComponents/SpatialCosts/FlatnessState.hpp>
#include <TrajectoryOptComponents/SpatialCosts/JerkBoundPenalty.hpp>
#include <TrajectoryOptComponents/SpatialCosts/PolytopePositionPenalty.hpp>
#include <TrajectoryOptComponents/SpatialCosts/SegmentBoundaryAttractorPenalty.hpp>
#include <TrajectoryOptComponents/SpatialCosts/ThrustBandPenalty.hpp>
#include <TrajectoryOptComponents/SpatialCosts/VelocityBoundPenalty.hpp>
#include <utils/geometry/quadrotor_flatness.hpp>
#include <utils/header/type_utils.hpp>

#include <vector>

namespace traj_opt_components
{
using super_utils::Mat3Df;
using super_utils::PolyhedraH;
using super_utils::Vec4f;
using super_utils::VecDf;

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
        local_cost += accumulatePolytopePositionPenalty((*h_polys)[poly_id],
                                                        p,
                                                        smooth_eps,
                                                        weight_pos,
                                                        grad_pos,
                                                        &max_violation_(1));
        local_cost += accumulateSegmentBoundaryAttractorPenalty(t,
                                                                seg_idx,
                                                                segment_times_,
                                                                waypoint_attractor,
                                                                waypoint_attractor_dead_d,
                                                                p,
                                                                smooth_eps,
                                                                weight_att,
                                                                grad_pos,
                                                                &max_violation_(5));
        local_cost += accumulateVelocityBoundPenalty(v,
                                                     magnitude_bounds(0),
                                                     smooth_eps,
                                                     weight_vel,
                                                     grad_vel,
                                                     &max_violation_(2));
        local_cost += accumulateAccelerationBoundPenalty(a,
                                                         magnitude_bounds(1),
                                                         smooth_eps,
                                                         weight_acc,
                                                         grad_acc,
                                                         &max_violation_(3));
        local_cost += accumulateJerkBoundPenalty(j,
                                                 magnitude_bounds(2),
                                                 smooth_eps,
                                                 weight_jer,
                                                 grad_jer,
                                                 &max_violation_(4));

        Eigen::Vector3d total_grad_pos = grad_pos;
        Eigen::Vector3d total_grad_vel = grad_vel;
        Eigen::Vector3d total_grad_acc = grad_acc;
        Eigen::Vector3d total_grad_jer = grad_jer;

        if (weight_omg > 0.0 || weight_acc_thr > 0.0)
        {
            const auto flatness_state =
                evaluateFlatnessPenaltyState(flatmap, v, a, j);
            Eigen::Vector3d grad_omg = Eigen::Vector3d::Zero();
            double grad_thr = 0.0;

            local_cost += accumulateAngularRateBoundPenalty(flatness_state.angular_rate,
                                                            magnitude_bounds(3),
                                                            smooth_eps,
                                                            weight_omg,
                                                            grad_omg,
                                                            &max_violation_(6));
            local_cost += accumulateThrustBandPenalty(flatness_state.thrust,
                                                      magnitude_bounds(4),
                                                      magnitude_bounds(5),
                                                      smooth_eps,
                                                      weight_acc_thr,
                                                      grad_thr,
                                                      &max_violation_(7));

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
} // namespace traj_opt_components
