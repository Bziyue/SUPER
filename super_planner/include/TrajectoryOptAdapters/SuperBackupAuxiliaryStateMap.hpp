#pragma once

#include "traj_opt/spline/SplineOptimizer.hpp"
#include <TrajectoryOptComponents/TimeMapUtils.hpp>
#include <data_structure/base/trajectory.h>

#include <algorithm>
#include <numeric>

namespace traj_opt_adapters
{
/** @brief Map uniform duration and switch-time coordinates to a backup trajectory.
 * @note Owns an immutable reference polynomial snapshot. One active solve exclusively
 *       owns this map; reset it before preparing a different backup problem. */
class SuperBackupAuxiliaryStateMap
{
public:
    using Spline = SplineTrajectory::SepticSplineND<3>;

    /** @brief Prepare the reference snapshot and auxiliary-coordinate configuration.
     * @param traj Non-null reference in local seconds; borrowed only during this call.
     * @param min_ts Lower switch-time bound in reference-local seconds.
     * @param max_ts Upper switch-time bound in reference-local seconds.
     * @param weight_ts Nonnegative progress weight favoring later switches.
     * @param uniform_time Whether one auxiliary coordinate controls all durations.
     * @param piece_num Positive number of backup segments.
     * @param initial_ts Initial switch time, clamped to the specified interval. */
    void reset(const geometry_utils::Trajectory *traj, double min_ts, double max_ts,
               double weight_ts, bool uniform_time, int piece_num, double initial_ts)
    {
        if (!traj || traj->empty() || piece_num <= 0)
            throw std::invalid_argument("Invalid backup reference or topology");
        std::vector<double> knots{0.0};
        knots.reserve(traj->size() + 1);
        Spline::CoefficientMatrix coefficients =
            Spline::CoefficientMatrix::Zero(traj->size() * Spline::kCoefficientCount, 3);
        for (std::size_t i = 0; i < traj->size(); ++i)
        {
            const auto &piece = (*traj)[static_cast<int>(i)];
            const auto &source = piece.getCoeffMat();
            if (source.rows() != 3 || source.cols() <= 0 || source.cols() > Spline::kCoefficientCount)
                throw std::invalid_argument("Unsupported backup reference degree");
            coefficients.middleRows(i * Spline::kCoefficientCount, source.cols()) =
                source.rowwise().reverse().transpose();
            knots.push_back(knots.back() + piece.getDuration());
        }
        reference_ = Spline::Polynomial(knots, coefficients, Spline::kCoefficientCount);
        if (!reference_.isValid()) throw std::invalid_argument("Invalid backup reference polynomial");
        min_ts_ = min_ts;
        max_ts_ = max_ts;
        weight_ts_ = std::max(0.0, weight_ts);
        uniform_time_ = uniform_time;
        piece_num_ = piece_num;
        initial_ts_ = initial_ts;
    }

    int dimension() const { return 1 + (uniform_time_ ? 1 : 0); }

    /** @brief Encode the reference total duration and bounded switch time; may allocate. */
    Eigen::VectorXd initial(const SplineTrajectory::SplineProblem<Spline> &problem) const
    {
        Eigen::VectorXd variables(dimension());
        int index = 0;
        if (uniform_time_)
            variables(index++) = time_map_.toTau(
                std::accumulate(problem.durations.begin(), problem.durations.end(), 0.0));
        const double initial = std::clamp(initial_ts_, min_ts_, max_ts_);
        traj_opt_components::TimeMapUtils::mapIntervalToInf(min_ts_, max_ts_, initial, variables(index));
        return variables;
    }

    /** @brief Decode durations and the reference P/V/A/J boundary without heap allocation.
     * @param variables Prepared auxiliary coordinates, borrowed for this call.
     * @param[in,out] state Physical backup parameters; topology and time origin stay fixed. */
    void apply(const Eigen::Ref<const Eigen::VectorXd> &variables,
               SplineTrajectory::MutableParameters<Spline> &state) const
    {
        int index = 0;
        if (uniform_time_)
        {
            const double duration = time_map_.toTime(variables(index++)) / piece_num_;
            std::fill(state.durations.begin(), state.durations.end(), duration);
        }
        const auto boundary = reference_.evaluateDerivatives<3>(decodeStartTime(variables(index)));
        state.waypoints.row(0) = boundary[0].transpose();
        state.boundary.start_velocity = boundary[1];
        state.boundary.start_acceleration = boundary[2];
        state.boundary.start_jerk = boundary[3];
    }

    /** @brief Pull back total duration and the moving boundary, adding the progress cost.
     * @param variables Prepared auxiliary coordinates.
     * @param state Current physical trajectory; borrowed for this call.
     * @param[in,out] gradient Accumulated physical derivatives; uniform time clears duration partials.
     * @param[in,out] output Preallocated auxiliary derivatives, accumulated without resizing.
     * @return Nonnegative linear progress cost; the backup time origin remains zero. */
    double backward(const Eigen::Ref<const Eigen::VectorXd> &variables,
                    const SplineTrajectory::ParameterView<Spline> & /*state*/,
                    SplineTrajectory::ParameterGradient<Spline> &gradient,
                    Eigen::Ref<Eigen::VectorXd> output) const
    {
        int index = 0;
        if (uniform_time_)
        {
            const double tau = variables(index);
            output(index++) += time_map_.backward(tau, time_map_.toTime(tau),
                                                  gradient.durations.sum() / piece_num_);
            gradient.durations.setZero();
        }
        const double switch_time = decodeStartTime(variables(index));
        const auto state = reference_.evaluateDerivatives<4>(switch_time);
        const double partial = gradient.start.p.dot(state[1]) + gradient.start.v.dot(state[2]) +
                               gradient.start.a.dot(state[3]) + gradient.start.j.dot(state[4]) - weight_ts_;
        double mapped_partial;
        traj_opt_components::TimeMapUtils::propagateGradIntervalToInf(
            min_ts_, max_ts_, variables(index), partial, mapped_partial);
        output(index) += mapped_partial;
        return weight_ts_ * (max_ts_ - switch_time);
    }

    double decodeStartTime(double variable) const
    {
        double time;
        traj_opt_components::TimeMapUtils::mapInfToInterval(min_ts_, max_ts_, variable, time);
        return time;
    }

private:
    Spline::Polynomial reference_;
    SplineTrajectory::QuadInvTimeMap time_map_;
    double min_ts_ = 0.0, max_ts_ = 0.0, weight_ts_ = 0.0, initial_ts_ = 0.0;
    bool uniform_time_ = false;
    int piece_num_ = 1;
};
} // namespace traj_opt_adapters
