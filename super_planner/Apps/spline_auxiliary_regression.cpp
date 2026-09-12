#ifdef NDEBUG
#undef NDEBUG
#endif
#define EIGEN_RUNTIME_NO_MALLOC
#include <traj_opt/spline_components.hpp>
#include <iostream>
#include <stdexcept>

namespace
{
using Spline = SplineTrajectory::SepticSplineND<3>;

void require(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}

struct Parameterization
{
    SplineTrajectory::QuadInvTimeMap time;
    SplineTrajectory::IdentitySpatialMap<3> space;
    const traj_opt_adapters::SuperBackupAuxiliaryStateMap &auxiliary;
};

struct IntegralCost
{
    static constexpr int kDerivativeOrder = 3;
    double operator()(const SplineTrajectory::IntegralPointInfo &,
                      const SplineTrajectory::SampleState<3> &state,
                      SplineTrajectory::SampleGradient<3> &gradient) const
    {
        gradient.p += 0.02 * state.p;
        gradient.v += 0.04 * state.v;
        gradient.a += 0.006 * state.a;
        gradient.j += 0.002 * state.j;
        return 0.01 * state.p.squaredNorm() + 0.02 * state.v.squaredNorm() +
               0.003 * state.a.squaredNorm() + 0.001 * state.j.squaredNorm();
    }
};

struct Objective
{
    traj_opt_components::LinearTimeCost duration{0.3};
    IntegralCost integral;
};
}

/** @brief Check moving-boundary and uniform-time pullbacks, layout and allocation reuse. */
int main()
{
    Spline::CoefficientMatrix reference_points(2, 3);
    reference_points << 0, 0, 0, 1.5, 0.3, 0.2;
    SplineTrajectory::BoundaryConditions<3> boundary;
    boundary.start_velocity << 0.5, 0.1, 0.0;
    boundary.end_velocity << 0.8, -0.1, 0.1;
    const Spline reference({2.0}, reference_points, 0.0, boundary);
    const auto trajectory = traj_opt_adapters::splineToSuperTrajectory(reference.polynomial());
    for (bool uniform : {false, true})
    {
        traj_opt_adapters::SuperBackupAuxiliaryStateMap auxiliary;
        auxiliary.reset(&trajectory, 0.2, 0.8, 0.15, uniform, 2, 0.4);
        SplineTrajectory::SplineOptimizer<Spline, Parameterization> optimizer{
            Parameterization{{}, {}, auxiliary}};
        SplineTrajectory::SplineProblem<Spline> problem;
        problem.durations = {1.2, 1.4};
        problem.waypoints.resize(3, 3);
        problem.waypoints << 0.3, 0.1, 0, 1, 0.3, 0.1, 2, 0.2, 0;
        SplineTrajectory::OptimizationMask mask;
        mask.time.assign(2, uniform ? 0 : 1);
        mask.waypoints = {0, 1, 1};
        problem.mask = mask;
        SplineTrajectory::OptimizerOptions options;
        options.energy_weight = 0.002;
        options.integration_steps = 8;
        require(bool(optimizer.prepare(problem, options)), "backup preparation failed");
        require(optimizer.dimension() == (uniform ? 0 : 2) + 6 + auxiliary.dimension(),
                "uniform time retained inactive duration coordinates");
        auto x = optimizer.initialGuess();
        x(optimizer.layout().auxiliary_offset + (uniform ? 1 : 0)) += 0.12;
        Objective objective;
        const auto check = SplineTrajectory::checkGradients(optimizer, x, objective, 1e-5, 2e-4);
        require(bool(check), "backup auxiliary gradient differs from finite differences");
        Eigen::VectorXd gradient(x.size());
        Eigen::internal::set_is_malloc_allowed(false);
        const auto evaluation = optimizer.evaluate(x, gradient, objective);
        Eigen::internal::set_is_malloc_allowed(true);
        require(bool(evaluation) && std::isfinite(optimizer.energy()), "backup evaluation failed");
        const auto state = optimizer.parameters();
        if (uniform) require(state.durations[0] == state.durations[1], "uniform durations diverged");
        std::cout << "uniform=" << uniform << " dimension=" << optimizer.dimension()
                  << " maximum_gradient_error=" << check.max_absolute_error << '\n';
    }
}
