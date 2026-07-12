#include <traj_opt/exp_traj_optimizer_s4.h>
#include <ros_interface/ros1/fsm_ros1.hpp>

#include <cstdlib>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

namespace
{
geometry_utils::Polytope makeBox(double xmin, double xmax)
{
    super_utils::MatD4f planes(6, 4);
    planes << 1.0, 0.0, 0.0, -xmax,
             -1.0, 0.0, 0.0, xmin,
              0.0, 1.0, 0.0, -2.0,
              0.0,-1.0, 0.0, -2.0,
              0.0, 0.0, 1.0, -2.0,
              0.0, 0.0,-1.0, -2.0;
    return geometry_utils::Polytope(planes);
}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "backend_equivalence");
    ros::NodeHandle node("~");
    auto ros_interface = std::make_shared<ros_interface::Ros1Interface>(node);

    const std::string config_path = std::string(ROOT_DIR) + "config/static_high_speed.yaml";
    traj_opt::Config config(config_path, "exp_traj");
    config.print_optimizer_log = true;
    if (argc > 2) config.opt_accuracy = std::stod(argv[2]);
    if (argc > 3 && std::string(argv[3]) == "energy-only") {
        config.penna_pos = config.penna_vel = config.penna_acc = 0.0;
        config.penna_jerk = config.penna_attract = 0.0;
        config.penna_omg = config.penna_thr = 0.0;
    }
    auto optimizer = std::make_shared<traj_opt::ExpTrajOpt>(config, ros_interface);

    const double perturbation = argc > 1 ? std::stod(argv[1]) : 0.0;
    super_utils::StatePVAJ head = super_utils::StatePVAJ::Zero();
    super_utils::StatePVAJ tail = super_utils::StatePVAJ::Zero();
    head.col(0) << 0.0, 0.0, 0.0;
    head.col(1) << 0.05 * perturbation, -0.02 * perturbation, 0.01 * perturbation;
    tail.col(0) << 6.0, 0.4 + 0.1 * perturbation, 0.2 - 0.05 * perturbation;

    geometry_utils::PolytopeVec corridor;
    corridor.emplace_back(makeBox(-1.0, 3.0));
    corridor.emplace_back(makeBox(2.0, 5.0));
    corridor.emplace_back(makeBox(4.0, 7.0));

    super_utils::vec_Vec3f initial_points;
    initial_points.emplace_back(2.5, 0.1 + 0.05 * perturbation, 0.02 * perturbation);
    initial_points.emplace_back(4.5, 0.3 - 0.04 * perturbation, 0.1);
    super_utils::VecDf initial_times(3);
    initial_times << 1.1 + 0.03 * perturbation,
                     0.9 - 0.02 * perturbation,
                     1.2 + 0.04 * perturbation;

    geometry_utils::Trajectory trajectory;
    const auto optimize_begin = std::chrono::steady_clock::now();
    const bool ok = optimizer->optimize(head, tail, corridor,
                                        initial_points, initial_times,
                                        trajectory);
    const auto optimize_end = std::chrono::steady_clock::now();
    const double optimize_us =
        std::chrono::duration<double, std::micro>(optimize_end - optimize_begin).count();
    std::cout << "[EQ-TIME] optimize_us=" << std::setprecision(17) << optimize_us << '\n';
    std::cout << "[EQ-RESULT] ok=" << ok
              << " pieces=" << trajectory.getPieceNum()
              << " duration=" << trajectory.getTotalDuration() << '\n';
    if (!ok || trajectory.getPieceNum() == 0) {
        return 1;
    }
    std::cout << std::setprecision(17);
    for (double fraction : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        const double t = fraction * trajectory.getTotalDuration();
        std::cout << "[EQ-SAMPLE] fraction=" << fraction
                  << " pos=" << trajectory.getPos(t).transpose()
                  << " vel=" << trajectory.getVel(t).transpose() << '\n';
    }
    return ok ? 0 : 1;
}
