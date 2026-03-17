#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "waypoint_mission/ros2_waypoint_planner.hpp"

#ifndef BACKWARD_HAS_DW
#define BACKWARD_HAS_DW 1
#endif
#include "utils/backward.hpp"
namespace backward{
    backward::SignalHandling sh;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("waypoint_mission");

    /* Publisher and subscriber */
    mission_planner::WaypointPlanner wpl(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
