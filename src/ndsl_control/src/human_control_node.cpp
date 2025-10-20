#include <ndsl_control/human_control_interface.h>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <cmath>
#include <vector>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("human_control_node");

    ego_planner::HumanControlInterface control_interface(node);

    // 等待起始位置设置完成
    RCLCPP_INFO(node->get_logger(), "Waiting for odometry data to set start position...");
    while (rclcpp::ok() && !control_interface.isStartPositionSet()) {
        rclcpp::spin_some(node);
        rclcpp::Rate(2).sleep();  // 0.5Hz = 2秒周期
    }
    RCLCPP_INFO(node->get_logger(), "Start position set, proceeding with waypoints.");

    // 定义矩形路径（6米 × 4米，高度1米）
    std::vector<Eigen::Vector3d> waypoints = {
        Eigen::Vector3d(0.0, 0.0, 0.5),  // 起点
        Eigen::Vector3d(1.5, 0.0, 0.5),  // 向右移动6米
        Eigen::Vector3d(1.5, 1.0, 0.5),  // 向上移动4米
        Eigen::Vector3d(0.0, 1.0, 0.5),  // 向左移动6米
        Eigen::Vector3d(0.0, 0.0, 0.5)   // 回到起点
    };

    rclcpp::Rate rate(10);  // 10Hz 汇报频率
    // Define a threshold for considering a waypoint "reached" (e.g., 0.1
    // meters)
    const double waypoint_reach_threshold = 0.2;  // meters

    for (const auto &wp : waypoints) {
        RCLCPP_INFO(node->get_logger(), "Targeting waypoint: [%f, %f, %f]", wp.x(), wp.y(), wp.z());
        control_interface.sendTargetPosition(wp);  // Send the current waypoint

        // Wait until the drone reaches the current waypoint
        // Add a timeout to prevent infinite loops if drone gets stuck
        auto waypoint_start_time = node->now();
        const double max_wait_time = 10.0;  // seconds, adjust as needed

        while (rclcpp::ok()) {
            // Calculate distance to target waypoint
            Eigen::Vector3d error = wp - control_interface.getCurrentPosition();
            double distance_to_waypoint =
                error.norm();  // L2 norm (Euclidean distance)

            if (distance_to_waypoint < waypoint_reach_threshold) {
                RCLCPP_INFO(node->get_logger(), "Reached waypoint: [%f, %f, %f]", wp.x(), wp.y(),
                         wp.z());
                break;  // Exit the while loop, move to the next waypoint
            }

            // Optional: Check for timeout if drone is taking too long
            if ((node->now() - waypoint_start_time).seconds() >
                max_wait_time) {
                RCLCPP_WARN(node->get_logger(),
                    "Timed out waiting for waypoint [%f, %f, %f]. Moving to "
                    "next.",
                    wp.x(), wp.y(), wp.z());
                break;  // Move to the next waypoint even if not reached
            }

            // Process ROS callbacks and sleep
            rclcpp::spin_some(node);
            rate.sleep();
        }
    }
    RCLCPP_INFO(node->get_logger(), "All waypoints visited!");

    return 0;
}