#ifndef HUMAN_CONTROL_INTERFACE_H
#define HUMAN_CONTROL_INTERFACE_H

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>

namespace ego_planner {

class HumanControlInterface {
public:
    HumanControlInterface(std::shared_ptr<rclcpp::Node> node);
    ~HumanControlInterface() = default;

    /**
     * @brief 发送目标位置以控制无人机
     * @param  position         目标位置: Eigen::Vector3d(0.0, 0.0, 0.5)
     */
    void sendTargetPosition(const Eigen::Vector3d &position);
    /**
     * @brief 发送目标角度偏移量（相对于起始姿态）
     * @param  yaw              目标偏移角度（单位：度）
     */
    void sendTargetAngleOffsetStart(double yaw);
    /**
     * @brief 发送目标角度偏移量（相对于最后姿态）
     * @param  yaw              目标偏移角度（单位：度）
     */
    void sendTargetAngleOffsetLast(double yaw);

    /**
     * @brief Report Methods
     */

    /**
     * @brief 获取当前位置距离起飞点的偏移量
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d reportPositionOffset();
    /**
     * @brief 获取当前角度相对于起始姿态的偏移量
     * @detail 这里只计算了yaw角度偏移
     * @return double
     */
    double reportAngleOffsetStart();
    /**
     * @brief 获取当前角度
     * @detail 这里只计算了yaw角度
     * @return double
     */
    double reportCurrentAngle();

    /**
     * @brief Getter Methods
     */
    Eigen::Vector3d getStartPosition() const { return start_position_; }
    Eigen::Vector3d getCurrentPosition() const { return current_position_; }
    Eigen::Vector3d getStartAngles() const { return start_angles_; }
    Eigen::Vector3d getLastAngles() const { return last_angles_; }
    bool isStartPositionSet() const { return start_position_set_; }

private:
    /**
     * @brief methods
     *
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<rclcpp::Node> node_;
    
    Eigen::Vector3d start_position_;    // 起飞点位置
    Eigen::Vector3d current_position_;  // 当前位置
    bool start_position_set_;           // 是否已设置起飞点

    Eigen::Vector3d start_angles_;
    Eigen::Vector3d last_angles_;
};

}  // namespace ego_planner

#endif  // HUMAN_CONTROL_INTERFACE_H