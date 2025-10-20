#include <ndsl_control/human_control_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace ego_planner {

HumanControlInterface::HumanControlInterface(std::shared_ptr<rclcpp::Node> node)
    : node_(node), start_position_set_(false) {
    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/vins_fusion/imu_propagate", 10,
        std::bind(&HumanControlInterface::odomCallback, this, std::placeholders::_1));
}

void HumanControlInterface::sendTargetAngleOffsetStart(double yaw) {
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = node_->now();
    goal_msg.header.frame_id = "world";
    // 只设置方向，位置保持不变
    goal_msg.pose.position.x = current_position_.x();
    goal_msg.pose.position.y = current_position_.y();
    goal_msg.pose.position.z = current_position_.z();
    // from deg to rad
    auto newYaw = start_angles_.z() + yaw * M_PI / 180.0;
    // 设置方向
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(newYaw, Eigen::Vector3d::UnitZ());
    goal_msg.pose.orientation.w = q.w();
    goal_msg.pose.orientation.x = q.x();
    goal_msg.pose.orientation.y = q.y();
    goal_msg.pose.orientation.z = q.z();

    goal_pub_->publish(goal_msg);
    RCLCPP_INFO(node_->get_logger(), "发送目标角度: yaw=%f", newYaw * 180 / M_PI);
}

void HumanControlInterface::sendTargetAngleOffsetLast(double yaw) {
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = node_->now();
    goal_msg.header.frame_id = "world";
    // 只设置方向，位置保持不变
    goal_msg.pose.position.x = current_position_.x();
    goal_msg.pose.position.y = current_position_.y();
    goal_msg.pose.position.z = current_position_.z();
    auto newYaw = last_angles_.z() + yaw * M_PI / 180.0;
    // 设置方向
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(newYaw, Eigen::Vector3d::UnitZ());
    goal_msg.pose.orientation.w = q.w();
    goal_msg.pose.orientation.x = q.x();
    goal_msg.pose.orientation.y = q.y();
    goal_msg.pose.orientation.z = q.z();

    goal_pub_->publish(goal_msg);
    RCLCPP_INFO(node_->get_logger(), "发送目标角度: yaw=%f", newYaw * 180 / M_PI);
}

void HumanControlInterface::sendTargetPosition(
    const Eigen::Vector3d &position) {
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = node_->now();
    goal_msg.header.frame_id = "world";
    goal_msg.pose.position.x = position.x();
    goal_msg.pose.position.y = position.y();
    goal_msg.pose.position.z = position.z();
    // 不设置方向，让 ego-planner 来做更多优化
    // goal_msg.pose.orientation.w = 1.0; // 默认方向（无旋转）

    goal_pub_->publish(goal_msg);
    RCLCPP_INFO(node_->get_logger(), "发送目标位置: [%f, %f, %f]", position.x(), position.y(),
             position.z());
}

void HumanControlInterface::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    // 设置起飞点（仅第一次收到消息时）
    if (!start_position_set_) {
        start_position_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                          msg->pose.pose.position.y,
                                          msg->pose.pose.position.z);
        Eigen::Quaterniond q_eigen(
            msg->pose.pose.orientation.w,  // 注意 Eigen::Quaterniond 的构造函数是 (w, x, y, z)
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z);
        start_angles_ = q_eigen.toRotationMatrix().eulerAngles(2, 1, 0);
        start_position_set_ = true;
        RCLCPP_INFO(node_->get_logger(), "Start position set: [%f, %f, %f]", start_position_.x(),
                 start_position_.y(), start_position_.z());
        RCLCPP_INFO(node_->get_logger(), "Start angles set: [%f, %f, %f]",
                 start_angles_.x() * 180 / M_PI, start_angles_.y() * 180 / M_PI,
                 start_angles_.z() * 180 / M_PI);
    }
    current_position_ =
        Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    Eigen::Quaterniond q_eigen(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    last_angles_ = q_eigen.toRotationMatrix().eulerAngles(2, 1, 0);
}

Eigen::Vector3d HumanControlInterface::reportPositionOffset() {
    if (!start_position_set_) {
        RCLCPP_WARN(node_->get_logger(), "Start position not set yet, cannot report offset.");
        return Eigen::Vector3d::Zero();  // 返回零向量表示无效
    }
    Eigen::Vector3d offset = current_position_ - start_position_;
    return offset;
}

double HumanControlInterface::reportAngleOffsetStart() {
    if (!start_position_set_) {
        RCLCPP_WARN(node_->get_logger(), "Start position not set yet, cannot report angle offset.");
        return -1;  // 返回零向量表示无效
    }
    return (last_angles_.z() - start_angles_.z()) * 180.0 /
           M_PI;  // 返回偏航角度差
}

double HumanControlInterface::reportCurrentAngle() {
    if (!start_position_set_) {
        RCLCPP_WARN(node_->get_logger(), "Start position not set yet, cannot report angle offset.");
        return -1;  // 返回零向量表示无效
    }
    return last_angles_.z() * 180.0 / M_PI;  // 返回偏航角度差
}

}  // namespace ego_planner