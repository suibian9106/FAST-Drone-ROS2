#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <quadrotor_msgs/msg/px4ctrl_debug.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <memory>

class PX4CtrlTester : public rclcpp::Node
{
public:
    PX4CtrlTester() : Node("px4ctrl_tester")
    {
        // 初始化发布器
        state_pub_ = this->create_publisher<mavros_msgs::msg::State>("/mavros/state", 10);
        extended_state_pub_ = this->create_publisher<mavros_msgs::msg::ExtendedState>("/mavros/extended_state", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/mavros/imu/data", 10);
        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/mavros/battery", 10);
        rc_pub_ = this->create_publisher<mavros_msgs::msg::RCIn>("/mavros/rc/in", 10);
        cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>("cmd", 10);
        takeoff_land_pub_ = this->create_publisher<quadrotor_msgs::msg::TakeoffLand>("takeoff_land", 10);

        // 初始化订阅器 - 监控px4ctrl的输出
        ctrl_sub_ = this->create_subscription<mavros_msgs::msg::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10,
            std::bind(&PX4CtrlTester::ctrl_callback, this, std::placeholders::_1));
        
        debug_sub_ = this->create_subscription<quadrotor_msgs::msg::Px4ctrlDebug>(
            "/debugPx4ctrl", 10,
            std::bind(&PX4CtrlTester::debug_callback, this, std::placeholders::_1));

        trigger_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/traj_start_trigger", 10,
            std::bind(&PX4CtrlTester::trigger_callback, this, std::placeholders::_1));

        // 初始化服务 - 模拟PX4对服务的响应
        set_mode_service_ = this->create_service<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode",
            std::bind(&PX4CtrlTester::set_mode_callback, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        arming_service_ = this->create_service<mavros_msgs::srv::CommandBool>(
            "/mavros/cmd/arming",
            std::bind(&PX4CtrlTester::arming_callback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        command_service_ = this->create_service<mavros_msgs::srv::CommandLong>(
            "/mavros/cmd/command",
            std::bind(&PX4CtrlTester::command_callback, this,
                     std::placeholders::_1, std::placeholders::_2));

        // 初始化定时器
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50Hz
            std::bind(&PX4CtrlTester::publish_sensor_data, this));
        
        test_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&PX4CtrlTester::run_test_sequence, this));

        // 初始化测试状态
        test_phase_ = 0;
        test_start_time_ = this->now();
        current_pose_.setZero();
        current_vel_.setZero();
        current_orientation_.setIdentity();

        RCLCPP_INFO(this->get_logger(), "PX4Ctrl测试节点已启动");
    }

private:
    // 发布器
    rclcpp::Publisher<mavros_msgs::msg::State>::SharedPtr state_pub_;
    rclcpp::Publisher<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<mavros_msgs::msg::RCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<quadrotor_msgs::msg::TakeoffLand>::SharedPtr takeoff_land_pub_;

    // 订阅器
    rclcpp::Subscription<mavros_msgs::msg::AttitudeTarget>::SharedPtr ctrl_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::Px4ctrlDebug>::SharedPtr debug_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;

    // 服务
    rclcpp::Service<mavros_msgs::srv::SetMode>::SharedPtr set_mode_service_;
    rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_service_;
    rclcpp::Service<mavros_msgs::srv::CommandLong>::SharedPtr command_service_;

    // 定时器
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr test_timer_;

    // 测试状态
    int test_phase_;
    rclcpp::Time test_start_time_;
    Eigen::Vector3d current_pose_;
    Eigen::Vector3d current_vel_;
    Eigen::Quaterniond current_orientation_;
    bool is_armed_ = false;
    std::string current_mode_ = "MANUAL";

    void publish_sensor_data()
    {
        auto now = this->now();

        // 发布状态信息
        auto state_msg = std::make_unique<mavros_msgs::msg::State>();
        state_msg->header.stamp = now;
        state_msg->connected = true;
        state_msg->armed = is_armed_;
        state_msg->guided = true;
        state_msg->mode = current_mode_;
        state_pub_->publish(std::move(state_msg));

        // 发布扩展状态
        auto extended_state_msg = std::make_unique<mavros_msgs::msg::ExtendedState>();
        extended_state_msg->header.stamp = now;
        extended_state_msg->landed_state = (current_pose_.z() < 0.1) ? 
            mavros_msgs::msg::ExtendedState::LANDED_STATE_ON_GROUND : 
            mavros_msgs::msg::ExtendedState::LANDED_STATE_IN_AIR;
        extended_state_pub_->publish(std::move(extended_state_msg));

        // 发布里程计
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = now;
        odom_msg->header.frame_id = "world";
        odom_msg->child_frame_id = "base_link";
        
        odom_msg->pose.pose.position.x = current_pose_.x();
        odom_msg->pose.pose.position.y = current_pose_.y();
        odom_msg->pose.pose.position.z = current_pose_.z();
        
        odom_msg->pose.pose.orientation.x = current_orientation_.x();
        odom_msg->pose.pose.orientation.y = current_orientation_.y();
        odom_msg->pose.pose.orientation.z = current_orientation_.z();
        odom_msg->pose.pose.orientation.w = current_orientation_.w();
        
        odom_msg->twist.twist.linear.x = current_vel_.x();
        odom_msg->twist.twist.linear.y = current_vel_.y();
        odom_msg->twist.twist.linear.z = current_vel_.z();
        
        odom_pub_->publish(std::move(odom_msg));

        // 发布IMU数据
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
        imu_msg->header.stamp = now;
        imu_msg->header.frame_id = "base_link";
        
        imu_msg->orientation.x = current_orientation_.x();
        imu_msg->orientation.y = current_orientation_.y();
        imu_msg->orientation.z = current_orientation_.z();
        imu_msg->orientation.w = current_orientation_.w();
        
        // 添加一些小的随机噪声模拟真实IMU
        imu_msg->angular_velocity.x = 0.01 * (rand() % 100 - 50) / 50.0;
        imu_msg->angular_velocity.y = 0.01 * (rand() % 100 - 50) / 50.0;
        imu_msg->angular_velocity.z = 0.01 * (rand() % 100 - 50) / 50.0;
        
        imu_msg->linear_acceleration.x = 0.02 * (rand() % 100 - 50) / 50.0;
        imu_msg->linear_acceleration.y = 0.02 * (rand() % 100 - 50) / 50.0;
        imu_msg->linear_acceleration.z = 9.81 + 0.02 * (rand() % 100 - 50) / 50.0;
        
        imu_pub_->publish(std::move(imu_msg));

        // 发布电池状态
        auto battery_msg = std::make_unique<sensor_msgs::msg::BatteryState>();
        battery_msg->header.stamp = now;
        battery_msg->voltage = 16.0;
        battery_msg->percentage = 0.8;
        battery_msg->cell_voltage = {4.0, 4.0, 4.0, 4.0};
        battery_pub_->publish(std::move(battery_msg));
    }

    void run_test_sequence()
    {
        auto elapsed = (this->now() - test_start_time_).seconds();
        
        switch (test_phase_) {
            case 0: // 初始状态 - 等待系统稳定
                if (elapsed > 2.0) {
                    RCLCPP_INFO(this->get_logger(), "测试阶段 1: 发送RC信号进入悬停模式");
                    send_rc_hover_mode();
                    test_phase_ = 1;
                    test_start_time_ = this->now();
                }
                break;
                
            case 1: // 发送悬停指令
                if (elapsed > 3.0) {
                    RCLCPP_INFO(this->get_logger(), "测试阶段 2: 发送位置控制指令");
                    send_position_command();
                    test_phase_ = 2;
                    test_start_time_ = this->now();
                }
                break;
                
            case 2: // 发送控制指令
                if (elapsed > 5.0) {
                    RCLCPP_INFO(this->get_logger(), "测试阶段 3: 发送起飞指令");
                    send_takeoff_command();
                    test_phase_ = 3;
                    test_start_time_ = this->now();
                }
                break;
                
            case 3: // 测试自动起飞
                if (elapsed > 8.0) {
                    RCLCPP_INFO(this->get_logger(), "测试阶段 4: 发送降落指令");
                    send_land_command();
                    test_phase_ = 4;
                    test_start_time_ = this->now();
                }
                break;
                
            case 4: // 测试自动降落
                if (elapsed > 5.0) {
                    RCLCPP_INFO(this->get_logger(), "测试完成");
                    test_phase_ = 5;
                }
                break;
        }
    }

    void send_rc_hover_mode()
    {
        auto rc_msg = std::make_unique<mavros_msgs::msg::RCIn>();
        rc_msg->header.stamp = this->now();
        
        // 设置通道值模拟进入悬停模式
        // 通道4: 模式切换 (1500 = 悬停模式)
        // 通道5: 齿轮控制 (2000 = 命令模式)
        rc_msg->channels = {1500, 1500, 1500, 1500, 1500, 2000, 1000, 1000};
        rc_pub_->publish(std::move(rc_msg));
    }

    void send_position_command()
    {
        auto cmd_msg = std::make_unique<quadrotor_msgs::msg::PositionCommand>();
        cmd_msg->header.stamp = this->now();
        cmd_msg->header.frame_id = "world";
        
        // 设置目标位置 (前方2米，高度3米)
        cmd_msg->position.x = 2.0;
        cmd_msg->position.y = 0.0;
        cmd_msg->position.z = 3.0;
        
        // 设置目标速度
        cmd_msg->velocity.x = 0.5;
        cmd_msg->velocity.y = 0.0;
        cmd_msg->velocity.z = 0.0;
        
        // 设置目标加速度
        cmd_msg->acceleration.x = 0.0;
        cmd_msg->acceleration.y = 0.0;
        cmd_msg->acceleration.z = 0.0;
        
        // 设置偏航角
        cmd_msg->yaw = 0.0;
        cmd_msg->yaw_dot = 0.0;
        
        cmd_pub_->publish(std::move(cmd_msg));
    }

    void send_takeoff_command()
    {
        auto takeoff_msg = std::make_unique<quadrotor_msgs::msg::TakeoffLand>();
        takeoff_msg->takeoff_land_cmd = quadrotor_msgs::msg::TakeoffLand::TAKEOFF;
        takeoff_land_pub_->publish(std::move(takeoff_msg));
    }

    void send_land_command()
    {
        auto land_msg = std::make_unique<quadrotor_msgs::msg::TakeoffLand>();
        land_msg->takeoff_land_cmd = quadrotor_msgs::msg::TakeoffLand::LAND;
        takeoff_land_pub_->publish(std::move(land_msg));
    }

    void ctrl_callback(const mavros_msgs::msg::AttitudeTarget::SharedPtr msg)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                           "收到控制指令: thrust=%.3f, q=[%.3f, %.3f, %.3f, %.3f]", 
                           msg->thrust, 
                           msg->orientation.x, msg->orientation.y, 
                           msg->orientation.z, msg->orientation.w);
    }

    void debug_callback(const quadrotor_msgs::msg::Px4ctrlDebug::SharedPtr msg)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "调试信息: des_v=[%.2f, %.2f, %.2f], des_a=[%.2f, %.2f, %.2f]",
                           msg->des_v_x, msg->des_v_y, msg->des_v_z,
                           msg->des_a_x, msg->des_a_y, msg->des_a_z);
    }

    void trigger_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到轨迹触发信号");
    }

    void set_mode_callback(const std::shared_ptr<mavros_msgs::srv::SetMode::Request> request,
                          std::shared_ptr<mavros_msgs::srv::SetMode::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到设置模式请求: %s", request->custom_mode.c_str());
        current_mode_ = request->custom_mode;
        response->mode_sent = true;
    }

    void arming_callback(const std::shared_ptr<mavros_msgs::srv::CommandBool::Request> request,
                        std::shared_ptr<mavros_msgs::srv::CommandBool::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到上锁/解锁请求: %s", request->value ? "ARM" : "DISARM");
        is_armed_ = request->value;
        response->success = true;
    }

    void command_callback(const std::shared_ptr<mavros_msgs::srv::CommandLong::Request> request,
                         std::shared_ptr<mavros_msgs::srv::CommandLong::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "收到通用命令: cmd=%d", request->command);
        response->success = true;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4CtrlTester>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}