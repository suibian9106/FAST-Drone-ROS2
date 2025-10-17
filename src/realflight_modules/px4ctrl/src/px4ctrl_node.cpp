#include <rclcpp/rclcpp.hpp>
#include <signal.h>

#include "PX4CtrlFSM.h"

class PX4CtrlNode : public rclcpp::Node {
public:
    PX4CtrlNode() : Node("px4ctrl") {
        // Setup parameter
        param.config_from_ros_handle(*this);

        // Initialize controller and FSM
        controller = std::make_unique<LinearControl>(param);
        fsm = std::make_unique<PX4CtrlFSM>(param, *controller);

        setup_subscriptions();
        setup_publishers();
        setup_services();

        // Wait for 1 second
        rclcpp::sleep_for(std::chrono::seconds(1));

        if (!param.takeoff_land.no_RC) {
            RCLCPP_WARN(this->get_logger(), "[PX4CTRL] Remote controller disabled, be careful!");
        } else {
            RCLCPP_INFO(this->get_logger(), "[PX4CTRL] Waiting for RC");
            while (rclcpp::ok()) {
                rclcpp::spin_some(get_node_base_interface());
                if (fsm->rc_is_received(this->now())) {
                    RCLCPP_INFO(this->get_logger(), "[PX4CTRL] RC received.");
                    break;
                }
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }

        int trials = 0;
        while (rclcpp::ok() && !fsm->state_data.current_state.connected) {
            rclcpp::spin_some(get_node_base_interface());
            rclcpp::sleep_for(std::chrono::seconds(1));
            if (trials++ > 5) RCLCPP_ERROR(this->get_logger(), "Unable to connnect to PX4!!!");
        }

        // Create timer for main control loop
        double ctrl_freq = param.ctrl_freq_max;
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / ctrl_freq),
            std::bind(&PX4CtrlNode::control_loop, this));
    }

private:
    void setup_subscriptions() {
        // State subscription
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                fsm->state_data.feed(msg);
            });

        // Extended state subscription
        extended_state_sub_ = this->create_subscription<mavros_msgs::msg::ExtendedState>(
            "/mavros/extended_state", 10,
            [this](const mavros_msgs::msg::ExtendedState::SharedPtr msg) {
                fsm->extended_state_data.feed(msg);
            });

        // Odometry subscription
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::QoS(100),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                fsm->odom_data.feed(msg);
            });

        // Command subscription
        cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "cmd", rclcpp::QoS(100),
            [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
                fsm->cmd_data.feed(msg);
            });

        // IMU subscription
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data",  // Note: do NOT change it to /mavros/imu/data_raw
            rclcpp::QoS(100),
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                fsm->imu_data.feed(msg);
            });

        // RC subscription (if enabled)
        if (!param.takeoff_land.no_RC) {
            rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>(
                "/mavros/rc/in", 10,
                [this](const mavros_msgs::msg::RCIn::SharedPtr msg) {
                    fsm->rc_data.feed(msg);
                });
        }

        // Battery subscription
        bat_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", rclcpp::QoS(100),
            [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
                fsm->bat_data.feed(msg);
            });

        // Takeoff/Land subscription
        takeoff_land_sub_ = this->create_subscription<quadrotor_msgs::msg::TakeoffLand>(
            "takeoff_land", rclcpp::QoS(100),
            [this](const quadrotor_msgs::msg::TakeoffLand::SharedPtr msg) {
                fsm->takeoff_land_data.feed(msg);
            });
    }

    void setup_publishers() {
        fsm->ctrl_FCU_pub = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(
            "/mavros/setpoint_raw/attitude", 10);
        fsm->traj_start_trigger_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/traj_start_trigger", 10);
        fsm->debug_pub = this->create_publisher<quadrotor_msgs::msg::Px4ctrlDebug>(
            "/debugPx4ctrl", 10);
    }

    void setup_services() {
        fsm->set_FCU_mode_srv = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        fsm->arming_client_srv = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        fsm->reboot_FCU_srv = this->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    }

    void control_loop() {
        fsm->process();
    }

    Parameter_t param;
    std::unique_ptr<LinearControl> controller;
    std::unique_ptr<PX4CtrlFSM> fsm;

    rclcpp::TimerBase::SharedPtr timer_;

    // Subscribers
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bat_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::TakeoffLand>::SharedPtr takeoff_land_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4CtrlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}