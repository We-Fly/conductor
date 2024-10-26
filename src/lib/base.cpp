#include "conductor/base.hpp"
#include "conductor/ansi_color.hpp"

/// @brief
/// @param rate_num loop循环频率
BaseConductor::BaseConductor(std::string node_name, double rate_num)
    : Node(node_name), rate(rate_num)
{
    mission_state = MissionState::kPrearm;
    this->initNode();
}

/// @brief 初始化各种订阅和发布服务
void BaseConductor::initNode()
{
    setlocale(LC_ALL, "");
    RCLCPP_INFO(this->get_logger(), COLORED_TEXT("Initializing node...", "\033[1m" ANSI_COLOR_MAGENTA));
    RCLCPP_INFO(this->get_logger(), COLORED_TEXT("高性能ですから!", "\033[2m" ANSI_COLOR_WHITE));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10,
        std::bind(&BaseConductor::subStateCallback, this, std::placeholders::_1));

    local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", 10);

    set_raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
        "mavros/setpoint_raw/local", 10);
}

/// @brief mavros/state 订阅回调函数
/// @param msg
void BaseConductor::subStateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
    this->current_state = *msg;
}

/// @brief 设置飞行模式为 guided
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::setModeGuided(double delay)
{
    auto _set_mode_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    auto mode_guided_msg = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    mode_guided_msg->custom_mode = "GUIDED";

    if (isTimeElapsed(delay))
    {
        while (!(_set_mode_client->wait_for_service(std::chrono::seconds(1))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service set_mode...");
        }

        auto result = _set_mode_client->async_send_request(mode_guided_msg);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            if (result.get()->mode_sent)
            {
                RCLCPP_INFO(this->get_logger(), "Guided enabled!");
                this->mission_state = MissionState::kArm;
                updateLastRequestTime();
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Guided Switch Failed!");
                this->mission_state = MissionState::kPrearm;
                updateLastRequestTime();
                return false;
            }
        }
    }
    return false;
}

/// @brief 解锁电机
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::arm(double delay)
{
    if (!current_state.armed &&
        isTimeElapsed(delay))
    {
        auto arming_client = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
        auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        arm_cmd->value = true;

        if (arming_client->wait_for_service(std::chrono::seconds(1)))
        {
            auto result = arming_client->async_send_request(arm_cmd);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                    rclcpp::FutureReturnCode::SUCCESS &&
                result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed!");
                this->mission_state = MissionState::kTakeoff;
                RCLCPP_INFO(this->get_logger(), MISSION_SWITCH_TO("takeoff"));
                updateLastRequestTime();
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Vehicle arm failed!");
                this->mission_state = MissionState::kPrearm;
                updateLastRequestTime();
                return false;
            }
        }
    }
    return false;
}

/// @brief 起飞到指定高度 ArduCopter
/// @param altitude 高度 单位 M
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::takeoff(double altitude, double delay)
{
    if (isTimeElapsed(delay))
    {
        auto takeoff_client = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");
        auto takeoff_cmd = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        takeoff_cmd->altitude = altitude;

        if (takeoff_client->wait_for_service(std::chrono::seconds(1)))
        {
            auto result = takeoff_client->async_send_request(takeoff_cmd);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS && result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), SUCCESS("Vehicle Takeoff to altitude: %0.2f"), altitude);
                updateLastRequestTime();
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Vehicle Takeoff Failed!");
                this->mission_state = MissionState::kArm;
                updateLastRequestTime();
                return false;
            }
        }
    }
    return false;
}

/// @brief 切换到landed模式（ArduCopter）降落飞机
/// @param delay 距离上一个操作的延迟
/// @return bool
bool BaseConductor::land(double delay)
{
    if (this->current_state.armed &&
        isTimeElapsed(delay))
    {
        auto land_client = this->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");
        auto land_cmd = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

        if (land_client->wait_for_service(std::chrono::seconds(1)))
        {
            auto result = land_client->async_send_request(land_cmd);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::FutureReturnCode::SUCCESS && result.get()->success)
            {
                RCLCPP_INFO(this->get_logger(), SUCCESS("Vehicle landed!"));
                updateLastRequestTime();
                return true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Vehicle land failed!");
                updateLastRequestTime();
                return false;
            }
        }    }
    return false;
}

/// @brief 判断是否到延迟时间
/// @param delay 延迟时间
/// @return bool
bool BaseConductor::isTimeElapsed(double delay)
{
    return this->now() - this->last_request > rclcpp::Duration::from_seconds(delay);
}

/// @brief 更新 last_request 的值为当前的 ROS 时间
void BaseConductor::updateLastRequestTime()
{
    last_request = this->now();
}