#include "conductor/apm.hpp"

// ArduConductor::ArduConductor(int &argc, char **argv, const std::string &name, double rate, uint32_t options)
// 	: BaseConductor(argc, argv, name, rate, options)
// {
// 	initNode();
// }

ArduConductor::ArduConductor(rclcpp::Node::SharedPtr node)
	: BaseConductor(node), flag_takeoff(APMTakeoffState::kLand)
{
	initNode();
}

/// @brief 设置全局坐标原点
/// @param latitude 纬度
/// @param longitude 经度
void ArduConductor::sendGpOrigin(double latitude, double longitude)
{
	// 临时创建一个发布者对象
	auto set_gp_origin_pub_ = this->node_->create_publisher<geographic_msgs::msg::GeoPointStamped>(
		"mavros/global_position/set_gp_origin", 10);
	// 构建 GeoPointStamped 消息
	auto gp_origin = geographic_msgs::msg::GeoPointStamped();
	gp_origin.position.latitude = latitude;
	gp_origin.position.longitude = longitude;
	gp_origin.position.altitude = 0;
	gp_origin.header.stamp = this->now();

	set_gp_origin_pub_->publish(gp_origin);
	RCLCPP_INFO(this->get_logger(), SUCCESS("Setting gp origin...(%0.2f, %0.2f)"), longitude, latitude);
}

/// @brief 设置移动速度
/// @param speed
/// @return
bool ArduConductor::setMoveSpeed(double speed)
{
	auto speed_client_ = this->node_->create_client<mavros_msgs::srv::CommandLong>("mavros/cmd/command");

	// 创建请求
	auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
	request->command = mavros_msgs::msg::CommandCode::DO_CHANGE_SPEED;
	request->param1 = 0; // ignored by Ardupilot
	request->param2 = speed;
	// 异步发送请求
	auto result_future = speed_client_->async_send_request(request);
	// 等待服务响应
	if (rclcpp::spin_until_future_complete(this->node_->get_node_base_interface(), result_future) == rclcpp::FutureReturnCode::SUCCESS && result_future.get()->success)
	{
		RCLCPP_INFO(this->get_logger(), SUCCESS("Speed set: %0.2f"), speed);
		return true;
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "Failed to set speed");
		return false;
	}
}

/// @brief flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
/// @param x
/// @param y
/// @param z
/// @param yaw_rate
/// @return
void ArduConductor::setSpeedBody(double x, double y, double z, double yaw_rate)
{
	auto raw_target = mavros_msgs::msg::PositionTarget();
	raw_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_PX |
						   mavros_msgs::msg::PositionTarget::IGNORE_PY |
						   mavros_msgs::msg::PositionTarget::IGNORE_PZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFY |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFZ;
	if (fabs(yaw_rate) < 1e-3)
	{
		raw_target.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
		raw_target.yaw = 0;
	}
	else
	{
		raw_target.type_mask |= mavros_msgs::msg::PositionTarget::IGNORE_YAW;
	}

	raw_target.velocity.x = x;
	raw_target.velocity.y = y;
	raw_target.velocity.z = z;
	raw_target.yaw_rate = yaw_rate;
	set_raw_pub_->publish(raw_target);
}

/// @brief FLU rad/s , must set continously, or the vehicle stops after a few seconds.(failsafe feature) used for adjusting yaw without setting others.
/// @param yaw_rate
/// @return
void ArduConductor::setAngularRate(double yaw_rate)
{
    auto raw_target = mavros_msgs::msg::PositionTarget();
	raw_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
						   mavros_msgs::msg::PositionTarget::IGNORE_VY |
						   mavros_msgs::msg::PositionTarget::IGNORE_VZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFY |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_YAW; // yaw_rate must be used with pose or vel.
	raw_target.position.x = 0;
	raw_target.position.y = 0;
	raw_target.position.z = 0;
	raw_target.yaw_rate = yaw_rate;
	set_raw_pub_->publish(raw_target);
	RCLCPP_INFO(this->get_logger(), "Setting angular rate to: %0.2f rad/s", yaw_rate);
}

/// @brief flu meters rad. yaw = 0 when not used. x=y=z=0 when not used.
/// @param x
/// @param y
/// @param z
/// @param yaw
/// @return
void ArduConductor::setPoseBody(double x, double y, double z, double yaw)
{
    auto raw_target = mavros_msgs::msg::PositionTarget();
	raw_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_BODY_OFFSET_NED;
	raw_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
						   mavros_msgs::msg::PositionTarget::IGNORE_VY |
						   mavros_msgs::msg::PositionTarget::IGNORE_VZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFY |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
	raw_target.position.x = x;
	raw_target.position.y = y;
	raw_target.position.z = z;
	raw_target.yaw = yaw;
	set_raw_pub_->publish(raw_target);
}

/// @brief 悬停
void ArduConductor::setBreak()
{
	setSpeedBody(0.0, 0.0, 0.0, 0.0);
}

void ArduConductor::initNode()
{
}

void ArduConductor::setPoseRelated(double x, double y, double z, double yaw)
{
	(void)x; (void)y; (void)z; (void)yaw;
}

void ArduConductor::setPoseWorld(double x, double y, double z, double yaw) const
{
	sendTranslatedPoseWorld(x, y, z, yaw);
}

// void ArduConductor::setPoseWorld(waypoint::Waypoint waypoint)
// {
// 	setWaypointPoseWorld(waypoint);
// }

// void ArduConductor::setWaypointPoseWorld(waypoint::Waypoint waypoint)
// {
// 	this->setMoveSpeed(waypoint.air_speed);
// 	this->setPoseWorld(waypoint.position.x,
// 					   waypoint.position.y,
// 					   waypoint.position.z,
// 					   waypoint.yaw);
// }

void ArduConductor::sendTranslatedPoseWorld(double x, double y, double z, double yaw) const
{
	double gamma_world = -1.5707963;
	auto raw_target = mavros_msgs::msg::PositionTarget();
	raw_target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
	raw_target.type_mask = mavros_msgs::msg::PositionTarget::IGNORE_VX |
						   mavros_msgs::msg::PositionTarget::IGNORE_VY |
						   mavros_msgs::msg::PositionTarget::IGNORE_VZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFX |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFY |
						   mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
						   mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;
	raw_target.position.x = -y;
	raw_target.position.y = x;
	raw_target.position.z = z;
	raw_target.yaw = yaw - gamma_world;

	// 使用 ROS2 语法发布消息
	set_raw_pub_->publish(raw_target);
}

/// @brief 解锁电机
/// @param delay 距离上一个操作的延迟
/// @return bool
bool ArduConductor::arm(double delay)
{
	if (!current_state.armed &&
		isTimeElapsed(delay))
	{
		auto arming_client = this->node_->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
		auto arm_cmd = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
		arm_cmd->value = true;

		if (arming_client->wait_for_service(std::chrono::seconds(1)))
		{
			auto result = arming_client->async_send_request(arm_cmd);
			if (rclcpp::spin_until_future_complete(this->node_->get_node_base_interface(), result) ==
					rclcpp::FutureReturnCode::SUCCESS &&
				result.get()->success)
			{
				RCLCPP_INFO(this->get_logger(), "Vehicle armed!");
				this->mission_state = MissionState::kTakeoff;
				this->flag_takeoff = APMTakeoffState::kArmed;
				RCLCPP_INFO(this->get_logger(), MISSION_SWITCH_TO("takeoff"));
				updateLastRequestTime();
				return true;
			}
			else
			{
				RCLCPP_ERROR(this->get_logger(), "Vehicle arm failed!");
				this->mission_state = MissionState::kPrearm;
				this->flag_takeoff = APMTakeoffState::kLand;
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
bool ArduConductor::land(double delay)
{
	if (this->current_state.armed &&
		isTimeElapsed(delay))
	{
		// 创建降落服务客户端
		auto land_client = this->node_->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/land");

		// 等待服务可用
		if (!land_client->wait_for_service(std::chrono::seconds(1)))
		{
			RCLCPP_ERROR(this->get_logger(), "Landing service not available.");
			return false;
		}

		// 创建服务请求
		auto land_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

		// 异步调用服务并等待结果
		auto result = land_client->async_send_request(land_request);
		if (rclcpp::spin_until_future_complete(this->node_->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS &&
			result.get()->success)
		{
			RCLCPP_INFO(this->get_logger(), SUCCESS("Vehicle landed!"));
			this->flag_takeoff = APMTakeoffState::kLand;
			updateLastRequestTime();
			return true;
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Vehicle land failed!");
			updateLastRequestTime();
			return false;
		}
	}
	return false;
}

/// @brief 起飞到指定高度 ArduCopter
/// @param pre_takeoff_alt 一段起飞高度 单位 M
/// @param altitude 最终起飞高度 单位 M
/// @param delay 距离上一个操作的延迟
/// @return bool
bool ArduConductor::takeoff(double pre_takeoff_alt, double altitude, double delay)
{
	if (isTimeElapsed(delay) && flag_takeoff == APMTakeoffState::kArmed)
	{
		flag_takeoff = APMTakeoffState::kTakeoff1;
		return false;
	}

	if (isTimeElapsed(delay) && flag_takeoff == APMTakeoffState::kTakeoff1)
	{
		// 创建起飞服务客户端
		auto takeoff_client = this->node_->create_client<mavros_msgs::srv::CommandTOL>("mavros/cmd/takeoff");

		// 等待服务可用
		if (!takeoff_client->wait_for_service(std::chrono::seconds(1)))
		{
			RCLCPP_ERROR(this->get_logger(), "Takeoff service not available.");
			return false;
		}

		// 创建服务请求
		auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
		takeoff_request->altitude = pre_takeoff_alt;

		// 异步调用服务并等待结果
		auto result = takeoff_client->async_send_request(takeoff_request);
		if (rclcpp::spin_until_future_complete(this->node_->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS &&
			result.get()->success)
		{
			RCLCPP_INFO(this->get_logger(), SUCCESS("Vehicle pre-Takeoff to altitude: %0.2f"), pre_takeoff_alt);
			this->flag_takeoff = APMTakeoffState::kTakeoff2;
			return false;
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "Vehicle pre-Takeoff Failed!");
			this->mission_state = MissionState::kArm;
			RCLCPP_INFO(this->get_logger(), MISSION_SWITCH_TO("arm"));
			updateLastRequestTime();
			return false;
		}
	}

	if (isTimeElapsed(delay + 2.0) && flag_takeoff == APMTakeoffState::kTakeoff2)
	{
		this->setMoveSpeed(0.2); // 设置空速
		this->setPoseWorld(0, 0, altitude, 0);

		RCLCPP_INFO(this->get_logger(), SUCCESS("Vehicle Takeoff to world_frame altitude: %0.2f"), altitude);
		updateLastRequestTime();
		this->flag_takeoff = APMTakeoffState::kArmed;
		return true;
	}

	return false;
}