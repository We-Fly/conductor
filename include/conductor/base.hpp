#ifndef BASE_HPP
#define BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include "conductor/mission_state.hpp"
#include <rclcpp/time.hpp>

class BaseConductor : public rclcpp::Node
{
protected:
    virtual void initNode();
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr set_raw_pub_;

    void subStateCallback(const mavros_msgs::msg::State::SharedPtr msg);

public:
    BaseConductor(std::string node_name="base_conductor", double rate = 20.0);

    MissionState mission_state;

    rclcpp::Time last_request;
    rclcpp::Rate rate;
    mavros_msgs::msg::State current_state;

    bool setModeGuided(double delay = 5.0);
    bool arm(double delay = 5.0);
    bool takeoff(double altitude, double delay = 3.0);
    bool land(double delay = 10.0);

    bool isTimeElapsed(double delay);
    void updateLastRequestTime();

};

#endif // BASE_HPP