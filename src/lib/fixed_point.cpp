#include "conductor/fixed_point.hpp"

FixedPoint::FixedPoint(rclcpp::Node::SharedPtr node, const std::string &topic, fixed_point::Point frame_center, const pid_controller::PidParams &params_x, const pid_controller::PidParams &params_y)
    : node_(node), frame_center_(frame_center), output_bound_(params_x.output_bound, params_y.output_bound), 
    pid_controller_x(params_x), pid_controller_y(params_y)
{
    if (topic != "none")
    {
        // 订阅目标点数据
        point_sub_ = node_->create_subscription<geometry_msgs::msg::Point>(
            topic, 10, std::bind(&FixedPoint::subPointCallback, this, std::placeholders::_1));
    }
}

void FixedPoint::subPointCallback(const geometry_msgs::msg::Point::SharedPtr msg)
{
    // Opencv 坐标系 往右X增大 往下Y增大
    // 飞机坐标系 X对应Opencv的-Y Y对应Opencv的-X
    RCLCPP_INFO(node_->get_logger(), SUCCESS("\n--------------------------"));
    this->point_ = *msg;
    RCLCPP_INFO(node_->get_logger(), SUCCESS("\nGot point \nX: %0.2f\nY: %0.2f"), point_.x, point_.y);
    auto offset = fixed_point::Point(point_.y - frame_center_.y, point_.x - frame_center_.x);
    RCLCPP_INFO(node_->get_logger(), SUCCESS("\nOffset \nX: %0.2f\nY: %0.2f"), offset.x, offset.y);
    calcXYOutput(offset);
    RCLCPP_INFO(node_->get_logger(), SUCCESS("\nOutput \nX: %0.2f\nY: %0.2f"), last_output_.x, last_output_.y);
    RCLCPP_INFO(node_->get_logger(), SUCCESS("\n--------------------------"));
}

auto FixedPoint::calcXYOutput(fixed_point::Point offset) -> fixed_point::Point
{
    last_output_.x = pid_controller_x.calcOutput(offset.x);
    last_output_.y = pid_controller_y.calcOutput(offset.y);
    return last_output_;
}

auto FixedPoint::calcXYOutput() -> fixed_point::Point
{
    return last_output_;
}

auto FixedPoint::getBoundedOutput(fixed_point::Point offset) -> fixed_point::Point
{
    calcXYOutput(offset);
    return {Bound<double>(last_output_.x, output_bound_.x), Bound<double>(last_output_.y, output_bound_.y)};
}

auto FixedPoint::getBoundedOutput() -> fixed_point::Point
{
    return {Bound<double>(last_output_.x, output_bound_.x), Bound<double>(last_output_.y, output_bound_.y)};
}

rclcpp::Time FixedPoint::getTimeNow()
{
    return this->node_->now();
}

rclcpp::Logger FixedPoint::get_logger()
{
    return this->node_->get_logger();
}

void FixedPoint::clear()
{
    last_output_.x = 0.0;
    last_output_.y = 0.0;
    pid_controller_x.clear();
    pid_controller_y.clear();
}
