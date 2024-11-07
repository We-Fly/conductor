#include "conductor/fixed_point.hpp"

FixedPointYolo::FixedPointYolo(rclcpp::Node::SharedPtr node,
                               const std::string &topic,
                               fixed_point::Point frame_center,
                               const pid_controller::PidParams &params_x,
                               const pid_controller::PidParams &params_y,
                               const std::string &target_id,
                               double lock_threshold_distance,
                               int lock_cutoff)
    : FixedPoint(node, "none", frame_center, params_x, params_y), target_id_(target_id), is_found_(false),
      lock_threshold_distance_(lock_threshold_distance), locked_count_(0), is_dropped_(false), is_locked_(false), lock_cutoff_(lock_cutoff),
      timeout_count_(0)
{
    last_frame_time_ = getTimeNow();
    this->init(topic);
}

void FixedPointYolo::init(const std::string &topic)
{
    // 订阅目标点数据
    target_object_sub_ = this->node_->create_subscription<conductor::msg::TargetObject>(
        topic, 10, std::bind(&FixedPointYolo::subTargetObjectCallback, this, std::placeholders::_1));
}

bool FixedPointYolo::is_lock_counter_reached()
{
    return locked_count_ > lock_cutoff_;
}

void FixedPointYolo::reset()
{
    is_found_ = false;
    locked_count_ = 0;
    is_dropped_ = false;
    is_locked_ = false;
}

void FixedPointYolo::updateTime()
{
    this->last_frame_time_ = getTimeNow();
}

bool FixedPointYolo::isTimeout()
{
    return (getTimeNow() - this->last_frame_time_) > std::chrono::seconds(1);
}

void FixedPointYolo::subTargetObjectCallback(const conductor::msg::TargetObject::SharedPtr msg)
{
    // OpenCV 坐标系：X 向右增大，Y 向下增大
    // 飞机坐标系：X 对应 OpenCV 的 -Y，Y 对应 OpenCV 的 -X

    this->target_object_ = *msg;

    // 从 header 中获取 yolotag
    std::string yolotag = msg->header.frame_id; // yolotag 存储在 header 的 frame_id 字段
    if (target_id_ == yolotag)
    {
        this->is_found_ = true;
        this->updateTime();
        RCLCPP_INFO(this->get_logger(), SUCCESS("\n--------------------------"));
        RCLCPP_INFO(this->get_logger(), SUCCESS("\nGot a Yolo tag: %s"), yolotag.c_str());
        RCLCPP_INFO(this->get_logger(), SUCCESS("\npoint \nX: %0.2f\nY: %0.2f"), msg->center.x, msg->center.y);

        auto offset = fixed_point::Point(
            target_object_.center.y - frame_center_.y,
            target_object_.center.x - frame_center_.x);

        double distance_ = std::sqrt(offset.x * offset.x + offset.y * offset.y);
        if (distance_ < lock_threshold_distance_)
        {
            locked_count_++;
            RCLCPP_INFO(this->get_logger(), SUCCESS("\nOffset \nX: %0.2f\nY: %0.2f, distance: %0.2f"), offset.x, offset.y, distance_);
        }
        else
        {
            locked_count_ = 0;
            RCLCPP_INFO(this->get_logger(), COLORED_TEXT("\nOffset \nX: %0.2f\nY: %0.2f, distance: %0.2f", ANSI_BACKGROUND_RED), offset.x, offset.y, distance_);
        }

        calcXYOutput(offset);

        RCLCPP_INFO(this->get_logger(), SUCCESS("\nOutput \nX: %0.2f\nY: %0.2f"), last_output_.x, last_output_.y);
        
        RCLCPP_INFO(this->get_logger(), SUCCESS("\n--------------------------"));
    }
}