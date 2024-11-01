#ifndef FIXED_POINT_HPP
#define FIXED_POINT_HPP

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include "conductor/pid_controller.hpp"
#include "conductor/bound.hpp"
#include "conductor/ansi_color.hpp"
#include <std_msgs/msg/string.hpp>
#include "conductor/msg/target_object.hpp"

namespace fixed_point
{
    // Size模板类定义
    template <typename T>
    class Size_
    {
    public:
        Size_() : width(0), height(0) {}
        Size_(T w, T h) : width(w), height(h) {}

        T width;
        T height;
    };

    // Point模板类定义
    template <typename T>
    class Point_
    {
    public:
        Point_() : x(0), y(0) {}
        Point_(T x, T y) : x(x), y(y) {}

        T x;
        T y;
    };

    typedef Size_<int> Size2i;
    typedef Size2i Size;

    typedef Point_<double> Point2d;
    typedef Point2d Point;
}

class FixedPoint
{
public:
    FixedPoint(rclcpp::Node::SharedPtr node, const std::string &topic, fixed_point::Point frame_center, const pid_controller::PidParams &params_x, const pid_controller::PidParams &params_y);

    auto calcXYOutput(fixed_point::Point offset) -> fixed_point::Point;
    auto calcXYOutput() -> fixed_point::Point;
    auto getBoundedOutput(fixed_point::Point offset) -> fixed_point::Point;
    auto getBoundedOutput() -> fixed_point::Point;
    rclcpp::Time getTimeNow();
    rclcpp::Logger get_logger();

    void clear();

    ~FixedPoint() {};

protected:
    void subPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
    fixed_point::Point last_output_;

private:
    rclcpp::Node::SharedPtr node_;                                         // 节点智能指针
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_; // 订阅器
    geometry_msgs::msg::Point point_;                                      // 目标点数据

public:
    fixed_point::Size frame_size_;
    fixed_point::Point frame_center_;
    fixed_point::Point output_bound_;

    PIDController pid_controller_x; // X轴PID控制器
    PIDController pid_controller_y; // Y轴PID控制器
};

class FixedPointYolo : public FixedPoint
{
public:
    FixedPointYolo(rclcpp::Node::SharedPtr node,
                   const std::string &topic,
                   fixed_point::Point frame_center,
                   const pid_controller::PidParams &params_x,
                   const pid_controller::PidParams &params_y,
                   const std::string &target_id,
                   double lock_threshold_distance,
                   int lock_cutoff);

    std::string target_id_;                      // 订阅的yolo标签
    bool is_found_;
    double lock_threshold_distance_;
    int locked_count_;
    bool is_dropped_;
    bool is_locked_;
    bool is_lock_counter_reached();
    int lock_cutoff_;
    void reset();
    void updateTime();
    bool isTimeout();
    int timeout_count_;

protected:
    void subPointCallback(const conductor::msg::TargetObject::SharedPtr msg);
    conductor::msg::TargetObject target_object_; // 目标点数据 - 自定义消息类型
    rclcpp::Time last_frame_time_;
};

#endif // FIXED_POINT_HPP