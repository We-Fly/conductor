#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include "conductor/pid_controller.h"
#include "conductor/bound.h"
#include "conductor/yolo.h"

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
    FixedPoint(const std::string &topic, fixed_point::Point center, ros::NodeHandle &nh, const PidParams &params_x, const PidParams &params_y);

    auto calcXYOutput(fixed_point::Point offset) -> fixed_point::Point;
    auto calcXYOutput() -> fixed_point::Point;
    auto getBoundedOutput(fixed_point::Point offset) -> fixed_point::Point;
    auto getBoundedOutput() -> fixed_point::Point;

    PIDController pid_controller_x; // X轴PID控制器
    PIDController pid_controller_y; // Y轴PID控制器
    void clear();

    ~FixedPoint(){};

protected:
    ros::NodeHandle nh_;         // ROS 节点句柄
    ros::Subscriber point_sub_;  // 订阅器 - 目标点数据
    geometry_msgs::Point point_; // 目标点数据

    fixed_point::Size frame_size_;
    fixed_point::Point center_;
    fixed_point::Point last_output_;

    fixed_point::Point output_bound_;

    void subPointCallback(const geometry_msgs::Point::ConstPtr &msg);
};

class FixedPointYolo : public FixedPoint
{
public:
    FixedPointYolo(const std::string &topic, const std::string &yolo_tag, fixed_point::Point center, double lock_distance, ros::NodeHandle &nh, const PidParams &params_x, const PidParams &params_y, int lock_cutoff);
    bool is_found_;
    bool is_dropped_;
    bool is_locked_;
    bool is_lock_counter_reached();
    int locked_count_;
    int lock_cutoff_;
    double lock_distance_;
protected:
    ros::Subscriber point_yolo_sub_;                                 // 订阅器 - 自定义消息类型
    conductor::yolo point_yolo_;                                     // 目标点数据 - 自定义消息类型
    std::string yolo_tag_;                                           // 订阅的yolo标签
    void subPointYoloCallback(const conductor::yolo::ConstPtr &msg); // 消息订阅回调函数
};

#endif // FIXED_POINT_H