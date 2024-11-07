#ifndef APM_HPP
#define APM_HPP

#include "conductor/base.hpp"
// #include "conductor/waypoint.hpp"
#include "conductor/ansi_color.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geographic_msgs/msg/geo_point_stamped.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/command_code.hpp>

// 起飞状态
enum class APMTakeoffState
{
    kLand,
    kArmed,
    kTakeoff1,
    kTakeoff2,
};

class ArduConductor : public BaseConductor
{
protected:
    void initNode() override;
    APMTakeoffState flag_takeoff;

public:
    ArduConductor(rclcpp::Node::SharedPtr node);
    
    bool takeoff(double pre_takeoff_alt, double altitude, double delay = 3.0); // 重写二段起飞方法
    bool arm(double delay = 5.0);
    bool land(double delay = 10.0);

    void sendGpOrigin(double latitude = 32.108693377508494, double longitude = 118.92943049870283);
    bool setMoveSpeed(double speed);
    void setSpeedBody(double x, double y, double z, double yaw_rate);
    void setAngularRate(double yaw_rate);
    void setPoseBody(double x, double y, double z, double yaw);
    void setPoseRelated(double x, double y, double z, double yaw);
    void setPoseWorld(double x, double y, double z, double yaw) const;
    // void setPoseWorld(waypoint::Waypoint waypoint);
    // void setWaypointPoseWorld(waypoint::Waypoint waypoint);
    void sendTranslatedPoseWorld(double x, double y, double z, double yaw) const;
    void setBreak();

    ~ArduConductor(){};
};

#endif // APM_HPP