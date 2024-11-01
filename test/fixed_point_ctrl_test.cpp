/**
 * @file fixed_point_ctrl.cpp
 * @brief 定点测试，使用Conductor封装库
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "conductor/fixed_point.hpp"

#include "conductor/mission_state.hpp"
#include "conductor/ansi_color.hpp"
#include "conductor/apm.hpp"
// #include "conductor/fixed_point.hpp"

#include <signal.h> // Necessary for the Custom SIGINT handler
#include <memory>   // For std::shared_ptr

#define PI 3.1415926535

bool is_interrupted = false;

// 定义一个安全的SIGINT处理函数
void safeSigintHandler(int sig)
{
    (void)sig;
    // 设置中断标志
    is_interrupted = true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 设置SIGINT处理
    signal(SIGINT, safeSigintHandler);

    pid_controller::PidParams pidpara{0.17, 0.0, 0.015, 10.0, 40, 0.0};

    auto apm = std::make_shared<ArduConductor>("ArduPilot_Guided_Node");
    FixedPointYolo fixed_point_red(apm, "detected_circle", {640 / 2, 480 / 2}, pidpara, pidpara, "red", 20, 50);

    // wait for FCU connection
    while (rclcpp::ok() && !apm->current_state.connected && !is_interrupted)
    {
        if (is_interrupted)
        {
            rclcpp::shutdown();
            return 0;
        }
        rclcpp::spin_some(apm);
        apm->rate.sleep();
    }

    RCLCPP_INFO(apm->get_logger(), SUCCESS("Drone connected!"));

    // 重置上一次操作的时间为当前时刻
    apm->last_request = apm->now();
    int count = 0;
    while (rclcpp::ok() && !is_interrupted)
    {
        // 任务执行状态机
        switch (apm->mission_state)
        {
        case MissionState::kPrearm:
            if (apm->setModeGuided(5.0)) // 修改飞行模式为 Guided (ArduCopter)
            {
                apm->sendGpOrigin();    // 如果切换成Guided模式就发送全局原点坐标
                apm->setMoveSpeed(0.15); // 设置空速
            }
            break;

        case MissionState::kArm:
            apm->arm(5.0); // 解锁电机
            break;

        case MissionState::kTakeoff:
            if (apm->takeoff(0.5, 1.0, 3.0)) // 起飞到1M高度
            {
                apm->mission_state = MissionState::kPose;
                count = 0;
                RCLCPP_INFO(apm->get_logger(), MISSION_SWITCH_TO("pose"));
            }
            break;

        case MissionState::kPose:
            if (apm->isTimeElapsed(1.0) && count == 0)
            {
                RCLCPP_INFO(apm->get_logger(), "clear PID controller");
                fixed_point_red.clear();
                count++;
            }
            else if (apm->isTimeElapsed(4.0) && count == 1)
            {
                apm->setSpeedBody(fixed_point_red.getBoundedOutput().x * 0.01, fixed_point_red.getBoundedOutput().y * 0.01, 0, 0);
                RCLCPP_INFO(apm->get_logger(), "start PID aiding");

                if (apm->isTimeElapsed(20))
                {
                    apm->setSpeedBody(0, 0, 0, 0); // stop the copter
                    count++;
                }
            }
            else if (count == 2)
            {
                apm->last_request = apm->now();
                apm->setSpeedBody(0, 0, 0, 0);
                RCLCPP_INFO(apm->get_logger(), "Landing after 3s");
                apm->mission_state = MissionState::kLand;
            }
            break;

        case MissionState::kLand:
            if (apm->land(3.0)) // 3s后降落
            {
                rclcpp::shutdown();
            }
            break;

        default:
            break;
        }

        rclcpp::spin_some(apm);
        apm->rate.sleep();
    }
    // 在循环结束后调用rclcpp::shutdown()
    rclcpp::shutdown();
    return 0;
}
