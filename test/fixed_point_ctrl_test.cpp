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

    // PidParams pidpara{0.17, 0.0, 0.015, 10.0, 40, 0.0};

    auto apm = std::make_shared<ArduConductor>("ArduPilot_Guided_Node");
    // FixedPoint fixed_point_red("filter_out", {800 / 2, 330}, node, pidpara, pidpara);

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
            if (apm->takeoff(0.5, 1.0)) // 起飞到1M高度
            {
                apm->mission_state = MissionState::kPose;
                RCLCPP_INFO(apm->get_logger(), MISSION_SWITCH_TO("pose"));
            }
            break;

        case MissionState::kPose:
            if (apm->isTimeElapsed(2.0) && count == 0)
            {
                apm->setPoseBody(0, 0, 0.6, 0);
                RCLCPP_INFO(apm->get_logger(), "takeoff to 1.0");
                // fixed_point_red.clear();
                count++;
            }
            else if (apm->isTimeElapsed(4.0) && count == 1)
            {
                // apm->setSpeedBody(fixed_point_red.getBoundedOutput().x * 0.01, fixed_point_red.getBoundedOutput().y * 0.01, 0, 0);

                if (apm->isTimeElapsed(20))
                {
                    apm->setSpeedBody(0, 0, 0, 0);
                    count++;
                }
            }
            else if (apm->isTimeElapsed(20.0))
            {
                apm->last_request = apm->now();
                apm->setSpeedBody(0, 0, 0, 0);
                apm->mission_state = MissionState::kLand;
            }
            break;

        case MissionState::kLand:
            if (apm->land(5.0)) // 10s后降落
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
