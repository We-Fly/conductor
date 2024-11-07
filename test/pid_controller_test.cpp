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

    rclcpp::NodeOptions options;

    auto node = std::make_shared<rclcpp::Node>("conductor_node", options);

    FixedPointYolo fixed_point_red(node, "/detected_circle", {640 / 2, 480 / 2}, pidpara, pidpara, "red", 20.0, 50);

    try
    {
        RCLCPP_INFO(node->get_logger(), "clear PID controller");
        fixed_point_red.clear();

        while (rclcpp::ok() && !is_interrupted)
        {
            if (!fixed_point_red.isTimeout())
            {
                RCLCPP_INFO(node->get_logger(), "PID aiding -> X: %f; Y: %f", fixed_point_red.getBoundedOutput().x, fixed_point_red.getBoundedOutput().y);
            }
            rclcpp::spin_some(node);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    // 在循环结束后调用rclcpp::shutdown()
    rclcpp::shutdown();
    return 0;
}
