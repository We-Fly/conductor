#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>

namespace pid_controller
{
struct PidParams
{
    double kp;
    double ki;
    double kd;
    double windup_guard;
    double output_bound;
    double sample_time;
};
}

class PIDController
{
public:
    PIDController(double kp, double ki, double kd, double windup_guard, double output_bound, double sample_time);
    PIDController(const pid_controller::PidParams& params);
    double calcOutputRealize(double feedback_value);
    double calcOutput();
    double calcOutput(double feedback_value);
    double getBoundedOutput(double feedback_value);
    double getBoundedOutput();
    void clear();

    void setSetpoint(double setpoint);
    void setKp(double proportional_gain);
    void setKi(double integral_gain);
    void setKd(double derivative_gain);
    void setWindup(double windup);
    void setSampleTime(double sample_time);
    void setOutputBound(double output_bound);

private:
    double kp_;
    double ki_;
    double kd_;
    double windup_guard_;
    double output_bound_;
    double sample_time_;
    double error_;
    double last_error_;
    double integral_;
    double derivative_;
    double setpoint_;
    double p_term_;
    double i_term_;
    double d_term_;
    double last_control_output_;
    rclcpp::Clock ros_clock_; // 使用 ROS 时间
    rclcpp::Time last_time_;
    rclcpp::Time current_time_;
};

#endif // PID_CONTROLLER_HPP