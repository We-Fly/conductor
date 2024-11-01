#include "conductor/pid_controller.hpp"
#include "conductor/bound.hpp"

PIDController::PIDController(double kp, double ki, double kd, double windup_guard, double output_bound, double sample_time)
    : kp_(kp), ki_(ki), kd_(kd), windup_guard_(windup_guard), output_bound_(output_bound), sample_time_(sample_time),
      error_(0.0), last_error_(0.0), integral_(0.0), derivative_(0.0), last_control_output_(0.0), ros_clock_(rclcpp::Clock(RCL_ROS_TIME)),
      last_time_(ros_clock_.now()), current_time_(ros_clock_.now())
{
    clear();
}

PIDController::PIDController(const pid_controller::PidParams &params)
    : kp_(params.kp), ki_(params.ki), kd_(params.kd), windup_guard_(params.windup_guard), output_bound_(params.output_bound), sample_time_(params.sample_time),
      error_(0.0), last_error_(0.0), integral_(0.0), derivative_(0.0), last_control_output_(0.0), ros_clock_(rclcpp::Clock(RCL_ROS_TIME)),
      last_time_(ros_clock_.now()), current_time_(ros_clock_.now())
{
    clear();
}

void PIDController::clear()
{
    last_error_ = 0.0;
    setpoint_ = 0.0;
    p_term_ = 0.0;
    i_term_ = 0.0;
    d_term_ = 0.0;
    last_control_output_ = 0.0;
}

// 位置式PID控制
double PIDController::calcOutputRealize(double feedback_value)
{
    // 计算误差
    double error = setpoint_ - feedback_value;

    // 计算距离上次更新经过的时间
    double delta_time = (ros_clock_.now() - last_time_).seconds();

    // 计算误差的变化率
    double delta_error = error - last_error_;

    // 检查是否满足采样时间条件
    if (delta_time >= sample_time_)
    {
        // 计算比例项
        p_term_ = kp_ * error;

        // 计算积分项并防止积分饱和
        i_term_ += error * delta_time;
        i_term_ = Bound<double>(i_term_, windup_guard_);

        // 计算微分项
        d_term_ = 0.0;
        if (delta_time > 0)
        {
            d_term_ = delta_error / delta_time;
        }

        // 更新上次更新时间和上次误差
        last_time_ = ros_clock_.now();
        last_error_ = error;
    }
    
    // 计算PID控制器输出
    last_control_output_ = p_term_ + (ki_ * i_term_) + (kd_ * d_term_);
    return last_control_output_;
}

double PIDController::calcOutput()
{
    return last_control_output_;
}

double PIDController::calcOutput(double feedback_value)
{
    return calcOutputRealize(feedback_value);
}

double PIDController::getBoundedOutput(double feedback_value)
{
    return Bound<double>(calcOutput(feedback_value), output_bound_);
}

double PIDController::getBoundedOutput()
{
    return Bound<double>(calcOutput(), output_bound_);
}

void PIDController::setKp(double proportional_gain)
{
    kp_ = proportional_gain;
}

void PIDController::setKi(double integral_gain)
{
    ki_ = integral_gain;
}

void PIDController::setKd(double derivative_gain)
{
    kd_ = derivative_gain;
}

void PIDController::setWindup(double windup)
{
    windup_guard_ = windup;
}

void PIDController::setSampleTime(double sample_time)
{
    sample_time_ = sample_time;
}

void PIDController::setOutputBound(double output_bound)
{
    output_bound_ = output_bound;
}

void PIDController::setSetpoint(double setpoint)
{
    setpoint_ = setpoint;
}
