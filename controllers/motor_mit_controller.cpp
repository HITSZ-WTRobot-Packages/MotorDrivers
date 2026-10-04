/*  
 * @file    motor_mit_controller.cpp
 * @author  Fang_Shuohan
 * @date    2026-10-03
 * @brief   通用mit模式控制器实现
 */

#include "motor_mit_controller.hpp"
#include <algorithm>
#include <cmath>


constexpr float kDegToRad = 0.01745329252f;
constexpr float kRpmToRadS = 0.10471975512f;

namespace controllers
{
MotorMITController::MotorMITController(motors::IMotor* motor, const Config& cfg) :
    IController(motor, cfg.ctrl_mode),
    mit_(MITPD::Config{cfg.mit.Kp, cfg.mit.Kd, std::fabs(cfg.mit.abs_output_max)}),
    kp_(cfg.mit.Kp),
    kd_(cfg.mit.Kd),
    internal_set_ratio_(std::max(uint32_t{1}, cfg.internal_set_ratio))
{
}

MotorMITController::~MotorMITController()
{
    if (motor_)
        motor_->releaseController(this);
}

bool MotorMITController::enable()
{
    if (!motor_)
        return false;
    if (ctrl_mode_ == ControlMode::InternalMIT)
    {
        if (!motor_->supportsInternalMIT())
            return false;
    }
    else if (ctrl_mode_ == ControlMode::ExternalPID)
    {
        if (!motor_->supportsCurrent())
            return false;
    }
    else
    {
        return false; // MIT 控制器不接受内部速度/位置模式。
    }

    const bool was_enabled = enabled();
    if (!IController::enable())
        return false;
    if (!was_enabled)
    {
        mit_.reset();
        // 启用后的第一次 update 就发送，之后按配置分频。
        internal_set_prescaler_ = internal_set_ratio_ - 1;
    }
    return true;
}

void MotorMITController::update()
{
    if (!enabled() || !motor_)
        return;

    if (ctrl_mode_ == ControlMode::InternalMIT)
    {
        if (++internal_set_prescaler_ >= internal_set_ratio_)
        {
            motor_->setInternalMIT(t_ff_, position_ref_, velocity_ref_, kp_, kd_);
            internal_set_prescaler_ = 0;
        }
        return;
    }


    float output = mit_.calc(position_ref_ * kDegToRad,
                             motor_->getAngle() * kDegToRad,
                             velocity_ref_ * kDegToRad,
                             motor_->getVelocity() * kRpmToRadS,
                             t_ff_);
    if (max_current_ != kNoLimit)
        output = std::clamp(output, -max_current_, max_current_);
    motor_->setCurrent(output);
}

void MotorMITController::setRef(const float position_deg, const float velocity_degs,
                                const float max_current, const float t_ff)
{
    position_ref_ = position_deg;
    velocity_ref_ = velocity_degs;
    max_current_ = std::fabs(max_current);
    t_ff_ = t_ff;


    // 内部MIT模式下，改参考值后立即发送一次，减少等待下个周期的迟滞。
    if (enabled() && motor_ && ctrl_mode_ == ControlMode::InternalMIT)
    {
        motor_->setInternalMIT(t_ff_, position_ref_, velocity_ref_, kp_, kd_);
        internal_set_prescaler_ = 0;
    }
}

} // namespace controllers
