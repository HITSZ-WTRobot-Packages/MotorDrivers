/**
 * @file    motor_vel_controller.cpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   通用速度控制器实现
 */
#include "motor_vel_controller.hpp"
#include <algorithm>

namespace controllers
{
MotorVelController::MotorVelController(motors::IMotor* motor, const Config& cfg) :
    IController(motor, cfg.ctrl_mode), pid_(cfg.pid), internal_set_ratio_(cfg.internal_set_ratio)
{
}

bool MotorVelController::selfEnable()
{
    // 使用已经解析过的模式；Default 解析为 InternalPos 时同样拒绝启用。
    return ctrl_mode_ != ControlMode::InternalPos && IController::selfEnable();
}

void MotorVelController::selfProtect()
{
    pid_.reset();
    velocity_target_         = 0.0f;
    ff_                      = 0.0f;
    max_current_             = kNoLimit;
    internal_set_prescaler_  = 0;

    // 自身状态可能尚未提交；不要经 setRef() 的权限门建立保护参考。
    // 仅内部模式立即发送零速指令；外部 PID 留给 update() 按周期计算输出。
    if (ctrl_mode_ == ControlMode::InternalVel || ctrl_mode_ == ControlMode::InternalVelPos)
        motor_->setInternalVelocity(0.0f, kNoLimit);
}

void MotorVelController::selfDisable()
{
    pid_.reset();
    velocity_target_        = 0.0f;
    ff_                     = 0.0f;
    max_current_            = kNoLimit;
    internal_set_prescaler_ = 0;
}

void MotorVelController::update()
{
    if (!isEnabled())
        return;

    // 如果驱动器自己带速度环，就直接把目标速度交给驱动器，避免重复做外部 PID。
    if (ctrl_mode_ == ControlMode::InternalVel || ctrl_mode_ == ControlMode::InternalVelPos)
    {
        ++internal_set_prescaler_;
        if (internal_set_prescaler_ >= internal_set_ratio_)
        {
            motor_->setInternalVelocity(velocity_target_, max_current_);
            internal_set_prescaler_ = 0;
        }
        return;
    }

    // 外部 PID 模式：根据当前速度反馈算出电流 / 力矩类输出。
    float output = pid_.calc(velocity_target_, motor_->getVelocity(), ff_);

    if (max_current_ != kNoLimit)
        output = std::clamp(output, -max_current_, max_current_);

    motor_->setCurrent(output);
}

bool MotorVelController::setRef(const float velocity, const float max_current, const float ff)
{
    if (!canAcceptReference())
        return false;

    applyReference(velocity, max_current, ff);
    return true;
}

bool MotorVelController::setRefFrom(const core::control::ControllerNode& source,
                                    const float velocity, const float max_current, const float ff)
{
    if (!canAcceptReferenceFrom(source))
        return false;

    applyReference(velocity, max_current, ff);
    return true;
}

void MotorVelController::applyReference(const float velocity, const float max_current, const float ff)
{
    max_current_     = max_current;
    velocity_target_ = velocity;
    ff_              = ff;

    // 内部速度模式下，改参考值后立即发送一次，减少等待下个周期的迟滞。
    if (ctrl_mode_ == ControlMode::InternalVel || ctrl_mode_ == ControlMode::InternalVelPos)
    {
        motor_->setInternalVelocity(velocity_target_, max_current_);
    }
}

} // namespace controllers
