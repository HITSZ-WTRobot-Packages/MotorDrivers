/**
 * @file    motor_vel_controller.cpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   通用速度控制器实现
 */
#include "motor_vel_controller.hpp"

namespace controllers
{
MotorVelController::MotorVelController(motors::IMotor* motor, const Config& cfg) :
    IController(motor, cfg.ctrl_mode), pid_(cfg.pid), internal_set_ratio_(cfg.internal_set_ratio)
{
}

MotorVelController::~MotorVelController()
{
    if (motor_)
        motor_->releaseController(this);
}

void MotorVelController::update()
{
    if (!enabled() || !motor_)
        return;

    // 如果驱动器自己带速度环，就直接把目标速度交给驱动器，避免重复做外部 PID。
    if (ctrl_mode_ == ControlMode::InternalVel || ctrl_mode_ == ControlMode::InternalVelPos)
    {
        ++internal_set_prescaler_;
        if (internal_set_prescaler_ >= internal_set_ratio_)
        {
            motor_->setInternalVelocity(velocity_target_);
            internal_set_prescaler_ = 0;
        }
        return;
    }

    // 外部 PID 模式：根据当前速度反馈算出电流 / 力矩类输出。
    const float output = pid_.calc(velocity_target_, motor_->getVelocity());

    motor_->setCurrent(output);
}

void MotorVelController::setRef(const float velocity)
{
    velocity_target_ = velocity;

    // 内部速度模式下，改参考值后立即发送一次，减少等待下个周期的迟滞。
    if (ctrl_mode_ == ControlMode::InternalVel || ctrl_mode_ == ControlMode::InternalVelPos)
    {
        if (motor_)
            motor_->setInternalVelocity(velocity_target_);
    }
}

} // namespace controllers
