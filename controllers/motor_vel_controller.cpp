/**
 * @file    motor_vel_controller.cpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#include "motor_vel_controller.hpp"

namespace controllers
{
MotorVelController::MotorVelController(motors::IMotor* motor, const Config& cfg) :
    IController(motor, cfg.ctrl_mode), pid_(cfg.pid)
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

    // If controller requested internal velocity (or internal vel+pos), prefer that
    if (ctrl_mode_ == ControlMode::InternalVelocity || ctrl_mode_ == ControlMode::InternalVelPos)
    {
        motor_->setInternalVelocity(velocity_target_);
        return;
    }

    // External PID path
    const float output = pid_.calc(velocity_target_, motor_->getVelocity());

    motor_->setCurrent(output);
}

void MotorVelController::setRef(const float& velocity)
{
    velocity_target_ = velocity;
    // Sending internal velocity immediately is decided by resolved control mode
    if (ctrl_mode_ == ControlMode::InternalVelocity || ctrl_mode_ == ControlMode::InternalVelPos)
    {
        if (motor_)
            motor_->setInternalVelocity(velocity_target_);
    }
}

} // namespace controllers