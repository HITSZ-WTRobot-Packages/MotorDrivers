/**
 * @file    motor_pos_controller.cpp
 * @author  syhanjin
 * @date    2026-01-28
 */
#include "motor_pos_controller.hpp"

namespace controllers
{
MotorPosController::MotorPosController(motors::IMotor* motor, const Config& cfg) :
    IController(motor, cfg.ctrl_mode), position_pid_(cfg.position_pid),
    velocity_pid_(cfg.velocity_pid), pos_vel_freq_ratio_(cfg.pos_vel_freq_ratio)
{
}

MotorPosController::~MotorPosController()
{
    if (motor_)
        motor_->releaseController(this);
}

void MotorPosController::update()
{
    if (!enabled() || !motor_)
        return;

    ++counter_;

    // If fully internal pos+vel mode, sending internal position immediately
    if (ctrl_mode_ == ControlMode::InternalVelPos)
    {
        motor_->setInternalVelocity(position_ref_);
        counter_ = 0;
        return;
    }

    // position loop executed at pos_vel_freq_ratio
    if (counter_ >= pos_vel_freq_ratio_)
    {
        position_pid_.calc(position_ref_, motor_->getAngle());
        counter_ = 0;
    }

    // runtime actions: prefer internal velocity if supported when configured
    if (ctrl_mode_ == ControlMode::InternalVelocity)
    {
        motor_->setInternalVelocity(position_pid_.getOutput());
        return;
    }

    // External PID path
    const float output = velocity_pid_.calc(position_pid_.getOutput(), motor_->getVelocity());

    motor_->setCurrent(output);
}

} // namespace controllers