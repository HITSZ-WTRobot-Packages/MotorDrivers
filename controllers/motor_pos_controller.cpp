/**
 * @file    motor_pos_controller.cpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   通用位置控制器实现
 */
#include "motor_pos_controller.hpp"

namespace controllers
{
MotorPosController::MotorPosController(motors::IMotor* motor, const Config& cfg) :
    IController(motor, cfg.ctrl_mode), position_pid_(cfg.position_pid),
    velocity_pid_(cfg.velocity_pid), pos_vel_freq_ratio_(cfg.pos_vel_freq_ratio),
    internal_set_ratio_(cfg.internal_set_ratio)
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

    // 对于 InternalPos / InternalVelPos，位置控制器都直接走“发送位置指令”这一路。
    if (ctrl_mode_ == ControlMode::InternalVelPos || ctrl_mode_ == ControlMode::InternalPos)
    {
        ++internal_set_prescaler_;
        if (internal_set_prescaler_ >= internal_set_ratio_)
        {
            motor_->setInternalPosition(position_ref_);
            internal_set_prescaler_ = 0;
        }
        return;
    }

    ++pos_vel_prescaler_;

    // 位置环不一定每次 update 都执行，可以按分频降低运算量或总线负载。
    if (pos_vel_prescaler_ >= pos_vel_freq_ratio_)
    {
        // 位置环的输出被约定为“目标角速度”。
        position_pid_.calc(position_ref_, motor_->getAngle());

        // 如果电机内部有速度环，这里直接把位置环输出作为速度参考交给驱动器。
        if (ctrl_mode_ == ControlMode::InternalVel)
        {
            motor_->setInternalVelocity(position_pid_.getOutput());
        }

        pos_vel_prescaler_ = 0;
    }

    if (ctrl_mode_ == ControlMode::InternalVel)
    {
        return;
    }

    // 外部串级模式：位置环先给出目标角速度，速度环再根据当前速度算出电流 / 力矩输出。
    const float output = velocity_pid_.calc(position_pid_.getOutput(), motor_->getVelocity());

    motor_->setCurrent(output);
}

void MotorPosController::setRef(const float position)
{
    position_ref_ = position;

    // InternalPos / InternalVelPos 下，改参考值后立即补发一次位置指令，减少响应延迟。
    if (ctrl_mode_ == ControlMode::InternalVelPos || ctrl_mode_ == ControlMode::InternalPos)
    {
        if (motor_)
            motor_->setInternalPosition(position_ref_);
    }
}

} // namespace controllers
