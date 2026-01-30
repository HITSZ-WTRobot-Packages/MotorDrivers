/**
 * @file    motor_vel_controller.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief
 */
#ifndef MOTOR_VEL_CONTROLLER_HPP
#define MOTOR_VEL_CONTROLLER_HPP
#include "motor_if.hpp"
#include "pid_motor.hpp"

namespace controllers
{

class MotorVelController final : public IController
{
public:
    struct Config
    {
        PIDMotor::Config pid{}; ///< 在 InternalVelocity 或 InternalVelPos 模式下该项无效
        ControlMode      ctrl_mode = ControlMode::Default;
    };

    MotorVelController(motors::IMotor* motor, const Config& cfg);
    ~MotorVelController() override;

    void update() override;
    void setRef(const float& velocity);

private:
    PIDMotor pid_;
    float    velocity_target_ = 0.0f;
};

} // namespace controllers

#endif // MOTOR_VEL_CONTROLLER_HPP
