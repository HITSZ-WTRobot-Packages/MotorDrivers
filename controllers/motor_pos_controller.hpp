/**
 * @file    motor_pos_controller.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#ifndef MOTOR_POS_CONTROLLER_HPP
#define MOTOR_POS_CONTROLLER_HPP
#include "motor_if.hpp"
#include "pid_motor.hpp"

#include <cstdint>

namespace controllers
{

class MotorPosController final : public IController
{
public:
    struct Config
    {
        PIDMotor::Config position_pid{};
        PIDMotor::Config velocity_pid{};
        uint32_t         pos_vel_freq_ratio = 1;

        ControlMode ctrl_mode = ControlMode::Default;
    };

    MotorPosController(motors::IMotor* motor, const Config& cfg);
    ~MotorPosController() override;

    void update() override;
    void setRef(float position);

private:
    PIDMotor position_pid_;
    PIDMotor velocity_pid_;

    uint32_t counter_      = 0;
    float    position_ref_ = 0.0f;

    uint32_t pos_vel_freq_ratio_;
};

} // namespace controllers

#endif // MOTOR_POS_CONTROLLER_HPP
