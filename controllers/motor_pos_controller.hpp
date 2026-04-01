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

        /**
         * 暂定该名字，当可以发送内部位置指令时，发送频率相对于更新频率的分频。
         * 用于只需要较低频率维持控制指令，同时需要节约总线负载的情况
         *
         * 如果只能发送内部速度指令，则会降速度指令的频率为位置环更新频率
         */
        uint32_t internal_set_ratio = 10;
    };

    MotorPosController(motors::IMotor* motor, const Config& cfg);
    ~MotorPosController() override;

    void update() override;
    void setRef(float position);

private:
    PIDMotor position_pid_;
    PIDMotor velocity_pid_;

    uint32_t pos_vel_prescaler_ = 0;

    float position_ref_ = 0.0f;

    uint32_t pos_vel_freq_ratio_;

    size_t   internal_set_prescaler_ = 0;
    uint32_t internal_set_ratio_;
};

} // namespace controllers

#endif // MOTOR_POS_CONTROLLER_HPP
