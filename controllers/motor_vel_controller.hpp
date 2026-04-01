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
#include <cstdint>

namespace controllers
{

class MotorVelController final : public IController
{
public:
    struct Config
    {
        PIDMotor::Config pid{}; ///< 在 InternalVelocity 或 InternalVelPos 模式下该项无效
        ControlMode      ctrl_mode = ControlMode::Default;

        /**
         * 暂定该名字，当可以发送内部速度指令时，发送频率相对于更新频率的分频。
         * 用于只需要较低频率维持控制指令，同时需要节约总线负载的情况
         */
        uint32_t internal_set_ratio = 10;
    };

    MotorVelController(motors::IMotor* motor, const Config& cfg);
    ~MotorVelController() override;

    bool enable() override
    {
        // 速度环控制器不支持内部控制模式
        return ctrl_mode_ != ControlMode::InternalPos && IController::enable();
    }

    void update() override;
    void setRef(const float& velocity);

    auto& getPID() { return pid_; }

private:
    PIDMotor pid_;
    float    velocity_target_ = 0.0f;

    size_t internal_set_prescaler_ = 0;
    size_t internal_set_ratio_;
};

} // namespace controllers

#endif // MOTOR_VEL_CONTROLLER_HPP
