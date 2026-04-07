/**
 * @file    motor_vel_controller.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   通用速度控制器
 *
 * 这里的“速度”应理解为电机角速度参考。
 * 整个电机控制驱动里，速度默认单位都是 rpm，除非变量名明确带有 `_rad`、`_rps` 等后缀。
 * 控制器要求参考值和绑定电机的 `getVelocity()` / `setInternalVelocity()` 保持一致。
 */
#ifndef MOTOR_VEL_CONTROLLER_HPP
#define MOTOR_VEL_CONTROLLER_HPP
#include "motor_if.hpp"
#include "pid_motor.hpp"
#include <cstdint>

namespace controllers
{

/**
 * @brief 电机速度控制器
 *
 * 它负责把“目标速度”转换成电机可以执行的命令：
 * - 外部 PID 模式下，输出为电流 / 力矩类命令
 * - 内部速度模式下，直接把目标速度交给驱动器内部闭环
 */
class MotorVelController final : public IController
{
public:
    /**
     * @brief 速度控制器配置
     */
    struct Config
    {
        PIDMotor::Config pid{}; ///< 外部 PID 配置；在内部速度模式下该项无效
        ControlMode      ctrl_mode = ControlMode::Default; ///< 控制模式，Default 时跟随电机默认模式

        /**
         * 当可以发送内部速度指令时，发送频率相对于 update 频率的分频。
         * 用于只需要较低频率维持控制指令，同时需要节约总线负载的情况
         */
        uint32_t internal_set_ratio = 10; ///< 应 >= 1
    };

    /**
     * @param motor 被控制的电机
     * @param cfg 控制器配置
     */
    MotorVelController(motors::IMotor* motor, const Config& cfg);
    ~MotorVelController() override;

    bool enable() override
    {
        // 速度控制器不支持 InternalPos，因为该模式表示电机只能接收位置指令。
        return ctrl_mode_ != ControlMode::InternalPos && IController::enable();
    }

    /**
     * @brief 周期更新控制器
     */
    void update() override;
    /**
     * @brief 设置速度参考
     * @param velocity 目标输出轴速度，单位 rpm
     */
    void setRef(float velocity);

    /**
     * @brief 直接访问内部 PID 对象
     */
    auto& getPID() { return pid_; }

private:
    PIDMotor pid_;                    ///< 速度环 PID
    float    velocity_target_ = 0.0f; ///< 目标速度，默认单位 rpm

    size_t internal_set_prescaler_ = 0; ///< 内部速度指令发送分频计数器
    size_t internal_set_ratio_;         ///< 内部速度指令发送分频比
};

} // namespace controllers

#endif // MOTOR_VEL_CONTROLLER_HPP
