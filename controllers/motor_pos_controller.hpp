/**
 * @file    motor_pos_controller.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   通用位置控制器
 *
 * 这个控制器统一处理“目标是一个角度”的场景。
 * 根据电机支持的能力和 `ControlMode` 的选择，它会自动走不同的控制路径：
 * - `InternalPos`：电机只支持位置指令时，直接把目标角度交给驱动器
 * - `InternalVelPos`：电机同时支持速度指令和位置指令时，位置控制器直接发送位置指令
 * - 内部速度模式：位置环先算出目标角速度，再交给驱动器内部速度环
 * - 外部 PID 模式：位置环算目标角速度，速度环再算电流输出
 */
#ifndef MOTOR_POS_CONTROLLER_HPP
#define MOTOR_POS_CONTROLLER_HPP
#include "motor_if.hpp"
#include "pid_motor.hpp"

#include <cstdint>

namespace controllers
{

/**
 * @brief 电机位置控制器
 *
 * 对初学者来说，可以把它理解成“把目标角度逐步变成驱动器能理解的控制命令”的中间层。
 */
class MotorPosController final : public IController
{
public:
    /**
     * @brief 位置控制器配置
     */
    struct Config
    {
        PIDMotor::Config position_pid{};         ///< 位置环 PID 配置，用于“角度 -> 目标角速度”
        PIDMotor::Config velocity_pid{};         ///< 速度环 PID 配置，用于“目标角速度 -> 电流输出”
        uint32_t         pos_vel_freq_ratio = 1; ///< 位置环相对更新周期的分频，应 >= 1

        ControlMode ctrl_mode = ControlMode::Default; ///< 控制模式，Default 时跟随电机默认模式

        /**
         * 当驱动器支持内部位置指令时，发送频率相对于控制器 update 频率的分频。
         * 用于只需要较低频率维持控制指令，同时需要节约总线负载的情况
         *
         * 如果只能发送内部速度指令，则会降速度指令的频率为位置环更新频率
         */
        uint32_t internal_set_ratio = 10; ///< 应 >= 1
    };

    /**
     * @param motor 被控制的电机
     * @param cfg 控制器配置
     */
    MotorPosController(motors::IMotor* motor, const Config& cfg);
    ~MotorPosController() override;

    /**
     * @brief 周期更新控制器
     */
    void update() override;
    /**
     * @brief 设置位置参考
     * @param position 目标输出轴角度，单位 deg
     */
    void setRef(float position);

private:
    PIDMotor position_pid_; ///< 外层位置环
    PIDMotor velocity_pid_; ///< 内层速度环

    uint32_t pos_vel_prescaler_ = 0; ///< 位置环分频计数器

    float position_ref_ = 0.0f; ///< 当前位置参考，单位 deg

    uint32_t pos_vel_freq_ratio_; ///< 位置环相对 update 的分频比

    size_t   internal_set_prescaler_ = 0; ///< 内部指令发送分频计数器
    uint32_t internal_set_ratio_;         ///< 内部指令发送分频比
};

} // namespace controllers

#endif // MOTOR_POS_CONTROLLER_HPP
