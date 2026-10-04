/*  
 * @file    motor_mit_controller.hpp
 * @author  Fang_Shuohan
 * @date    2026-10-03
 * @brief   通用mit模式控制器实现
 *
 * 本控制器适合需要位置和速度联合调节的场景，使用MIT风格的PD控制律。
 * 本控制器适配两种模式：InternalMIT和ExternalPID
 * 不同于其他内部模式，InternalMIT模式下也需要传入kp和kd参数
 * ExternalPID模式下，输出单位由不同的电机驱动决定，如DM电机驱动的输出单位为Nm,而像其他电机驱动的输出单位可能不同。
 * 需要关注单位问题：本控制器需要传入的参考值和DM电机的InternalMIT模式保持一致，单位为deg和deg/s
 *  
 */

#ifndef MOTOR_MIT_CONTROLLER_HPP
#define MOTOR_MIT_CONTROLLER_HPP

#include "motor_if.hpp"
#include "mit_pd.hpp"
#include <cstdint>

namespace controllers
{
class MotorMITController final : public IController
{
public:
    struct Config
    {
        // 增益按 rad / rad/s 定义。
        // InternalMIT: Kp [Nm/rad]、Kd [Nm/(rad/s)]，abs_output_max 无效。
        // ExternalPID: 输出单位由 setCurrent() 决定，abs_output_max 限制总输出。
        MITPD::Config mit{};
        ControlMode ctrl_mode = ControlMode::Default; ///< 显式选择 InternalMIT/ExternalPID；Default 跟随电机默认模式
        uint32_t internal_set_ratio = 1; ///< 内部 MIT 指令发送分频，0 按 1 处理
    };

    MotorMITController(motors::IMotor* motor, const Config& cfg);
    ~MotorMITController() override;

    bool enable() override;
    void update() override;

    /**
     * @param position_deg 位置参考，单位 deg
     * @param velocity_degs 速度参考，单位 deg/s；保持位置时传 0
     * @param max_current 外部模式额外输出上限；kNoLimit 表示不额外限制。
     *                    内部模式忽略，不能用它限制驱动器内部 PD 的总力矩。
     * @param t_ff 前馈；内部模式为 Nm，外部模式与 setCurrent() 输出单位一致
     *
     * 未启用时只保存参考。启用后的内部模式立即发送，并重置分频计数。
     * 内部路径要求驱动 setInternalMIT() 接收 deg、deg/s 和按弧度定义的增益。
     * 所有调用应在同一个控制任务中串行执行。
     */
    void setRef(float position_deg, float velocity_degs,
                float max_current = kNoLimit, float t_ff = 0.0f);

    static constexpr float kNoLimit = motors::IMotor::kNoLimit;

private:
    MITPD mit_;
    float kp_;
    float kd_;
    float position_ref_ = 0.0f;
    float velocity_ref_ = 0.0f;
    float max_current_ = kNoLimit;
    float t_ff_ = 0.0f;
    uint32_t internal_set_prescaler_ = 0;
    uint32_t internal_set_ratio_;
};
} // namespace controllers

#endif // MOTOR_MIT_CONTROLLER_HPP
