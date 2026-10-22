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
    /// 析构前必须由宿主显式失能；控制器不拥有电机。
    ~MotorVelController() override = default;

    /**
     * @brief 周期更新控制器
     */
    void update() override;
    /**
     * @brief 从直接调用方设置速度参考，仅 Standalone 接受
     * @note Controlled / Disabled / Protected 均拒绝，拒绝不修改任何业务状态或输出。
     *       调用须与生命周期、update()、PID 访问及直接电机命令串行化。
     * @param velocity 目标输出轴速度，单位 rpm
     * @param max_current 外部 PID 模式下临时电流/力矩输出上限；
     *                   传 kNoLimit（默认）表示不施加额外限制
     * @param ff 前馈输入，量纲须匹配电机低层输入
     * @return true 仅表示接受参考，不代表发送受理或硬件执行；否则返回 false。
     */
    [[nodiscard]] bool setRef(float velocity, float max_current = kNoLimit, float ff = 0.0f);

    /**
     * @brief 从当前父节点设置速度参考
     * @note 仅自身 Controlled 且 currentController() == &source 时接受；其他情况拒绝，
     *       不修改任何业务状态或输出。不额外要求 source 已使能，允许父节点保护钩子提交安全参考。
     *       source 是调用方显式声明，可用公开 owner 指针冒充，并非强身份认证；
     *       不保护 raw IMotor / getMotor() 旁路。调用须与生命周期、update()、PID 访问
     *       及直接电机命令串行化。
     * @param source 声明的参考来源，必须为当前父节点
     * @param velocity 目标输出轴速度，单位 rpm
     * @param max_current 外部 PID 临时电流/力矩输出上限，kNoLimit 表示不施加额外限制
     * @param ff 前馈输入，量纲须匹配电机低层输入
     * @return true 仅表示接受参考，不代表发送受理或硬件执行；否则返回 false。
     */
    [[nodiscard]] bool setRefFrom(const core::control::ControllerNode& source, float velocity,
                                 float max_current = kNoLimit, float ff = 0.0f);

    /**
     * @brief 直接访问内部 PID 对象
     */
    auto& getPID() { return pid_; }

    static constexpr float kNoLimit = motors::IMotor::kNoLimit;

protected:
    bool selfEnable() override;
    void selfProtect() override;
    void selfDisable() override;

private:
    void applyReference(float velocity, float max_current, float ff);

    PIDMotor pid_;                    ///< 速度环 PID
    float    velocity_target_ = 0.0f; ///< 目标速度，默认单位 rpm
    float    ff_              = 0.0f; ///< 前馈输入

    float max_current_ = kNoLimit; ///< 电流/力矩输出限幅

    size_t internal_set_prescaler_ = 0; ///< 内部速度指令发送分频计数器
    size_t internal_set_ratio_;         ///< 内部速度指令发送分频比
};

} // namespace controllers

#endif // MOTOR_VEL_CONTROLLER_HPP
