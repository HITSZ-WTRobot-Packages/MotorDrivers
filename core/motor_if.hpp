/**
 * @file    motor_if.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   电机抽象接口与控制器抽象接口
 *
 * 这个文件定义了 MotorDrivers 最核心的三件东西：
 * - `motors::IMotor`：所有电机驱动共同遵守的接口
 * - `controllers::ControlMode`：控制链路应该如何闭环
 * - `controllers::IController`：控制器基类
 *
 * 设计上，`IMotor` 不直接按“某个具体电机型号”建模，而是按“能力”建模：
 * - 反馈能力：获取角度、速度、连接状态，重置零点
 * - 控制输入能力：电流 / 力矩、内部速度、内部位置、MIT
 * - 控制权能力：申请、释放、查询当前控制器
 *
 * 单位约定：
 * - 角度默认使用 deg
 * - 速度默认使用 rpm，除非变量名明确带有 `_rad`、`_rps` 等后缀
 * - 力矩默认使用 Nm
 * - `resetAngle()` 的含义是“把当前输出角度重新视作零点”
 */
#ifndef I_MOTOR_HPP
#define I_MOTOR_HPP
#include <cassert>

namespace motors
{
class IMotor;
} // namespace motors

namespace controllers
{
/**
 * @brief 控制器希望电机使用的控制方式
 *
 * 这里描述的是“控制链路闭合在哪里”。
 * 例如 `ExternalPID` 表示在 MCU 里自己做闭环，`InternalVel` 表示把速度参考交给驱动器内部。
 */
enum class ControlMode
{
    Default,        ///< 使用电机驱动声明的默认控制模式
    ExternalPID,    ///< 外部控制器计算输出，再通过 `setCurrent()` 下发
    InternalVel,    ///< 使用驱动器内部速度环
    InternalPos,    ///< 仅支持发送位置指令，不支持发送速度指令
    InternalVelPos, ///< 同时支持发送速度指令与位置指令
    InternalMIT,    ///< 使用 MIT 格式一并发送位置、速度、刚度、阻尼和前馈
};

class IController;
} // namespace controllers

namespace motors
{
/**
 * @brief 电机抽象接口
 *
 * 驱动层负责把具体电机与驱动接入方式包装成这组接口。上层控制器不需要知道“这个电机到底是
 * DJI、DM、VESC，还是以后接入的其他类型驱动”，也不需要关心它走的是哪种总线，只需要关心它
 * 支持哪些能力，以及角度、速度等反馈是什么。
 *
 * 这套接口故意把控制输入拆成四种，是为了明确区分控制深度：
 * - `setCurrent()`：最低层的电流 / 力矩输入
 * - `setInternalVelocity()`：驱动器的速度指令输入
 * - `setInternalPosition()`：驱动器的位置指令输入
 * - `setInternalMIT()`：一次发送更完整的 MIT 控制量
 */
class IMotor
{
public:
    IMotor() : controller_(nullptr) {}
    virtual ~IMotor() = default;

    /**
     * @brief 返回该电机推荐的默认控制模式
     *
     * 当控制器配置使用 `ControlMode::Default` 时，会退回到这里。
     */
    [[nodiscard]] virtual controllers::ControlMode defaultControlMode() const
    {
        return controllers::ControlMode::ExternalPID;
    }

    /**
     * @brief 获取输出轴绝对角度
     * @return 角度，单位 deg
     */
    [[nodiscard]] virtual float getAngle() const = 0;
    /**
     * @brief 获取输出轴速度
     * @return 速度，单位 rpm
     */
    [[nodiscard]] virtual float getVelocity() const = 0;
    /**
     * @brief 把当前反馈角度重新定义为零点
     */
    virtual void resetAngle() = 0;

    /**
     * @brief 电机是否连接
     *
     * 通过 watchdog 实现，超时时间默认为 10 ms
     */
    [[nodiscard]] virtual bool isConnected() const { return false; }

    /**
     * @brief 是否支持通过 `setCurrent()` 下发输出
     *
     * 这不是在判断“有没有这个成员函数”，而是在判断：
     * 当前这台电机、当前这份配置下，是否允许用户把它当作最低层电流 / 力矩输入来使用。
     *
     * 注意不同驱动对“current”的具体物理意义可能不同。
     * 例如 DM 在 MIT 模式下会把它退化成力矩输入使用。
     */
    [[nodiscard]] virtual bool supportsCurrent() const { return false; }
    /**
     * @brief 下发最低层电流 / 力矩类输出
     * @param current 输出值，具体物理意义由驱动决定
     */
    virtual void setCurrent(const float current) { (void)current; }

    /**
     * @brief 是否支持内部速度控制
     */
    [[nodiscard]] virtual bool supportsInternalVelocity() const { return false; }
    /**
     * @brief 向驱动器发送内部速度指令
     * @param rpm 速度参考；默认单位 rpm，除非变量名或驱动注释明确说明
     */
    virtual void setInternalVelocity(const float rpm) { (void)rpm; }

    /**
     * @brief 是否支持内部位置控制
     */
    [[nodiscard]] virtual bool supportsInternalPosition() const { return false; }
    /**
     * @brief 向驱动器发送内部位置指令
     * @param pos 位置参考，默认单位 deg
     */
    virtual void setInternalPosition(const float pos) { (void)pos; }

    /**
     * @brief 是否支持 MIT 控制格式
     */
    [[nodiscard]] virtual bool supportsInternalMIT() const { return false; }
    /**
     * @brief 发送 MIT 控制指令
     *
     * 这类接口用来表达“单次同时给位置、速度、刚度、阻尼、前馈”的高级输入能力。
     *
     * 注意这里的 `v_ref` 不一定等同于普通 `setInternalVelocity()` 里的速度参考。
     * 在 MIT / 阻抗控制语义下，`v_ref` 往往是和 `p_ref` 配套使用的速度项，物理意义更接近
     * “位置参考的导数”。因此它的单位由具体驱动决定，特殊情况会在驱动注释里明确说明。
     */
    virtual void setInternalMIT(float t_ff, float p_ref, float v_ref, float kp, float kd) {}

    /**
     * @brief 尝试让控制器获取该电机的控制权
     *
     * 一个电机同一时刻只允许被一个控制器接管，用来避免多个控制器同时写目标值。
     */
    virtual bool tryAcquireController(controllers::IController* ctrl)
    {
        if (controller_ == nullptr)
        {
            controller_ = ctrl;
            return true;
        }
        return controller_ == ctrl; // re-acquire allowed
    }

    /**
     * @brief 释放控制权
     */
    virtual void releaseController(controllers::IController* ctrl)
    {
        if (controller_ == ctrl)
            controller_ = nullptr;
    }

    /**
     * @brief 当前持有该电机控制权的控制器
     */
    [[nodiscard]] controllers::IController* currentController() const { return controller_; }

private:
    controllers::IController* controller_; ///< 当前持有控制权的控制器
};
} // namespace motors

namespace controllers
{

/**
 * @brief 控制器抽象基类
 *
 * 派生类只需要关心：
 * - 如何根据反馈计算输出
 * - 何时调用对应的电机接口发送命令
 *
 * 控制权申请、启停状态这些通用逻辑在基类里统一处理。
 */
class IController
{
public:
    virtual ~IController() = default;

    /**
     * @brief 周期更新控制器
     *
     * 通常在固定频率任务中调用，例如 1 kHz 控制任务。
     */
    virtual void update() = 0;

    /**
     * @brief 启用控制器，并尝试获取电机控制权
     * @return 启用后是否成功持有电机
     */
    virtual bool enable()
    {
        if (!enabled_)
        {
            if (motor_)
            {
                if (motor_->tryAcquireController(this))
                    enabled_ = true;
            }
            else
            {
                enabled_ = false;
            }
        }
        return enabled_;
    }

    /**
     * @brief 停用控制器，并释放已持有的电机
     */
    virtual void disable()
    {
        if (enabled_)
        {
            if (motor_)
                motor_->releaseController(this);
            enabled_ = false;
        }
    }
    /**
     * @brief 控制器当前是否处于启用状态
     */
    [[nodiscard]] bool enabled() const { return enabled_; }

    /**
     * @brief 取得绑定的电机对象
     *
     * 用于直接从控制器读取电机相应数据，比如获取当前输出角度，输出速度等
     */
    [[nodiscard]] motors::IMotor* getMotor() const { return motor_; }

protected:
    IController(motors::IMotor* motor, const ControlMode ctrl_mode) : motor_(motor), enabled_(false)
    {
        // 解析最终控制模式：
        // - 用户选 Default：使用电机驱动自己的默认模式
        // - 用户显式指定：要求驱动明确声明支持该能力
        if (motor_ == nullptr)
        {
            ctrl_mode_ = ControlMode::ExternalPID;
            return;
        }
        if (ctrl_mode == ControlMode::Default)
        {
            ctrl_mode_ = motor_->defaultControlMode();
        }
        else
        {
            switch (ctrl_mode)
            {
            case ControlMode::ExternalPID:
                assert(motor_->supportsCurrent());
                ctrl_mode_ = ControlMode::ExternalPID;
                break;
            case ControlMode::InternalVel:
                // 我们充分相信用户能决定好控制模式，如果用户决定的不对，就应该报错，而不是兼容
                assert(motor_->supportsInternalVelocity());
                ctrl_mode_ = ControlMode::InternalVel;
                break;
            case ControlMode::InternalVelPos:
                // TODO: fixbug: 按当前语义，InternalVelPos 应同时要求支持速度指令和位置指令；
                // 这里暂时只检查了位置能力，按要求先补注释，不改原逻辑。
                assert(motor_->supportsInternalPosition());
                ctrl_mode_ = ControlMode::InternalVelPos;
                break;
            case ControlMode::InternalPos:
                assert(motor_->supportsInternalPosition());
                ctrl_mode_ = ControlMode::InternalPos;
                break;
            case ControlMode::InternalMIT:
                // MIT 在速度环 & 位置环 时会自动退化为 ExternalPID
                assert(motor_->supportsInternalMIT());
                ctrl_mode_ = ControlMode::InternalMIT;
                break;
            default:;
            }
        }
    }
    motors::IMotor* motor_;     ///< 绑定的电机对象
    ControlMode     ctrl_mode_; ///< 解析后的最终控制模式

private:
    bool enabled_; ///< 控制器是否已经成功持有电机
};

} // namespace controllers

#endif // I_MOTOR_HPP
