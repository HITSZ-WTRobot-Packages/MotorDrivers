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
 * - 生命周期能力：由 Core 管理保护态、直接控制态和父控制器关系
 *
 * 单位约定：
 * - 角度默认使用 deg
 * - 速度默认使用 rpm，除非变量名明确带有 `_rad`、`_rps` 等后缀
 * - 力矩默认使用 Nm
 * - `resetAngle()` 的含义是“把当前输出角度重新视作零点”
 *
 * Core 的状态/关系查询只是瞬时快照，不保证业务调用并发安全。宿主必须串行化生命周期操作、
 * 业务命令和周期发包；不能把控制关系当作线程同步或硬件安全联锁。
 * 所有电机和控制器销毁前必须显式完成 disable；析构不会自动释放控制关系。
 */
#ifndef I_MOTOR_HPP
#define I_MOTOR_HPP
#include "IController.hpp"

#include <array>
#include <cassert>
#include <limits>

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
 *
 * 公开命令只在 Standalone 或 Controlled 状态下转发给驱动，不识别调用者来源；
 * Disabled 和 Protected 状态下忽略命令。不支持的能力仍由默认 apply 实现忽略。
 * 生命周期钩子必须直接调用 apply 或内部协议 helper，不能经过公开权限包装，
 * 因为钩子执行时 Core 可能尚未提交新状态。
 */
class IMotor : public core::control::IController<0>
{
public:
    IMotor() = default;
    ~IMotor() override = default;

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
    void setCurrent(const float current)
    {
        const auto current_state = state();
        if (current_state == State::Standalone || current_state == State::Controlled)
            applyCurrent(current);
    }

    /**
     * 内部速度控制能力等级
     */
    enum class InternalVelocitySupportLevel
    {
        NotSupported, /// 不支持该控制模式
        RefOnly,      /// 只支持设置目标速度
        RefAndMaxi,   /// 支持设置目标速度和最大电流/力矩
    };

    /**
     * @brief 查询内部速度模式支持等级
     */
    [[nodiscard]] virtual InternalVelocitySupportLevel internalVelocitySupportLevel() const
    {
        return InternalVelocitySupportLevel::NotSupported;
    }

    /**
     * @brief 向驱动器发送内部速度指令
     * @param rpm 速度参考；默认单位 rpm
     * @param maxi 最大电流/力矩限制，传 kNoLimit 表示不限制；
     *             不支持 maxi 的电机会忽略该参数
     */
    void setInternalVelocity(const float rpm, const float maxi)
    {
        const auto current_state = state();
        if (current_state == State::Standalone || current_state == State::Controlled)
            applyInternalVelocity(rpm, maxi);
    }

    /**
     * 内部位置模式能力等级
     */
    enum class InternalPositionSupportLevel
    {
        NotSupported,
        RefOnly,
        RefAndMaxv,
    };

    /**
     * @brief 查询内部位置模式支持等级
     */
    [[nodiscard]] virtual InternalPositionSupportLevel internalPositionSupportLevel() const
    {
        return InternalPositionSupportLevel::NotSupported;
    }
    /**
     * @brief 向驱动器发送内部位置指令
     * @param pos 位置参考，默认单位 deg
     * @param maxv 最大速度限制，单位 rpm，传 kNoLimit 表示不限制；
     *             不支持 maxv 的电机会忽略该参数
     */
    void setInternalPosition(const float pos, const float maxv)
    {
        const auto current_state = state();
        if (current_state == State::Standalone || current_state == State::Controlled)
            applyInternalPosition(pos, maxv);
    }

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
    void setInternalMIT(float t_ff, float p_ref, float v_ref, float kp, float kd)
    {
        const auto current_state = state();
        if (current_state == State::Standalone || current_state == State::Controlled)
            applyInternalMIT(t_ff, p_ref, v_ref, kp, kd);
    }

    static constexpr float kNoLimit = std::numeric_limits<float>::max();

protected:
    /** @brief 实际下发电流 / 力矩输出；参数单位与 setCurrent() 相同。 */
    virtual void applyCurrent(const float current) { (void)current; }

    /** @brief 实际下发内部速度指令；参数顺序和单位与 setInternalVelocity() 相同。 */
    virtual void applyInternalVelocity(const float rpm, const float maxi)
    {
        (void)rpm;
        (void)maxi;
    }

    /** @brief 实际下发内部位置指令；参数顺序和单位与 setInternalPosition() 相同。 */
    virtual void applyInternalPosition(const float pos, const float maxv)
    {
        (void)pos;
        (void)maxv;
    }

    /** @brief 实际下发 MIT 指令；参数顺序和单位与 setInternalMIT() 相同。 */
    virtual void applyInternalMIT(float t_ff, float p_ref, float v_ref, float kp, float kd)
    {
        (void)t_ff;
        (void)p_ref;
        (void)v_ref;
        (void)kp;
        (void)kd;
    }

    /**
     * @brief 直接建立驱动的安全保护输出，不释放控制关系或失能子节点。
     * @note 必须遵守 Core self* 钩子约定，绕过公开命令的状态门禁。
     */
    void selfProtect() override = 0;

    /**
     * @brief 直接提交驱动停机并清理自身资源，不依赖已提交的逻辑状态。
     * @note 必须遵守 Core self* 钩子约定；不得重入生命周期 API。
     */
    void selfDisable() override = 0;
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
 * Core 统一管理生命周期及控制关系；唯一候选子节点是构造时绑定的电机，必须非空。
 * enable() 成功后根控制器处于 Protected，需要 standalone() 后才能通过 setRef() 接受参考；
 * Controlled 状态只接受 setRefFrom() 显式声明的当前父节点来源，不鉴别真实调用者身份。
 * Protected 状态下允许 update() 继续执行派生类已建立的安全参考。
 * 派生类 selfProtect() 应清理参考、PID 和分频状态并立即建立安全输出；selfDisable()
 * 只清理自身业务状态，不操作电机生命周期或控制关系。钩子不得重入生命周期 API。
 */
class IController : public core::control::IController<1>
{
public:
    ~IController() override = default;

    /**
     * @brief 周期更新控制器
     *
     * 通常在固定频率任务中调用，例如 1 kHz 控制任务。
     */
    virtual void update() = 0;

    /**
     * @brief 取得绑定的电机对象
     *
     * 用于直接从控制器读取电机相应数据，比如获取当前输出角度，输出速度等
     */
    [[nodiscard]] motors::IMotor* getMotor() const { return motor_; }

protected:
    /**
     * @param motor 借用的非空电机；必须比本控制器存活更久，Core 拒绝空候选子节点。
     * @param ctrl_mode 控制模式；Default 沿用驱动声明的默认模式。
     */
    IController(motors::IMotor* motor, const ControlMode ctrl_mode)
        : core::control::IController<1>(std::array<core::control::ControllerNode*, 1>{ motor }),
          motor_(motor)
    {
        // 解析最终控制模式：
        // - 用户选 Default：使用电机驱动自己的默认模式
        // - 用户显式指定：要求驱动明确声明支持该能力
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
                assert(motor_->internalVelocitySupportLevel() !=
                       motors::IMotor::InternalVelocitySupportLevel::NotSupported);
                ctrl_mode_ = ControlMode::InternalVel;
                break;
            case ControlMode::InternalVelPos:
                assert(motor_->internalPositionSupportLevel() !=
                       motors::IMotor::InternalPositionSupportLevel::NotSupported);
                assert(motor_->internalVelocitySupportLevel() !=
                       motors::IMotor::InternalVelocitySupportLevel::NotSupported);
                ctrl_mode_ = ControlMode::InternalVelPos;
                break;
            case ControlMode::InternalPos:
                assert(motor_->internalPositionSupportLevel() !=
                       motors::IMotor::InternalPositionSupportLevel::NotSupported);
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

    /** @brief 直接参考仅在 Standalone 接受；children 的持有关系由 Core 保证。 */
    [[nodiscard]] bool canAcceptReference() const noexcept
    {
        return isStandalone();
    }

    /** @brief 受控参考仅接受显式声明的当前父节点，不检查电机持有关系或来源节点状态。 */
    [[nodiscard]] bool canAcceptReferenceFrom(
            const core::control::ControllerNode& source) const noexcept
    {
        return isControlled() && currentController() == &source;
    }


    motors::IMotor* motor_;     ///< 绑定的电机对象
    ControlMode     ctrl_mode_; ///< 解析后的最终控制模式
};

} // namespace controllers

#endif // I_MOTOR_HPP
