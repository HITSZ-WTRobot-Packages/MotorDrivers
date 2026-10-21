/**
 * @file    dm.hpp
 * @author  syhanjin
 * @date    2026-02-27
 * @brief   达妙电机驱动
 */
#pragma once
#include "can_driver.hpp"
#include "motor_if.hpp"
#include "watchdog.hpp"

namespace motors
{

/**
 * @brief 达妙（DM）电机驱动对象
 *
 * DM 驱动支持多种控制模式：
 * - MIT
 * - 内部位置
 * - 内部速度
 *
 * 这类驱动对参数量程比较敏感，因此配置中的量程必须和上位机设定一致，
 * 否则反馈换算和指令打包都会出错。
 *
 * 当前库对 DM 还额外有两个明确约束：
 * - 所有 DM 电机共享同一个 `master_id`
 * - `id0` 建议使用 `0x09 ~ 0x0F`，以便避开 DJI 的标准 ID 区间
 *
 * 反馈通过 `bsp::can::Can` 的成员回调接收：构造时即向 `cfg.can` 注册
 * `on_can`，因此本对象必须在 `Can::start()` 之前构造，并存活到该总线不再使用
 * （can_driver2 没有注销接口）。对象禁止复制与移动，避免回调借用旧地址。
 *
 * Core enable() 成功后根节点为 Protected；直接控制需 standalone()。公开 set* 仅在
 * Standalone / Controlled 接受，不鉴别调用者。宿主须串行化生命周期、业务与周期发包，
 * Core 状态快照不是业务锁。析构前必须显式 disable()；本驱动不在析构时自动释放。
 */
class DMMotor : public IMotor
{
public:
    /**
     * @brief 已封装的 DM 电机类型
     */
    enum class Type
    {
        J4310_2EC,   ///< J4310-2EC
        J10010L_2EC, ///< J10010L-2EC
        S3519,       ///< S3519
        S2325_1EC,   ///< S2325-1EC + 3520-1EC(驱动器)
        J4340_2EC,   ///< J4340-2EC
        J8009P_2EC,  ///< J8009P-2EC

        MotorTypeCount, ///< 类型计数，占位用
    };
    /**
     * @brief DM 驱动器内部控制模式
     *
     * 这里不是 MCU 侧控制器模式，而是驱动器固件当前配置成哪种协议解释方式。
     * 发送时标准帧 ID 为“模式基值 + 电机 ID”：
     * - `Vel = 0x200 + ID`
     * - `Pos = 0x100 + ID`
     * - `MIT = 0x000 + ID`
     */
    enum class Mode
    {
        MIT = 0x000, ///< MIT 模式
        Pos = 0x100, ///< 内部位置模式
        Vel = 0x200, ///< 内部速度模式
    };
    /**
     * @brief 达妙反馈状态码
     */
    enum class DriverState : uint8_t
    {
        Disabled             = 0x0U, // 失能
        Enabled              = 0x1U, // 使能
        OverVoltage          = 0x8U, // 超压
        UnderVoltage         = 0x9U, // 欠压
        OverCurrent          = 0xAU, // 过电流
        MosOverheating       = 0xBU, // MOS 过温
        MotorCoilOverheating = 0xCU, // 电机线圈过温
        Disconnected         = 0xDU, // 通讯丢失
        Overload             = 0xEU, // 过载
    };

    /**
     * @brief 达妙电机配置
     */
    struct Config
    {
        bsp::can::Can& can;         ///< 所在 CAN 总线（借用，须比本对象长寿）
        uint8_t        id0;         ///< 电调 ID，仅低 4 位有效，建议从 0x09 到 0x0F
        Type           type;        ///< 电机类型
        Mode           mode;        ///< 驱动器控制模式，需要和上位机配置一致
        float          pos_max_rad; ///< 驱动器配置的位置最大值，单位 rad
        float          vel_max_rad; ///< 驱动器配置的速度最大值，单位 rad/s
        float          tor_max;     ///< 驱动器配置的力矩最大值，单位 Nm

        float default_angle_zero = 0; ///< 默认电机零点，理解为零点时电机编码器返回的角度

        bool  auto_zero      = false; ///< 上电收够一段稳定反馈后，是否自动把当前角度设为零点
        bool  reverse        = false; ///< 是否反转输出方向
        float reduction_rate = 1.0f;  ///< 外接减速比
        bool  can_fd_brs     = false; ///< true：CAN FD + BRS；false：Classic CAN，须匹配电调配置
    };

    explicit DMMotor(const Config& cfg);
    /// 析构前必须显式 disable；不注销 CAN 回调，对象须存活至总线不再使用。
    ~DMMotor() override = default;

    DMMotor(const DMMotor&)            = delete;
    DMMotor& operator=(const DMMotor&) = delete;
    DMMotor(DMMotor&&)                 = delete;
    DMMotor& operator=(DMMotor&&)      = delete;

    /**
     * @brief 获取输出轴角度
     * @return 相对软件零点、跨圈展开的输出轴角度，单位 deg
     */
    [[nodiscard]] float getAngle() const override { return abs_angle_; }
    /**
     * @brief 获取输出轴角速度
     * @return 速度；DM 对外接口单位按库约定使用 rpm
     */
    [[nodiscard]] float getVelocity() const override { return velocity_; }
    /**
     * @brief 获取反馈力矩
     * @return 力矩，单位 Nm
     */
    [[nodiscard]] float getTorque() const { return feedback_.torque; }
    /**
     * @brief 把当前输出角度设置为零点
     */
    void resetAngle() override;

    [[nodiscard]] controllers::ControlMode defaultControlMode() const override
    {
        switch (cfg_.mode)
        {
        case Mode::MIT:
            return controllers::ControlMode::InternalMIT;
        case Mode::Pos:
            return controllers::ControlMode::InternalPos;
        case Mode::Vel:
            return controllers::ControlMode::InternalVel;
        default:
            return controllers::ControlMode::ExternalPID;
        }
    }

    /**
     * @brief 解码一帧 DM 反馈
     */
    void decode(const uint8_t data[8]);

    [[nodiscard]] bool isConnected() const override { return watchdog_.isFed(); }

    [[nodiscard]] bool supportsCurrent() const override { return cfg_.mode == Mode::MIT; }

    [[nodiscard]] InternalVelocitySupportLevel internalVelocitySupportLevel() const override
    {
        return cfg_.mode == Mode::Vel ? InternalVelocitySupportLevel::RefOnly
                                      : InternalVelocitySupportLevel::NotSupported;
    }

    [[nodiscard]] InternalPositionSupportLevel internalPositionSupportLevel() const override
    {
        return cfg_.mode == Mode::Pos ? InternalPositionSupportLevel::RefAndMaxv
                                      : InternalPositionSupportLevel::NotSupported;
    }

    [[nodiscard]] bool supportsInternalMIT() const override { return cfg_.mode == Mode::MIT; }

    /**
     * @brief 初始化 DM 反馈的 CAN 滤波器
     *
     * 只接受发给主机（master）的标准帧：`id1 = master_id`、掩码 `0x7FF`；命中帧写入
     * FIFO0。RTR 位不参与掩码，因此硬件同时放行数据帧与远程帧，远程帧由 `on_can`
     * 丢弃（原实现靠掩码低半字一并排除）。
     *
     * @param can 目标总线（须在 `Can::start()` 之前调用）
     * @param filter_index 过滤器资源编号。can_driver2 中 bxCAN 为 filter bank 编号，
     *                     FDCAN 为所选标准/扩展过滤器数组内的 element 下标，两者硬件
     *                     布局不同，索引不可互换。
     * @param master_id 本机主控 ID，用于接收属于自己的反馈；当前库要求所有 DM 电机共用同一个值
     * @return can_driver2 状态码；调用方需自行检查
     */
    [[nodiscard]] static bsp::can::Status CAN_FilterInit(bsp::can::Can& can,
                                                         uint32_t       filter_index,
                                                         uint32_t       master_id);

    /** @brief 一收一回保活；Disabled 时仅发送失能协议包，不改变 Core 生命周期。 */
    void ping() const
    {
        if (!isEnabled() || !enable_requested_)
            sendDisableFrame();
        else if (feedback_.state == DriverState::Disabled)
            sendEnableFrame();
    }

    /**
     * @brief 最近一次驱动器反馈状态；不同于 Core 逻辑 state() 或软件使能请求标记
     */
    [[nodiscard]] DriverState driverState() const { return feedback_.state; }

protected:
    // 钩子绕过公开 set* 状态门控；Core 提交状态前也必须建立安全输出。
    /// 发送使能请求并乐观返回 true；不检查发送受理或等待反馈。
    bool selfEnable() override;
    /// 速度归零 / 位置保持 getAngle() 且沿用速度量程 / MIT 直接发送协议全零。
    void selfProtect() override;
    /// 发送协议失能并清除软件请求标记；不操作 Core 父子生命周期。
    void selfDisable() override;

    /**
     * @brief 低层输出接口
     *
     * 当 DM 运行在 MIT 模式时，这个接口会把 MIT 退化为力矩输入使用。
     * 因此这里的入参语义应理解为力矩，单位 Nm。
     * 位置、速度与增益直接使用协议零值，不应用软件位置坐标逆换算。
     */
    void applyCurrent(float current) override;
    /**
     * @brief 发送内部速度参考
     *
     * 库设计上对外统一使用 rpm；DM 驱动内部会把它换算成协议要求的 rad/s。
     * DM 速度模式协议不支持 maxi，该参数被忽略。
     */
    void applyInternalVelocity(float rpm, float maxi) override;
    /**
     * @brief 发送内部位置参考
     *
     * 参考与 getAngle() 使用相同的软件零点和展开输出轴坐标（deg）；
     * 恢复方向、减速比、软件零点与当前圈数后，换算成绝对协议位置（rad）。
     * maxv 单位 rpm，传 kNoLimit 时回退到 cfg_.vel_max_rad。
     */
    void applyInternalPosition(float pos, float maxv) override;
    /**
     * @brief 发送 MIT 控制指令
     *
     * - `p_ref` 为与 getAngle() 同软件零点、同跨圈展开坐标的输出轴位置，单位 deg。
     * - `v_ref` 为配套的输出轴速度参考，单位 deg/s，不是普通速度模式的 rpm。
     * - `t_ff` 使用 Nm；力矩方向与 kp/kd 的协议单位保持不变。
     *
     * 位置恢复软件零点、圈数、方向与减速比，速度恢复方向与减速比，
     * 换算为协议 rad / rad/s 后才按配置量程限幅。
     */
    void applyInternalMIT(float t_ff, float p_ref, float v_ref, float kp, float kd) override;

private:
    void sendEnableFrame() const;
    void sendDisableFrame() const;

    /**
     * @brief 将与 getAngle() 同零点、同展开坐标的输出轴位置（deg）逆换算为协议 rad。
     * @return 当前圈的绝对协议位置，不限幅
     */
    [[nodiscard]] float toProtocolPosition(float output_deg) const;

    /**
     * @brief 按原始协议坐标限幅、打包并发送 MIT，不再应用方向或软件位置逆换算。
     * @param t_ff 已处理方向的协议力矩，单位 Nm
     * @param p_ref 原始协议位置，单位 rad
     * @param v_ref 原始协议速度，单位 rad/s
     * @param kp 协议位置增益
     * @param kd 协议速度增益
     * @note 纯力矩与保护路径直接使用协议零位置、零速度、零增益。
     */
    void sendRawMIT(float t_ff, float p_ref, float v_ref, float kp, float kd);

    Config cfg_; ///< 构造时保存的配置，外接减速比归一化为正值（非正值按 1）

    bool enable_requested_{ false }; ///< 使能协议请求标记，不是 Core 状态或反馈运行状态

    uint32_t          feedback_count_ = 0; ///< 反馈计数，用于上电自动清零
    service::Watchdog watchdog_;
    struct
    {
        float angle{ 0 };      ///< 当前圈内角度，单位 deg
        float velocity{ 0 };   ///< 电机侧速度，库内统一按 rpm 保存
        float torque{ 0 };     ///< 反馈力矩，单位 Nm
        float temp_mos{ 0 };   ///< MOS 温度
        float temp_rotor{ 0 }; ///< 转子温度

        DriverState state{ DriverState::Disabled }; ///< 驱动器反馈状态码

        int32_t count{ 0 }; ///< 圈数累计，用于展开成连续角度
    } feedback_{};
    float angle_zero_{ 0 };    ///< 零点角度，单位 deg
    float inv_reduction_rate_; ///< 外接减速比倒数
    float pos_max_deg;         ///< 位置量程上限，单位 deg

    float abs_angle_ = 0; ///< 输出轴绝对角度，单位 deg
    float velocity_  = 0; ///< 输出轴速度，库内统一按 rpm 保存

    float sign_; ///< 方向符号，正转为 1，反转为 -1

    /**
     * @brief CAN 接收回调，由 `Can` 在 ISR 上下文广播调用
     *
     * 只消费本对象自己的合法反馈：标准 ID、Classic/FD 数据帧、8 字节负载，且
     * `data[0]` 低 4 位等于自身 `id0`。
     */
    void on_can(const bsp::can::FrameView& frame) noexcept;

    /**
     * @brief 生成发送报文头（Standard / Data，按配置选择 Classic 或 FD+BRS）
     */
    [[nodiscard]] bsp::can::FrameHeader tx_header() const noexcept;

    // DM 协议定义好的固定使能 / 失能指令。
    static constexpr uint8_t ENABLE_MSG[]  = { 0xFF, 0XFF, 0XFF, 0xFF, 0XFF, 0XFF, 0XFF, 0xFC };
    static constexpr uint8_t DISABLE_MSG[] = { 0xFF, 0XFF, 0XFF, 0xFF, 0XFF, 0XFF, 0XFF, 0xFD };
};

} // namespace motors
