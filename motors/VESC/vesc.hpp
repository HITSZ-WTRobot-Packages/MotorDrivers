/**
 * @file    vesc.hpp
 * @author  syhanjin
 * @date    2026-03-06
 * @brief   VESC 电机驱动封装
 */
#pragma once

#include "can_driver.hpp"
#include "motor_if.hpp"
#include "watchdog.hpp"

namespace motors
{
/**
 * @brief VESC 电调对象
 *
 * 当前封装重点覆盖：
 * - 电流控制
 * - 内部速度控制
 * - 常见状态反馈解析
 *
 * 从能力模型上看，VESC 这一路主要提供：
 * - 角度反馈
 * - 速度反馈
 * - 电流输入
 * - 内部速度输入
 *
 * 当前 VESC 实现里，速度接口单位为 rpm。
 *
 * 生命周期：对象在构造时把成员回调注册到 `bsp::can::Can`，因此必须在对应总线的
 * `Can::start()` 之前构造，并保证对象比总线的使用期更长——本类不提供注销回调的
 * 接口。为避免借用中的回调指向已失效地址，本类既不可复制也不可移动。
 */
class VESCMotor : public IMotor
{
public:
    /**
     * VESC CAN 指令集（设置类）
     * 对应 VESC_CAN_PocketSet_t
     */
    enum class SetCommand : uint8_t
    {
        SetDuty            = 0U,  ///< 设置占空比, Data: Duty Circle * 100,000 (int32)
        SetCurrent         = 1U,  ///< 设置电流, Data: current * 1000 (int32)
        SetCurrentBrake    = 2U,  ///< 设置刹车电流, Data: current * 1000 (int32)
        SetRPM             = 3U,  ///< 设置电角转速, Data: ERPM (int32)
        SetPosition        = 4U,  ///< 设置位置, Data: pos * 1,000,000 (int32)
        SetCurrentRel      = 10U, ///< 设置相对电流，Data: ratio (-1 to 1) * 100,000 (int32)
        SetCurrentBrakeRel = 11U, ///< 设置相对刹车电流，Data: ratio (-1 to 1) * 100,100 (int32)
    };

    /**
     * VESC CAN 状态反馈指令集
     * 对应 VESC_CAN_PocketStatus_t
     *
     * 下面各状态包的字段说明，按“当前公开资料 + 本项目解码实现”整理。
     * 如果注释里写了“推测”，表示该字段含义或物理单位在当前资料下并非完全确认。
     */
    enum class StatusCommand : uint8_t
    {
        /**
         * 状态包 1
         *
         * 数据布局：
         * - Data[0..3]：ERPM（Electrical RPM，电角转速），`int32`
         * - Data[4..5]：总电流 * 10，`int16`
         *   当前按安培 A 解释
         * - Data[6..7]：占空比 * 1000，`int16`
         *   即最新 duty cycle，理论范围约为 `-1 ~ 1`
         */
        VESC_CAN_STATUS_1 = 9U,

        /**
         * 状态包 2
         *
         * 数据布局：
         * - Data[0..3]：累计放电安时 `Amp Hours * 10000`，`int32`
         * - Data[4..7]：累计回充安时 `Amp Hours Charged * 10000`，`int32`
         */
        VESC_CAN_STATUS_2 = 14U,
        /**
         * 状态包 3
         *
         * 数据布局：
         * - Data[0..3]：累计放电瓦时 `Watt Hours * 10000`，`int32`
         * - Data[4..7]：累计回充瓦时 `Watt Hours Charged * 10000`，`int32`
         */
        VESC_CAN_STATUS_3 = 15U,
        /**
         * 状态包 4
         *
         * 数据布局：
         * - Data[0..1]：FET 温度 * 10，`int16`
         *   单位按摄氏度 `degC` 推测
         * - Data[2..3]：电机温度 * 10，`int16`
         *   单位按摄氏度 `degC` 推测
         * - Data[4..5]：输入电流 * 10，`int16`
         *   单位按安培 A 推测
         * - Data[6..7]：PID 位置量 * 50，`int16`
         *   当前项目把它当作 0~360 deg 单圈位置来使用，但该字段的原始含义与单位仍带推测成分
         */
        VESC_CAN_STATUS_4 = 16U,
        /**
         * 状态包 5
         *
         * 数据布局（协议只定义 6 字节负载，不存在 Data[6..7]）：
         * - Data[0..3]：转速计原始值 `Tachometer Value`，`int32`
         *   当前项目仅按原始计数保存，其精确物理含义仍带推测成分
         * - Data[4..5]：输入电压 * 10，`int16`
         *
         * 接收时接受协议 DLC 6，也兼容 DLC 7/8 的补齐帧；多出的字节不会被读取。
         */
        VESC_CAN_STATUS_5 = 27U, ///< 输入电压与转速计原始值（协议 6 字节负载）
    };

    /**
     * VESC 配置结构体
     * 对应 VESC_Config_t
     */
    struct Config
    {
        bsp::can::Can& can; ///< 所在 CAN 总线，须比本对象长寿且在 Can::start() 前构造本对象
        uint8_t        id;  ///< 控制器 id，0xFF 代表广播
        /**
         * 电机极对数，用于机械 rpm 与 ERPM 的换算。
         *
         * 注意这里填写的是“极对数”，不是总极数。
         * 例如 14 极电机，这里应填写 7。
         *
         * 字段名 `electrodes` 沿用历史命名，当前不改动接口以兼容现有代码。
         */
        uint8_t electrodes;
        float   reduction_rate = 1.0f; ///< 外接减速比（VESC 上位机设置的减速比不生效）

        bool auto_zero = true;  ///< 上电收够一段稳定反馈后，是否自动把当前角度设为零点
        bool reverse   = false; ///< 是否反转输出方向
    };

    explicit VESCMotor(const Config& cfg);
    /// 析构不注销回调：总线的回调表在 start() 后固定，对象须活到总线不再使用。
    ~VESCMotor() override = default;

    VESCMotor(const VESCMotor&)            = delete;
    VESCMotor& operator=(const VESCMotor&) = delete;
    VESCMotor(VESCMotor&&)                 = delete;
    VESCMotor& operator=(VESCMotor&&)      = delete;

    /**
     * @brief 获取输出轴角度
     * @return 角度，单位 deg
     */
    [[nodiscard]] float getAngle() const override { return abs_angle_; }
    /**
     * @brief 获取输出轴速度
     * @return 速度；当前 VESC 实现单位为 rpm
     */
    [[nodiscard]] float getVelocity() const override { return velocity_; }
    void                resetAngle() override;

    [[nodiscard]] controllers::ControlMode defaultControlMode() const override
    {
        return controllers::ControlMode::ExternalPID;
    }

    [[nodiscard]] bool isConnected() const override { return watchdog_.isFed(); }

    [[nodiscard]] bool supportsCurrent() const override { return true; }
    void               setCurrent(float current) override;

    [[nodiscard]] InternalVelocitySupportLevel internalVelocitySupportLevel() const override
    {
        return InternalVelocitySupportLevel::RefOnly;
    }
    /**
     * @brief 发送内部速度参考
     *
     * VESC 内部速度模式不支持 maxi，该参数被忽略。
     */
    void setInternalVelocity(float rpm, float maxi) override;

    /**
     * @brief 是否支持内部位置控制
     *
     * 协议本身存在位置设定命令，但当前这个类没有把它作为稳定接口对外开放。
     *
     * 不重写 internalPositionSupportLevel()，继承基类默认 NotSupported。
     */

    /**
     * @brief 初始化 CAN 滤波器
     * @param can 所属 CAN 总线，须在 Can::start() 之前调用
     * @param filter_index 过滤器资源下标。can_driver2 的 index 含义随后端而定：
     *        bxCAN 为 filter bank 编号，FDCAN 为对应 ID 类型过滤器数组内的 element
     *        下标，两者硬件布局不同、不可互换，沿用调用方原有的分配约定。
     * @return 配置结果；非 Ok 由调用方处理
     *
     * VESC 只使用扩展帧，这里配置为掩码为 0 的扩展 ID 掩码过滤器：不比较任何标识符
     * 位，等价于“接受全部扩展帧（数据帧与远程帧）”，命中后进入 FIFO0；与旧 can_driver
     * 版本“32 位掩码、仅比较 IDE 位、FIFO0”的接受语义一致。
     */
    [[nodiscard]] static bsp::can::Status CAN_FilterInit(bsp::can::Can& can, uint32_t filter_index);

private:
    // 参数范围限制，对应 VESC_SET_*_MAX。
    static constexpr float kSetDutyMax            = 1.0f;
    static constexpr float kSetCurrentMax         = 2.0e6f;
    static constexpr float kSetCurrentBrakeMax    = 2.0e6f;
    static constexpr float kSetRPMMax             = 2.0e4f;
    static constexpr float kSetPositionMax        = 360.0f;
    static constexpr float kSetCurrentRelMax      = 1.0f;
    static constexpr float kSetCurrentBrakeRelMax = 1.0f;

    Config cfg_; ///< 构造时保存的配置

    uint32_t          feedback_count_ = 0; ///< 反馈计数，用于上电自动清零
    service::Watchdog watchdog_;
    struct
    {
        float erpm{ 0.0f };          ///< 电角转速 ERPM，来自状态包 1
        float pos{ 0.0f };           ///< 单圈位置，当前项目按 0~360 deg 使用，来自状态包 4
        float duty{ 0.0f };          ///< 占空比，来自状态包 1
        float current_motor{ 0.0f }; ///< 电机电流，来自状态包 1
        float current_in{ 0.0f };    ///< 输入电流，来自状态包 4，单位按 A 推测

        float amp_hours{ 0.0f };          ///< 累计放电安时
        float amp_hours_charged{ 0.0f };  ///< 累计回充安时
        float watt_hours{ 0.0f };         ///< 累计放电瓦时
        float watt_hours_charged{ 0.0f }; ///< 累计回充瓦时

        float motor_temperature{ 0.0f }; ///< 电机温度，来自状态包 4，单位按 degC 推测
        float mos_temperature{ 0.0f };   ///< MOSFET 温度，来自状态包 4，单位按 degC 推测

        float vin{ 0.0f };              ///< 输入电压
        float tachometer_value{ 0.0f }; ///< 原始转速计值，来自状态包 5，精确物理含义仍待确认

        int32_t round_cnt{ 0 }; ///< 圈数统计
    } feedback_{};

    float angle_zero_ = 0.0f; ///< 零点角度，单位 deg

    float sign_;               ///< 方向符号，正转为 1，反转为 -1
    float erpm2velocity_;      ///< ERPM 到输出轴机械 rpm 的换算系数
    float inv_reduction_rate_; ///< 外接减速比倒数

    float abs_angle_ = 0.0f; ///< 输出轴绝对角度，单位 deg
    float velocity_  = 0.0f; ///< 输出轴角速度，当前实现单位 rpm

    /**
     * @brief 总线接收回调，构造时注册到 `cfg_.can`
     * @param frame 一帧只读视图；仅在回调内有效，ISR 上下文同步调用
     *
     * 只处理属于本对象的、受支持的状态反馈帧：扩展帧、经典数据帧，且扩展 ID 低字节
     * 等于 `Config::id`。
     *
     * 负载长度按状态包分别校验：状态 1~4 需要 8 字节；状态 5 协议的负载为 6 字节，
     * 因此还接受 DLC 7/8 的补齐帧（多出的字节不会被读取）。长度不足、超过经典 CAN
     * 的 8 字节上限，或状态号不受支持的帧一律丢弃：既不进入解码，也不喂狗。
     */
    void on_can(const bsp::can::FrameView& frame) noexcept;

    /**
     * @brief 解码状态反馈
     * @param status_cmd 状态指令；调用方须已确认其属于本实现支持的状态包
     * @param data 数据指针；调用方须保证其长度覆盖该状态包用到的字段
     *             （状态 1~4 为 8 字节，状态 5 为 6 字节）
     *
     * 只在调用方完成负载长度校验后调用，本函数自身不做边界检查。
     */
    void decode(StatusCommand status_cmd, const uint8_t* data);

    /**
     * @brief 限幅
     * @param value 输入值
     * @param abs_max 最大绝对值
     * @return 限幅后的值
     */
    static float clamp(float value, float abs_max);
    /**
     * @brief 发送设置指令
     * @param cmd 指令
     * @param value 值
     */
    void sendSetCommand(SetCommand cmd, float value) const;
};
} // namespace motors
