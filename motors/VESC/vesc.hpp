/**
 * @file    vesc.hpp
 * @author  syhanjin
 * @date    2026-03-06
 * @brief   VESC 电机驱动封装
 */
#pragma once

#include "can.h"
#include "motor_if.hpp"
#include "watchdog.hpp"

#ifndef MOTORS_VESC_MAX_NUM
#    define MOTORS_VESC_MAX_NUM (16)
#endif

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
         * 数据布局：
         * - Data[0..3]：转速计原始值 `Tachometer Value`，`int32`
         *   当前项目仅按原始计数保存，其精确物理含义仍带推测成分
         * - Data[4..5]：输入电压 * 10，`int16`
         * - Data[6..7]：保留字段
         */
        VESC_CAN_STATUS_5 = 27U, ///< 输入电压与转速计原始值
    };

    /**
     * VESC 配置结构体
     * 对应 VESC_Config_t
     */
    struct Config
    {
        CAN_HandleTypeDef* hcan; ///< 所在 CAN 总线
        uint8_t            id;   ///< 控制器 id，0xFF 代表广播
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
    ~VESCMotor() override;

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

    [[nodiscard]] bool supportsInternalVelocity() const override { return true; }
    void               setInternalVelocity(float rpm) override;

    /**
     * @brief 是否支持内部位置控制
     *
     * 协议本身存在位置设定命令，但当前这个类没有把它作为稳定接口对外开放。
     */
    [[nodiscard]] bool supportsInternalPosition() const override { return false; }

    /**
     * @brief 初始化 CAN 滤波器
     * @param hcan CAN 句柄
     * @param filter_bank 滤波器组
     */
    static void CAN_FilterInit(CAN_HandleTypeDef* hcan, uint32_t filter_bank);
    /**
     * @brief CAN 基础接收回调
     * @param hcan CAN 句柄
     * @param header CAN 报文头
     * @param data 数据
     *
     * 如果项目里已经有统一 CAN 分发器，推荐把它注册到分发器中；否则也可以在 HAL 的 FIFO
     * 回调里直接调用它。
     */
    static void CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                       const CAN_RxHeaderTypeDef* header,
                                       const uint8_t*             data);

private:
    // 参数范围限制，对应 VESC_SET_*_MAX。
    static constexpr float kSetDutyMax            = 1.0f;
    static constexpr float kSetCurrentMax         = 2.0e6f;
    static constexpr float kSetCurrentBrakeMax    = 2.0e6f;
    static constexpr float kSetRPMMax             = 2.0e4f;
    static constexpr float kSetPositionMax        = 360.0f;
    static constexpr float kSetCurrentRelMax      = 1.0f;
    static constexpr float kSetCurrentBrakeRelMax = 1.0f;

    Config cfg_{}; ///< 构造时保存的配置

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
     * @brief 解码状态反馈
     * @param status_cmd 状态指令
     * @param data 数据
     */
    void decode(StatusCommand status_cmd, const uint8_t data[8]);

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

extern "C"
{
/**
 * @brief VESC FIFO0 中断回调包装
 *
 * 如果项目没有统一 CAN 分发器，可以直接使用这个 HAL 包装；否则更推荐统一把报文分发到
 * `CANBaseReceiveCallback()`。
 */
void VESC_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
/**
 * @brief VESC FIFO1 中断回调包装
 *
 * 如果项目没有统一 CAN 分发器，可以直接使用这个 HAL 包装；否则更推荐统一把报文分发到
 * `CANBaseReceiveCallback()`。
 */
void VESC_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
}
