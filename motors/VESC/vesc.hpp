/**
 * @file    vesc.hpp
 * @author  syhanjin
 * @date    2026-03-06
 * @brief   VESC motor driver (C++)
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
        SetRPM             = 3U,  ///< 设置转速, Data: ERPM (int32)
        SetPosition        = 4U,  ///< 设置位置, Data: pos * 1,000,000 (int32)
        SetCurrentRel      = 10U, ///< 设置相对电流，Data: ratio (-1 to 1) * 100,000 (int32)
        SetCurrentBrakeRel = 11U, ///< 设置相对刹车电流，Data: ratio (-1 to 1) * 100,100 (int32)
    };

    /**
     * VESC CAN 状态反馈指令集
     * 对应 VESC_CAN_PocketStatus_t
     */
    enum class StatusCommand : uint8_t
    {
        /**
         * Status Message 1
         * Data 6 ~ 7: Duty Cycle * 1000 (int16) - latest duty cycle (-1 to 1) multiplied by 1000
         * Data 4 ~ 5: Toal Current * 10 (int16) - Current in all units summed together with a scale
         * factor of 10 - assumed amps Data 0 ~ 3: ERPM (int32) - 32 bits probably because int16 is
         * +/- 32k which may be less than needed for some high speed motors
         */
        VESC_CAN_STATUS_1 = 9U,

        /**
         * Status Message 2
         * Data 4 ~ 7: Amp Hours Charged * 10000 (int32) - total regenerative amp hours put back in
         * battery Data 0 ~ 3: Amp Hours * 10000 (int32) - total amp hours consumed by unit
         */
        VESC_CAN_STATUS_2 = 14U,
        /**
         * Status Message 3
         * Data 4 ~ 7: watt_hours_charged * 10000 (int32) - total regenerative watt-hours put back
         * in battery Data 0 ~ 3: watt_hours * 10000 (int32) - total watt-hours consumed by unit
         */
        VESC_CAN_STATUS_3 = 15U,
        /**
         * Status Message 4
         * Data 6 ~ 7: PID Pos * 50 (int16) - not sure about units
         * Data 4 ~ 5: Toal Current In * 10 (int16) - assumed amps
         * Data 2 ~ 3: Motor Temp * 10 (int16) - assumed °C
         * Data 0 ~ 1: FET Temp * 10 (int16) - assumed °C
         */
        VESC_CAN_STATUS_4 = 16U,
        /**
         * Status Message 5
         * Data 6 ~ 7: Reserved
         * Data 4 ~ 5: Input Voltage * 10
         * Data 0 ~ 3: Tachometer value - assumed rpm(erpm)
         */
        VESC_CAN_STATUS_5 = 27U, ///< Input Voltage, Tachometer value
    };

    /**
     * VESC 配置结构体
     * 对应 VESC_Config_t
     */
    struct Config
    {
        CAN_HandleTypeDef* hcan;
        uint8_t            id;         ///< 控制器 id，0xFF 代表广播
        uint8_t            electrodes; ///< 电极数

        bool auto_zero = true; ///< 自动重置零点
        bool reverse   = false;
    };

    explicit VESCMotor(const Config& cfg);
    ~VESCMotor() override;

    [[nodiscard]] float getAngle() const override
    {
        return abs_angle_;
    }
    [[nodiscard]] float getVelocity() const override
    {
        return velocity_;
    }
    void resetAngle() override;

    [[nodiscard]] controllers::ControlMode defaultControlMode() const override
    {
        return controllers::ControlMode::ExternalPID;
    }

    [[nodiscard]] bool isConnected() const
    {
        return watchdog_.isFed();
    }

    [[nodiscard]] bool supportsCurrent() const override
    {
        return true;
    }
    void setCurrent(float current) override;

    [[nodiscard]] bool supportsInternalVelocity() const override
    {
        return true;
    }
    void setInternalVelocity(float rpm) override;

    /**
     * @brief 是否支持内部位置控制
     */
    [[nodiscard]] bool supportsInternalPosition() const override
    {
        return true;
    }
    /**
     * @brief 设置内部位置
     * @param pos 目标位置
     */
    void setInternalPosition(float pos) override;

    /**
     * @brief 定期重发最新 set 指令，保持 VESC 目标激活
     */
    void update();

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
     */
    static void CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                       const CAN_RxHeaderTypeDef* header,
                                       const uint8_t*             data);

private:
    // 参数范围限制，对应 VESC_SET_*_MAX
    static constexpr float kSetDutyMax            = 1.0f;
    static constexpr float kSetCurrentMax         = 2.0e6f;
    static constexpr float kSetCurrentBrakeMax    = 2.0e6f;
    static constexpr float kSetRPMMax             = 2.0e4f;
    static constexpr float kSetPositionMax        = 360.0f;
    static constexpr float kSetCurrentRelMax      = 1.0f;
    static constexpr float kSetCurrentBrakeRelMax = 1.0f;

    Config cfg_{};

    uint32_t          feedback_count_ = 0; ///< 反馈数
    service::Watchdog watchdog_;
    struct
    {
        float erpm{ 0.0f };          ///< 电转速
        float pos{ 0.0f };           ///< 绝对角度 0~360
        float duty{ 0.0f };          ///< 占空比
        float current_motor{ 0.0f }; ///< 电机电流
        float current_in{ 0.0f };    ///< 输入电流

        float amp_hours{ 0.0f };
        float amp_hours_charged{ 0.0f };
        float watt_hours{ 0.0f };
        float watt_hours_charged{ 0.0f };

        float motor_temperature{ 0.0f }; ///< 电机温度
        float mos_temperature{ 0.0f };   ///< MOSFET 温度

        float vin{ 0.0f }; ///< 输入电压
        float tachometer_value{ 0.0f };

        int32_t round_cnt{ 0 }; ///< 圈数统计
    } feedback_{};

    float angle_zero_ = 0.0f; ///< 零点角度

    float sign_ = 1.0f;

    float abs_angle_ = 0.0f; ///< 绝对角度
    float velocity_  = 0.0f; ///< 速度

    SetCommand last_set_cmd_    = SetCommand::SetRPM;
    float      last_set_value_  = 0.0f;
    bool       has_pending_set_ = false;

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
void VESC_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
void VESC_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
}
