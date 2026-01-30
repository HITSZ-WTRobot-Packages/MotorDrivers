/**
 * @file    dji.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#pragma once

#define CAN_NUM (2)

#include "can_driver.h"
#include "motor_if.hpp"
#include "watchdog.hpp"

namespace motors
{

class DJIMotor final : public IMotor
{
public:
    enum class IqSetCMDGroup : size_t
    {
        IqCMDGroup_1_4 = 0U,
        IqCMDGroup_5_8 = 4U,
    };

    enum class Type
    {
        M3508_C620 = 0U,
        M2006_C610,

        MOTOR_TYPE_COUNT,
    };

    struct Config
    {
        CAN_HandleTypeDef* hcan;
        Type               type;
        uint8_t            id1; ///< 电机编号 1~8

        bool  auto_zero      = true;  ///< 是否自动清零角度
        bool  reverse        = false; ///< 是否反转
        float reduction_rate = 1.0f;  ///< 外接减速比
    };

    explicit DJIMotor(const Config& cfg);
    ~DJIMotor() override;
    float getAngle() const override { return abs_angle_; }
    float getVelocity() const override { return velocity_; }
    void  resetAngle() override;

    void decode(const uint8_t data[8]);
    bool isConnected() const { return watchdog_.isFed(); }

    controllers::ControlMode defaultControlMode() const override
    {
        return controllers::ControlMode::ExternalPID;
    }

    bool supportsCurrent() const override { return true; }

    void setCurrent(const float current) override
    {
        iq_cmd_ = static_cast<int16_t>(cfg_.reverse ? -current : current);
    }
    int16_t getIqCMD() const { return iq_cmd_; }

    static void CAN_FilterInit(CAN_HandleTypeDef* hcan, uint32_t filter_bank);
    static void SendIqCommand(CAN_HandleTypeDef* hcan, IqSetCMDGroup cmd_group);
    static void CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                       const CAN_RxHeaderTypeDef* header,
                                       const uint8_t*             data);

private:
    Config cfg_;

    float angle_zero_ = 0; //< 零点角度 (unit: degree)

    float inv_reduction_rate_; ///< 减速比

    /* Feedback */
    uint32_t          feedback_count_ = 0; //< 接收到的反馈数据数量
    service::Watchdog watchdog_;
    struct
    {
        float mech_angle{ 0 }; //< 单圈机械角度 (unit: degree)
        float rpm{ 0 };        //< 转速
        // float current; //< 电流大小
        // float temperature; //< 温度

        int32_t round_cnt{ 0 }; //< 圈数
    } feedback_{};

    /* Data */
    float abs_angle_ = 0; //< 电机轴输出角度 (unit: degree)
    float velocity_  = 0; //< 电机轴输出速度 (unit: rpm)

    /* Output */
    int16_t iq_cmd_ = 0; //< 电流指令值
};

} // namespace motors

void DJI_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
void DJI_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
