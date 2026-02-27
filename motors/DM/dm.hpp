/**
 * @file    dm.hpp
 * @author  syhanjin
 * @date    2026-02-27
 * @brief   达妙电机驱动
 */
#pragma once
#include "can.h"
#include "motor_if.hpp"
#include "watchdog.hpp"

#ifndef MOTORS_DM_MAX_NUM
#    define MOTORS_DM_MAX_NUM (8)
#endif

namespace motors
{

class DMMotor : public IMotor
{
public:
    enum class Type
    {
        J4310_2EC,

        MotorTypeCount,
    };
    enum class Mode
    {
        MIT = 0x000,
        Pos = 0x100,
        Vel = 0x200,
    };

    struct Config
    {
        CAN_HandleTypeDef* hcan;
        uint8_t            id0; ///< 电调 ID，建议从 0x009 到 0x00F
        Type               type;
        Mode               mode;        ///< 电机控制模式，需要在上位机修改
        float              pos_max_rad; ///< 位置最大值，需要在上位机修改
        float              vel_max_rad; ///< 速度最大值，需要在上位机修改
        float              tor_max;     ///< 力矩最大值，需要在上位机修改

        bool  auto_zero      = true;
        bool  reverse        = false; ///< 是否反转
        float reduction_rate = 1.0f;  ///< 外接减速比
    };

    explicit DMMotor(const Config& cfg);
    ~DMMotor() override;

    [[nodiscard]] float getAngle() const override { return abs_angle_; }
    [[nodiscard]] float getVelocity() const override { return velocity_; }
    void                resetAngle() override;

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

    void decode(const uint8_t data[8]);

    [[nodiscard]] bool isConnected() const { return watchdog_.isFed(); }

    [[nodiscard]] bool supportsCurrent() const override { return cfg_.mode == Mode::MIT; }
    void               setCurrent(float current) override;

    [[nodiscard]] bool supportsInternalVelocity() const override { return cfg_.mode == Mode::Vel; }
    void               setInternalVelocity(float rpm) override;

    [[nodiscard]] bool supportsInternalPosition() const override { return cfg_.mode == Mode::Pos; }
    void               setInternalPosition(float pos) override;

    [[nodiscard]] bool supportsInternalMIT() const override { return true; }
    void setInternalMIT(float t_ff, float p_ref, float v_ref, float kp, float kd) override;

    static void CAN_FilterInit(CAN_HandleTypeDef* hcan, uint32_t filter_bank, uint32_t master_id);
    static void CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                       const CAN_RxHeaderTypeDef* header,
                                       const uint8_t*             data);

    bool tryAcquireController(controllers::IController* ctrl) override;
    void releaseController(controllers::IController* ctrl) override;

    bool enable();
    bool disable();
    void ping()
    {
        // 失能状态下，使用使能包作为心跳包
        if (!enabled_)
            disable();
    }

private:
    Config cfg_;

    bool enabled_{ false };

    uint32_t          feedback_count_ = 0;
    service::Watchdog watchdog_;
    struct
    {
        float angle{ 0 };
        float velocity{ 0 };
        float torque{ 0 };
        float temp_mos{ 0 };
        float temp_rotor{ 0 };

        uint8_t error{ 0 };

        int32_t count{ 0 };
    } feedback_{};
    float angle_zero_{ 0 };
    float inv_reduction_rate_;
    float pos_max_deg;

    float abs_angle_ = 0;
    float velocity_  = 0;

    float sign_; ///< 反转符号

    [[nodiscard]] CAN_TxHeaderTypeDef tx_header(const uint8_t& DLC) const;

    static constexpr uint8_t ENABLE_MSG[]  = { 0xFF, 0XFF, 0XFF, 0xFF, 0XFF, 0XFF, 0XFF, 0XFC };
    static constexpr uint8_t DISABLE_MSG[] = { 0xFF, 0XFF, 0XFF, 0xFF, 0XFF, 0XFF, 0XFF, 0XFD };
};

} // namespace motors

extern "C"
{
void DJI_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
void DJI_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
}