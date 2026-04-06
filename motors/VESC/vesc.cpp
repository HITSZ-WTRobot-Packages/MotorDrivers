/**
 * @file    vesc.cpp
 * @author  syhanjin
 * @date    2026-03-06
 * @brief   VESC motor driver (C++)
 */
#include "vesc.hpp"

#include "FixedPointerMap.hpp"
#include "can_driver.h"

#include <array>
#include <cstdint>

namespace motors
{

struct FeedbackMap
{
    CAN_HandleTypeDef*                                      hcan = nullptr;
    FixedPointerMap<size_t, VESCMotor, MOTORS_VESC_MAX_NUM> motors{};
};

static std::array<FeedbackMap, CAN_NUM> map{};

static FeedbackMap* find_map(const CAN_HandleTypeDef* hcan)
{
    for (auto& m : map)
    {
        if (m.hcan == hcan)
            return &m;
    }
    return nullptr;
}

static bool register_motor(CAN_HandleTypeDef* hcan, const size_t id, VESCMotor* motor)
{
    if (!hcan || !motor)
        return false;

    FeedbackMap* m = find_map(hcan);
    if (!m)
    {
        for (auto& slot : map)
        {
            if (slot.hcan == nullptr)
            {
                slot.hcan = hcan;
                m         = &slot;
                break;
            }
        }
        if (!m)
            return false;
    }

    return m->motors.insert(id, motor);
}

static bool unregister_motor(CAN_HandleTypeDef* hcan, const size_t id)
{
    if (!hcan)
        return false;

    const auto m = find_map(hcan);
    if (!m)
        return false;

    return m->motors.erase(id);
}

static int32_t be_to_i32(const uint8_t* bytes)
{
    return static_cast<int32_t>(
            static_cast<uint32_t>(bytes[0]) << 24 | static_cast<uint32_t>(bytes[1]) << 16 |
            static_cast<uint32_t>(bytes[2]) << 8 | static_cast<uint32_t>(bytes[3]));
}

static int16_t be_to_i16(const uint8_t* bytes)
{
    return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) << 8 |
                                static_cast<uint16_t>(bytes[1]));
}

VESCMotor::VESCMotor(const Config& cfg) : cfg_(cfg), sign_(cfg_.reverse ? -1.0f : 1.0f)
{
    if (cfg_.electrodes == 0)
        cfg_.electrodes = 1;

    cfg_.reduction_rate = cfg_.reduction_rate > 0 ? cfg_.reduction_rate : 1.0f;

    inv_reduction_rate_ = 1.0f / cfg_.reduction_rate;
    erpm2velocity_      = inv_reduction_rate_ / static_cast<float>(cfg_.electrodes);

    if (!register_motor(cfg_.hcan, cfg_.id, this))
        Error_Handler();
}

VESCMotor::~VESCMotor()
{
    unregister_motor(cfg_.hcan, cfg_.id);
}

void VESCMotor::resetAngle()
{
    feedback_.round_cnt = 0;
    angle_zero_         = feedback_.pos;
    abs_angle_          = 0.0f;
}

float VESCMotor::clamp(const float value, const float abs_max)
{
    if (value > abs_max)
        return abs_max;
    if (value < -abs_max)
        return -abs_max;
    return value;
}

void VESCMotor::sendSetCommand(const SetCommand cmd, const float value) const
{
    int32_t data_value = 0;

    switch (cmd)
    {
    case SetCommand::SetDuty:
        data_value = static_cast<int32_t>(clamp(sign_ * value, kSetDutyMax) * 1.0e5f);
        break;
    case SetCommand::SetCurrent:
        data_value = static_cast<int32_t>(clamp(sign_ * value, kSetCurrentMax) * 1.0e3f);
        break;
    case SetCommand::SetCurrentBrake:
        data_value = static_cast<int32_t>(clamp(value, kSetCurrentBrakeMax) * 1.0e3f);
        break;
    case SetCommand::SetRPM:
        data_value = static_cast<int32_t>(
                clamp(sign_ * value * static_cast<float>(cfg_.electrodes) * cfg_.reduction_rate,
                      kSetRPMMax));
        break;
    case SetCommand::SetPosition:
        data_value = static_cast<int32_t>(clamp(sign_ * value, kSetPositionMax) * 1.0e6f);
        break;
    case SetCommand::SetCurrentRel:
        data_value = static_cast<int32_t>(clamp(sign_ * value, kSetCurrentRelMax) * 1.0e5f);
        break;
    case SetCommand::SetCurrentBrakeRel:
        data_value = static_cast<int32_t>(clamp(value, kSetCurrentBrakeRelMax) * 1.0e5f);
        break;
    default:
        return;
    }

    const uint8_t data[8] = { static_cast<uint8_t>(data_value >> 24),
                              static_cast<uint8_t>(data_value >> 16),
                              static_cast<uint8_t>(data_value >> 8),
                              static_cast<uint8_t>(data_value),
                              0x00,
                              0x00,
                              0x00,
                              0x00 };

    CAN_TxHeaderTypeDef tx_header{};
    tx_header.ExtId = (static_cast<uint32_t>(cmd) << 8) | cfg_.id;
    tx_header.IDE   = CAN_ID_EXT;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 4;

    CAN_SendMessage(cfg_.hcan, &tx_header, data);
}

void VESCMotor::setCurrent(const float current)
{
    sendSetCommand(SetCommand::SetCurrent, current);
}

void VESCMotor::setInternalVelocity(const float rpm)
{
    sendSetCommand(SetCommand::SetRPM, rpm);
}

void VESCMotor::decode(const StatusCommand status_cmd, const uint8_t data[8])
{
    watchdog_.feed();
    ++feedback_count_;

    switch (status_cmd)
    {
    case StatusCommand::VESC_CAN_STATUS_1:
        feedback_.erpm          = static_cast<float>(be_to_i32(data + 0));
        feedback_.current_motor = static_cast<float>(be_to_i16(data + 4)) / 10.0f;
        feedback_.duty          = static_cast<float>(be_to_i16(data + 6)) / 1000.0f;
        velocity_               = sign_ * erpm2velocity_ * feedback_.erpm;
        break;

    case StatusCommand::VESC_CAN_STATUS_2:
        feedback_.amp_hours         = static_cast<float>(be_to_i32(data + 0)) / 10000.0f;
        feedback_.amp_hours_charged = static_cast<float>(be_to_i32(data + 4)) / 10000.0f;
        break;

    case StatusCommand::VESC_CAN_STATUS_3:
        feedback_.watt_hours         = static_cast<float>(be_to_i32(data + 0)) / 10000.0f;
        feedback_.watt_hours_charged = static_cast<float>(be_to_i32(data + 4)) / 10000.0f;
        break;

    case StatusCommand::VESC_CAN_STATUS_4:
    {
        feedback_.mos_temperature   = static_cast<float>(be_to_i16(data + 0)) / 10.0f;
        feedback_.motor_temperature = static_cast<float>(be_to_i16(data + 2)) / 10.0f;
        feedback_.current_in        = static_cast<float>(be_to_i16(data + 4)) / 10.0f;

        const float new_pos = static_cast<float>(be_to_i16(data + 6)) / 50.0f;

        if (new_pos < 90.0f && feedback_.pos > 270.0f)
            feedback_.round_cnt++;
        else if (new_pos > 270.0f && feedback_.pos < 90.0f)
            feedback_.round_cnt--;

        feedback_.pos = new_pos;

        abs_angle_ = sign_ *
                     (static_cast<float>(feedback_.round_cnt) * 360.0f +
                      (feedback_.pos - angle_zero_)) *
                     inv_reduction_rate_;
        break;
    }

    case StatusCommand::VESC_CAN_STATUS_5:
        feedback_.tachometer_value = static_cast<float>(be_to_i32(data + 0));
        feedback_.vin              = static_cast<float>(be_to_i16(data + 4)) / 10.0f;
        break;

    default:
        return;
    }

    if (feedback_count_ == 50 && cfg_.auto_zero)
        resetAngle();
}

void VESCMotor::CAN_FilterInit(CAN_HandleTypeDef* hcan, const uint32_t filter_bank)
{
    const CAN_FilterTypeDef filter = { .FilterIdHigh         = 0x0000,
                                       .FilterIdLow          = 0x0000 | CAN_ID_EXT,
                                       .FilterMaskIdHigh     = 0x0000,
                                       .FilterMaskIdLow      = 0x0000 | CAN_ID_EXT,
                                       .FilterFIFOAssignment = CAN_FILTER_FIFO0,
                                       .FilterBank           = filter_bank,
                                       .FilterMode           = CAN_FILTERMODE_IDMASK,
                                       .FilterScale          = CAN_FILTERSCALE_32BIT,
                                       .FilterActivation     = ENABLE,
                                       .SlaveStartFilterBank = 14 };

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
        Error_Handler();
}

void VESCMotor::CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                       const CAN_RxHeaderTypeDef* header,
                                       const uint8_t*             data)
{
    if (!hcan || !header || !data || header->IDE != CAN_ID_EXT)
        return;

    const auto m = find_map(hcan);
    if (!m)
        return;

    const auto id    = static_cast<uint8_t>(header->ExtId & 0xFF);
    const auto motor = m->motors.find(id);
    if (!motor)
        return;

    const auto status_cmd = static_cast<StatusCommand>((header->ExtId >> 8) & 0xFF);
    motor->decode(status_cmd, data);
}

extern "C" void VESC_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    do
    {
        CAN_RxHeaderTypeDef header;
        uint8_t             data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            Error_Handler();
            return;
        }
        VESCMotor::CANBaseReceiveCallback(hcan, &header, data);
    } while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0);
}

extern "C" void VESC_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    do
    {
        CAN_RxHeaderTypeDef header;
        uint8_t             data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data) != HAL_OK)
        {
            Error_Handler();
            return;
        }
        VESCMotor::CANBaseReceiveCallback(hcan, &header, data);
    } while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0);
}

} // namespace motors
