/**
 * @file    dji.cpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   Brief description of the file
 *
 * Detailed description (optional).
 *
 */
#include "dji.hpp"
#include "can_driver.h"
#include <array>
#include <cstdint>

namespace motors
{

constexpr size_t kMaxMotorPerCan = 8;

// CAN 映射结构体
struct DJI_FeedbackMap
{
    CAN_HandleTypeDef*                     hcan = nullptr; // CAN handle
    std::array<DJIMotor*, kMaxMotorPerCan> motors{};       // 电机数组
};

// 全局固定数组，CAN_NUM 个 CAN
static std::array<DJI_FeedbackMap, CAN_NUM> map{};

// 辅助函数：id1(1~8) -> index(0~7)
constexpr size_t id_to_index(const size_t id1)
{
    return id1 - 1;
}
constexpr bool valid_id1(const size_t id1)
{
    return id1 >= 1 && id1 <= kMaxMotorPerCan;
}
constexpr bool valid_id0(const size_t id0)
{
    return id0 < kMaxMotorPerCan;
}

// 查找 hcan 对应的 map
static DJI_FeedbackMap* find_map(const CAN_HandleTypeDef* hcan)
{
    for (auto& m : map)
    {
        if (m.hcan == hcan)
            return &m;
    }
    return nullptr;
}

// 注册电机
bool register_motor(CAN_HandleTypeDef* hcan, const size_t id1, DJIMotor* motor)
{
    if (!hcan || !motor || !valid_id1(id1))
        return false;

    DJI_FeedbackMap* m = find_map(hcan);
    if (!m)
    {
        // 找空槽创建
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
            return false; // 没空槽
    }

    const size_t idx = id_to_index(id1);
    if (m->motors[idx] != nullptr)
        return false; // 已注册
    m->motors[idx] = motor;
    return true;
}

// 注销电机
bool unregister_motor(CAN_HandleTypeDef* hcan, const size_t id1)
{
    if (!hcan || !valid_id1(id1))
        return false;

    DJI_FeedbackMap* m = find_map(hcan);
    if (!m)
        return false;

    size_t idx = id_to_index(id1);
    if (m->motors[idx] == nullptr)
        return false; // 未注册
    m->motors[idx] = nullptr;
    return true;
}

// 查找电机
DJIMotor* get_motor(const CAN_HandleTypeDef* hcan, const CAN_RxHeaderTypeDef* header)
{
    if (header->IDE != CAN_ID_STD)
        return nullptr;
    const uint8_t id0 = header->StdId - 0x201;
    if (!hcan || !valid_id0(id0))
        return nullptr;
    const DJI_FeedbackMap* m = find_map(hcan);
    return m ? m->motors[id0] : nullptr;
}

static constexpr float get_reduction_rate(const DJIMotor::Type type)
{
    switch (type)
    {
    case DJIMotor::Type::M3508_C620:
        return 3591.0f / 187.0f;
    case DJIMotor::Type::M2006_C610:
        return 36.0f;
    default:
        return 1.0f;
    }
}

DJIMotor::DJIMotor(const Config& cfg) : cfg_(cfg)
{
    inv_reduction_rate_ = 1.0f / // 取倒数将除法转为乘法加快运算速度
                          ((cfg_.reduction_rate > 0 ? cfg_.reduction_rate : 1.0f) // 外接减速比
                           * get_reduction_rate(cfg_.type));                      // 电机内部减速比

    /* 注册回调 */
    if (!register_motor(cfg_.hcan, cfg_.id1, this))
        Error_Handler();
}

DJIMotor::~DJIMotor()
{
    unregister_motor(cfg_.hcan, cfg_.id1);
}

void DJIMotor::resetAngle()
{
    feedback_.round_cnt = 0;
    angle_zero_         = feedback_.mech_angle;
    abs_angle_          = 0;
}

static int16_t read_int16(const uint8_t data[2])
{
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) << 8 | data[1]);
}

void DJIMotor::decode(const uint8_t data[8])
{
    watchdog_.feed();

    const float feedback_angle = static_cast<float>(read_int16(&data[0])) * 360.0f / 8192.0f;

    const float feedback_rpm = read_int16(&data[2]);

    // TODO: 堵转电流检测
    // const float feedback_current = (float)((int16_t)data[4] << 8 | data[5]) / 16384.0f * 20.0f;

    // M3508 和 M2006 的转速均不会超过 120 deg/s
    if (feedback_angle < 90 && feedback_.mech_angle > 270)
        feedback_.round_cnt++;
    if (feedback_angle > 270 && feedback_.mech_angle < 90)
        feedback_.round_cnt--;

    feedback_.mech_angle = feedback_angle;
    abs_angle_           = (cfg_.reverse ? -1.0f : 1.0f) * // 反转时需要反转角度输入
                 (static_cast<float>(feedback_.round_cnt) * 360.0f + feedback_.mech_angle -
                  angle_zero_) *
                 inv_reduction_rate_;

    feedback_.rpm = feedback_rpm;
    velocity_     = (cfg_.reverse ? -1.0f : 1.0f) * // 反转时需要反转速度输入
                feedback_.rpm * inv_reduction_rate_;

    feedback_count_++;
    if (feedback_count_ == 50 && cfg_.auto_zero)
    {
        // 上电后第 50 次反馈执行输出轴清零操作
        resetAngle();
    }
}

void DJIMotor::CAN_FilterInit(CAN_HandleTypeDef* hcan, const uint32_t filter_bank)
{
    const CAN_FilterTypeDef sFilterConfig = { .FilterIdHigh     = 0x200 << 5,
                                              .FilterIdLow      = 0x0000,
                                              .FilterMaskIdHigh = 0x7F0 << 5, //< 高 7
                                                                              // 位匹配，第 4
                                                                              // 位忽略
                                              .FilterMaskIdLow      = 0x0000,
                                              .FilterFIFOAssignment = CAN_FILTER_FIFO0,
                                              .FilterBank           = filter_bank,
                                              .FilterMode           = CAN_FILTERMODE_IDMASK,
                                              .FilterScale          = CAN_FILTERSCALE_32BIT,
                                              .FilterActivation     = ENABLE,
                                              .SlaveStartFilterBank = 14 };
    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
void DJIMotor::SendIqCommand(CAN_HandleTypeDef* hcan, IqSetCMDGroup cmd_group)
{
    if (!hcan)
        return;

    const DJI_FeedbackMap* m = find_map(hcan);

    uint8_t iq_data[8] = {};
    for (size_t j = 0; j < 4; j++)
    {
        const DJIMotor* dji = m->motors[j + static_cast<size_t>(cmd_group)];
        // 只有受控的大疆电机才应该发送指令
        if (dji != nullptr && dji->currentController() != nullptr)
        {
            const int16_t iq_cmd = dji->getIqCMD();
            iq_data[1 + j * 2]   = static_cast<uint8_t>(iq_cmd & 0xFF);      // 电流值低 8 位
            iq_data[0 + j * 2]   = static_cast<uint8_t>(iq_cmd >> 8 & 0xFF); // 电流值高 8 位
        }
    }

    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = cmd_group == IqSetCMDGroup::IqCMDGroup_1_4 ? 0x200 : 0x1FF;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;

    CAN_SendMessage(hcan, &tx_header, iq_data);
}
void DJIMotor::CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                      const CAN_RxHeaderTypeDef* header,
                                      const uint8_t*             data)
{
    DJI_FeedbackMap* m = find_map(hcan);
    if (m == nullptr)
        return;

    DJIMotor* motor = get_motor(hcan, header);
    if (motor != nullptr)
        motor->decode(data);
}

extern "C" void DJI_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
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
        DJIMotor::CANBaseReceiveCallback(hcan, &header, data);
    } while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0);
}

extern "C" void DJI_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
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
        DJIMotor::CANBaseReceiveCallback(hcan, &header, data);
    } while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0);
}

} // namespace motors