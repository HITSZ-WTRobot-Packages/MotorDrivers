/**
 * @file    mi.cpp
 * @author  Luoyue777
 * @date    2026-05-05
 * @brief   小米 CyberGear 微电机驱动实现
 *
 * 主要完成：
 * - 扩展帧 CAN 报文到电机对象的映射与分发
 * - mode 2 自动反馈解码（角度、速度、力矩、温度）
 * - mode 0x11 参数读取响应解码
 * - MIT / 位置 / 速度三种模式的指令打包
 * - 使能 / 失能序列
 */
#include "mi.hpp"
#include "can_driver.h"
#include "FixedPointerMap.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180.0f)
#define RAD2DEG(__RAD__) ((__RAD__) / 3.14159265358979323846f * 180.0f)
#define RPM2RPS(__RPM__) ((__RPM__) / 60.0f * 2.0f * 3.14159265358979323846f)
#define RPS2RPM(__RPS__) ((__RPS__) * 60.0f / 2.0f / 3.14159265358979323846f)

namespace motors
{

// ---- CAN 反馈映射表 ----

struct FeedbackMap
{
    CAN_HandleTypeDef*                                  hcan = nullptr;
    FixedPointerMap<size_t, MIMotor, MOTORS_MI_MAX_NUM> motors{};
};

static std::array<FeedbackMap, CAN_NUM> map{};
static uint32_t                         global_master_id_;

static FeedbackMap* find_map(const CAN_HandleTypeDef* hcan)
{
    for (auto& m : map)
    {
        if (m.hcan == hcan)
            return &m;
    }
    return nullptr;
}

static bool register_motor(CAN_HandleTypeDef* hcan, const size_t id, MIMotor* motor)
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

// ---- 构造 / 析构 ----

MIMotor::MIMotor(const Config& cfg) : cfg_(cfg), sign_(cfg_.reverse ? -1.0f : 1.0f)
{
    inv_reduction_rate_ =
        1.0f / (cfg_.reduction_rate > 0 ? cfg_.reduction_rate : 1.0f);

    if (!register_motor(cfg_.hcan, cfg_.id, this))
        Error_Handler();
}

MIMotor::~MIMotor()
{
    unregister_motor(cfg_.hcan, cfg_.id);
}

// ---- 反馈接口 ----

void MIMotor::resetAngle()
{
    angle_zero_ = feedback_.angle;
    abs_angle_  = 0;
}

controllers::ControlMode MIMotor::defaultControlMode() const
{
    switch (cfg_.run_mode)
    {
    case RunMode::MIT:
        return controllers::ControlMode::InternalMIT;
    case RunMode::Pos:
        return controllers::ControlMode::InternalPos;
    case RunMode::Spd:
        return controllers::ControlMode::InternalVel;
    case RunMode::Cur:
    default:
        return controllers::ControlMode::ExternalPID;
    }
}

// ---- 控制输出 ----

void MIMotor::setCurrent(const float current)
{
    // MIT 退化为纯力矩输入：kp=kd=0, pos=vel=0, 仅前馈力矩
    setInternalMIT(current, 0, 0, 0, 0);
}

void MIMotor::setInternalVelocity(const float rpm)
{
    const float rps     = sign_ * RPM2RPS(rpm);
    const float clamped = std::clamp(rps, -cfg_.vel_max_rad, cfg_.vel_max_rad);

    uint8_t data[4];
    std::memcpy(data, &clamped, sizeof(float));
    writeParam(INDEX_SPEED_CMD, data);
}

void MIMotor::setInternalPosition(const float pos_deg)
{
    const float pos_rad = sign_ * DEG2RAD(pos_deg);
    const float clamped = std::clamp(pos_rad, -cfg_.pos_max_rad, cfg_.pos_max_rad);

    uint8_t data[4];
    std::memcpy(data, &clamped, sizeof(float));
    writeParam(INDEX_POS_CMD, data);
}

void MIMotor::setInternalMIT(const float t_ff, float p_ref, float v_ref, float kp, float kd)
{
    // 转换为协议单位并限幅
    const float t_clamped = std::clamp(sign_ * t_ff, -cfg_.tor_max, cfg_.tor_max);
    const float p_rad     = std::clamp(sign_ * DEG2RAD(p_ref), -cfg_.pos_max_rad, cfg_.pos_max_rad);
    const float v_rps     = std::clamp(sign_ * DEG2RAD(v_ref), -cfg_.vel_max_rad, cfg_.vel_max_rad);
    const float kp_c      = std::clamp(kp, 0.0f, cfg_.kp_max);
    const float kd_c      = std::clamp(kd, 0.0f, cfg_.kd_max);

    // 物理量 → uint16 打包（协议要求）
    auto float_to_uint = [](float x, float x_min, float x_max, int bits) -> uint16_t {
        const float span = x_max - x_min;
        if (x > x_max) x = x_max;
        else if (x < x_min) x = x_min;
        return static_cast<uint16_t>((x - x_min) * static_cast<float>((1 << bits) - 1) / span);
    };

    const uint16_t t_val =
        float_to_uint(t_clamped, -cfg_.tor_max, cfg_.tor_max, 16);
    const uint16_t p_val =
        float_to_uint(p_rad, -cfg_.pos_max_rad, cfg_.pos_max_rad, 16);
    const uint16_t v_val =
        float_to_uint(v_rps, -cfg_.vel_max_rad, cfg_.vel_max_rad, 16);
    const uint16_t kp_val = float_to_uint(kp_c, 0.0f, cfg_.kp_max, 16);
    const uint16_t kd_val = float_to_uint(kd_c, 0.0f, cfg_.kd_max, 16);

    const uint8_t payload[8] = {
        static_cast<uint8_t>(p_val >> 8),   // 位置高 8 位
        static_cast<uint8_t>(p_val & 0xFF), // 位置低 8 位
        static_cast<uint8_t>(v_val >> 8),   // 速度高 8 位
        static_cast<uint8_t>(v_val & 0xFF), // 速度低 8 位
        static_cast<uint8_t>(kp_val >> 8),  // Kp 高 8 位
        static_cast<uint8_t>(kp_val & 0xFF), // Kp 低 8 位
        static_cast<uint8_t>(kd_val >> 8),  // Kd 高 8 位
        static_cast<uint8_t>(kd_val & 0xFF), // Kd 低 8 位
    };

    // 扩展 ID data 字段承载力矩值
    sendFrame(Control, t_val, payload);
}

// ---- CAN 通信 ----

void MIMotor::sendFrame(const MsgMode mode, const uint16_t data_field, const uint8_t payload[8])
{
    CAN_TxHeaderTypeDef tx_header;
    tx_header.ExtId = packExtId(cfg_.id, data_field, static_cast<uint8_t>(mode));
    tx_header.IDE   = CAN_ID_EXT;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;

    CAN_SendMessage(cfg_.hcan, &tx_header, payload);
}

void MIMotor::writeParam(const uint16_t index, const uint8_t value[4])
{
    uint8_t payload[8] = {};
    std::memcpy(&payload[0], &index, 2);
    std::memcpy(&payload[4], value, 4);

    sendFrame(WriteParam, cfg_.master_id, payload);
}

void MIMotor::setRunMode(const RunMode mode)
{
    uint8_t value[4] = {};
    value[0]         = static_cast<uint8_t>(mode);
    writeParam(INDEX_RUN_MODE, value);
    cfg_.run_mode = mode;
}

// ---- 反馈解码 ----

void MIMotor::decode(const CAN_RxHeaderTypeDef* header, const uint8_t data[8])
{
    if (!header || header->IDE != CAN_ID_EXT)
        return;

    const uint32_t ext_id    = header->ExtId;
    const uint8_t  mode      = getExtIdMode(ext_id);
    const uint16_t data_field = getExtIdData(ext_id);
    const uint8_t  motor_id  = data_field & 0xFF;

    // 仅处理发给本电机的帧
    if (motor_id != cfg_.id)
        return;

    switch (static_cast<MsgMode>(mode))
    {
    case Feedback: {
        // 收到反馈即表示在线
        watchdog_.feed();

        const uint8_t state_byte = static_cast<uint8_t>(data_field >> 8);

        // 解析故障状态（bits 5-0）
        if ((state_byte & 0xC0) == 0)
        {
            feedback_.state = State::OK;
        }
        else
        {
            uint8_t s = state_byte;
            for (uint8_t i = 1; i < 7; i++)
            {
                if (s & 0x01)
                    feedback_.state = static_cast<State>(i);
                s >>= 1;
            }
        }

        // 原始反馈解析
        // 角度：int16 → rad，范围 ±4π（约 ±12.57 rad）
        const int16_t raw_angle  = static_cast<int16_t>((data[0] << 8 | data[1]) ^ 0x8000);
        const int16_t raw_speed  = static_cast<int16_t>((data[2] << 8 | data[3]) ^ 0x8000);
        const int16_t raw_torque = static_cast<int16_t>((data[4] << 8 | data[5]) ^ 0x8000);
        const int16_t raw_temp   = static_cast<int16_t>(data[6] << 8 | data[7]);

        const float angle_rad  = static_cast<float>(raw_angle) / 32768.0f * 4.0f *
                                 3.14159265358979323846f;
        const float speed_rps  = static_cast<float>(raw_speed) / 32768.0f * 30.0f;
        const float torque_nm  = static_cast<float>(raw_torque) / 32768.0f * 12.0f;
        const float temp_degc  = static_cast<float>(raw_temp) / 10.0f;

        feedback_.angle       = RAD2DEG(angle_rad);
        feedback_.velocity    = RPS2RPM(speed_rps);
        feedback_.torque      = torque_nm;
        feedback_.temperature = temp_degc;

        // 换算到输出轴
        abs_angle_ = sign_ * (feedback_.angle - angle_zero_) * inv_reduction_rate_;
        velocity_  = sign_ * feedback_.velocity * inv_reduction_rate_;

        feedback_count_++;
        if (feedback_count_ == 50 && cfg_.auto_zero)
            resetAngle();
        break;
    }

    case ReadParam: {
        // 参数读取响应
        const uint16_t index = static_cast<uint16_t>(data[0] | (data[1] << 8));

        if (index == INDEX_MECH_POS)
        {
            // data[4..7] 为 IEEE 754 float（小端），单位 rad
            uint32_t temp = (static_cast<uint32_t>(data[7]) << 24) |
                            (static_cast<uint32_t>(data[6]) << 16) |
                            (static_cast<uint32_t>(data[5]) << 8) | data[4];
            float pos_rad;
            std::memcpy(&pos_rad, &temp, sizeof(float));

            feedback_.angle = RAD2DEG(pos_rad);
            abs_angle_ = sign_ * (feedback_.angle - angle_zero_) * inv_reduction_rate_;
        }
        else if (index == INDEX_MECH_VEL)
        {
            uint32_t temp = (static_cast<uint32_t>(data[7]) << 24) |
                            (static_cast<uint32_t>(data[6]) << 16) |
                            (static_cast<uint32_t>(data[5]) << 8) | data[4];
            float vel_rps;
            std::memcpy(&vel_rps, &temp, sizeof(float));

            feedback_.velocity = RPS2RPM(vel_rps);
            velocity_ = sign_ * feedback_.velocity * inv_reduction_rate_;
        }
        break;
    }

    default:
        break;
    }
}

// ---- 控制权与使能 ----

bool MIMotor::tryAcquireController(controllers::IController* ctrl)
{
    if (!enabled_)
        enable();
    return IMotor::tryAcquireController(ctrl);
}

void MIMotor::releaseController(controllers::IController* ctrl)
{
    IMotor::releaseController(ctrl);
}

bool MIMotor::enable()
{
    // 使能序列：停止 → 设运行模式 → 使能
    uint8_t payload[8] = {};

    // 1. 停止
    sendFrame(StopFrame, cfg_.master_id, payload);

    // 2. 设置运行模式
    setRunMode(cfg_.run_mode);

    // 3. 使能
    sendFrame(EnableFrame, cfg_.master_id, payload);

    enabled_ = true;
    return true;
}

bool MIMotor::disable()
{
    uint8_t payload[8] = {};
    sendFrame(StopFrame, cfg_.master_id, payload);
    enabled_ = false;
    return true;
}

// ---- CAN 滤波器与回调分发 ----

void MIMotor::CAN_FilterInit(CAN_HandleTypeDef* hcan,
                             const uint32_t     filter_bank,
                             const uint32_t     master_id)
{
    global_master_id_ = master_id;

    CAN_FilterTypeDef filter{};
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank           = filter_bank;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    // STM32 32-bit 滤波器匹配扩展帧：
    // 硬件比对值为 { EXID[28:0] << 3, IDE(1), RTR(1), 0(1) }
    // ID[7:0] 映射到 EXID[7:0]，对应 filter 位[10:3]
    // 匹配条件：EXID[7:0] == master_id, IDE == 1, RTR == 0
    filter.FilterIdHigh     = 0;
    filter.FilterIdLow      = static_cast<uint16_t>((master_id << 3) | (1 << 2));
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow  = static_cast<uint16_t>(0x7F8 | (1 << 2) | (1 << 1));

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
        Error_Handler();
}

void MIMotor::CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                     const CAN_RxHeaderTypeDef* header,
                                     const uint8_t*             data)
{
    if (!hcan || !header || !data)
        return;
    if (header->IDE != CAN_ID_EXT)
        return;

    const auto m = find_map(hcan);
    if (!m)
        return;

    // 从扩展 ID data 字段低字节提取电机 ID
    const uint16_t data_field = getExtIdData(header->ExtId);
    const uint8_t  motor_id  = data_field & 0xFF;

    auto motor = m->motors.find(motor_id);
    if (motor != nullptr)
        motor->decode(header, data);
}

// ---- HAL FIFO 回调包装 ----

extern "C" void MI_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
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
        MIMotor::CANBaseReceiveCallback(hcan, &header, data);
    } while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0);
}

extern "C" void MI_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
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
        MIMotor::CANBaseReceiveCallback(hcan, &header, data);
    } while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0);
}

} // namespace motors
