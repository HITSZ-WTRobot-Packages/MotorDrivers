/**
 * @file    mi.cpp
 * @author  luoyue
 * @date    2026-05-11
 * @brief   小米 CyberGear 微电机驱动实现
 *
 * 这里实现了：
 * - CAN 扩展帧反馈到电机对象的映射与分发
 * - Type 2 反馈数据到物理量的换算与多圈展开
 * - Type 1 MIT 控制指令的打包与发送
 * - Type 3 / 4 使能失能
 * - Type 6 机械归零
 * - Type 17 / 18 参数读写
 *
 * 物理量 ↔ 协议 uint16 的换算均采用 ±half_range 映射到 [0, 65535] 的方式，
 * 与 Python 参考实现 pcan_cybergear.py 中的 _float_to_uint / _uint_to_float 保持一致。
 */
#include "mi.hpp"

#include "FixedPointerMap.hpp"
#include "can_driver.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180.0f)
#define RAD2DEG(__RAD__) ((__RAD__) / 3.14159265358979323846f * 180.0f)
#define RPM2RPS(__RPM__) ((__RPM__) / 60 * 2 * 3.14159265358979323846f)
#define RPS2RPM(__RPS__) ((__RPS__) * 60 / 2 / 3.14159265358979323846f)

namespace motors
{

// ---- 派生常量 ----

static constexpr float kPosHalfDeg = RAD2DEG(MIMotor::kPosMaxRad); // ±12.5 rad → ±716.2°

// ---- CAN 反馈映射表 ----

struct FeedbackMap
{
    CAN_HandleTypeDef*                                  hcan = nullptr;
    FixedPointerMap<size_t, MIMotor, MOTORS_MI_MAX_NUM> motors{};
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

// ---- 字节序读写 ----
//
// 协议字节序约定（按 CyberGear 说明书）：
// - 反馈帧（Type 2）：电机以大端序发送，MCU 以大端序解析
// - Type 1 控制帧：位置/速度/Kp/Kd 以大端序发送
// - Type 17 / 18 参数帧：index 与参数值按说明书示例以小端序发送
//
// 注意：Python 参考实现里的 struct.pack("HHHH", ...) 默认使用本机小端序，
// 与说明书 Type 1 示例不一致，这里以说明书为准。

static uint16_t read_u16_be(const uint8_t* src)
{
    return static_cast<uint16_t>(src[0]) << 8 | src[1];
}

static uint16_t read_u16_le(const uint8_t* src)
{
    return static_cast<uint16_t>(src[1]) << 8 | src[0];
}

static void write_u16_be(uint8_t* dst, uint16_t val)
{
    dst[0] = static_cast<uint8_t>(val >> 8);
    dst[1] = static_cast<uint8_t>(val & 0xFF);
}

static void write_float_le(uint8_t* dst, float val)
{
    union
    {
        float    f;
        uint32_t u;
    } conv{};
    conv.f = val;
    dst[0] = static_cast<uint8_t>(conv.u);
    dst[1] = static_cast<uint8_t>(conv.u >> 8);
    dst[2] = static_cast<uint8_t>(conv.u >> 16);
    dst[3] = static_cast<uint8_t>(conv.u >> 24);
}

// ---- 物理量 <-> 协议 uint16 换算 ----
//
// 所有映射均采用与 Python 参考实现一致的公式：
//   uint = (value - min) / (max - min) * 65535
//   value = uint / 65535 * (max - min) + min
//
// 等价于以 32767.5 为中点的对称映射：
//   uint = value / half_range * 32767.5 + 32767.5
//   value = (uint - 32767.5) / 32767.5 * half_range

// 位置：uint16 → deg
static float pos_raw_to_deg(uint16_t raw)
{
    return (static_cast<float>(raw) - 32767.5f) / 32767.5f * kPosHalfDeg;
}

// 位置：deg → uint16
static uint16_t pos_deg_to_raw(float deg)
{
    deg = std::clamp(deg, -kPosHalfDeg, kPosHalfDeg);
    return static_cast<uint16_t>((deg / kPosHalfDeg) * 32767.5f + 32767.5f);
}

// 速度：uint16 → rps
static float vel_raw_to_rps(uint16_t raw)
{
    return (static_cast<float>(raw) - 32767.5f) / 32767.5f * MIMotor::kVelMaxRps;
}

// 速度：rps → uint16
static uint16_t vel_rps_to_raw(float rps)
{
    rps = std::clamp(rps, -MIMotor::kVelMaxRps, MIMotor::kVelMaxRps);
    return static_cast<uint16_t>((rps / MIMotor::kVelMaxRps) * 32767.5f + 32767.5f);
}

// 力矩：uint16 → Nm
static float tor_raw_to_nm(uint16_t raw)
{
    return (static_cast<float>(raw) - 32767.5f) / 32767.5f * MIMotor::kTorqueMax;
}

// 力矩：Nm → uint16
static uint16_t tor_nm_to_raw(float nm)
{
    nm = std::clamp(nm, -MIMotor::kTorqueMax, MIMotor::kTorqueMax);
    return static_cast<uint16_t>((nm / MIMotor::kTorqueMax) * 32767.5f + 32767.5f);
}

// Kp: float → uint16
static uint16_t kp_to_raw(float kp)
{
    kp = std::clamp(kp, 0.0f, MIMotor::kKpMax);
    return static_cast<uint16_t>(kp / MIMotor::kKpMax * 65535.0f);
}

// Kd: float → uint16
static uint16_t kd_to_raw(float kd)
{
    kd = std::clamp(kd, 0.0f, MIMotor::kKdMax);
    return static_cast<uint16_t>(kd / MIMotor::kKdMax * 65535.0f);
}

// ---- 构造 / 析构 ----

MIMotor::MIMotor(const Config& cfg) : cfg_(cfg), sign_(cfg_.reverse ? -1.0f : 1.0f)
{
    const float reduction_rate = cfg_.reduction_rate > 0 ? cfg_.reduction_rate : 1.0f;
    inv_reduction_rate_        = 1.0f / reduction_rate;

    if (!register_motor(cfg_.hcan, cfg_.id, this))
        Error_Handler();
}

MIMotor::~MIMotor()
{
    unregister_motor(cfg_.hcan, cfg_.id);
}

// ---- 角度 / 零点 ----

void MIMotor::resetAngle()
{
    feedback_.round_cnt = 0;
    angle_zero_         = feedback_.angle;
    abs_angle_          = 0.0f;
}

// ---- CAN 报文头 ----

CAN_TxHeaderTypeDef MIMotor::tx_header(const uint8_t comm_type, const uint16_t data_area2) const
{
    CAN_TxHeaderTypeDef hdr{};
    hdr.ExtId = (static_cast<uint32_t>(comm_type) << 24) |
                (static_cast<uint32_t>(data_area2) << 8) | (cfg_.id & 0x7F);
    hdr.IDE = CAN_ID_EXT;
    hdr.RTR = CAN_RTR_DATA;
    hdr.DLC = 8;
    return hdr;
}

// ---- 使能 / 失能 ----

bool MIMotor::sendEnableDisable(const uint8_t comm_type, const uint8_t data0)
{
    uint8_t    data[8] = { data0, 0, 0, 0, 0, 0, 0, 0 };
    const auto hdr      = tx_header(comm_type, cfg_.master_id);
    return CAN_SendMessage(cfg_.hcan, &hdr, data) != CAN_SEND_FAILED;
}

bool MIMotor::enable()
{
    if (sendEnableDisable(3, 0))
    {
        enabled_ = true;
        return true;
    }
    return false;
}

bool MIMotor::disable(const bool clear_fault)
{
    if (sendEnableDisable(4, clear_fault ? 1U : 0U))
    {
        enabled_ = false;
        return true;
    }
    return false;
}

bool MIMotor::setMechanicalZero()
{
    return sendEnableDisable(6, 1);
}

// ---- MIT 控制指令（Type 1） ----

void MIMotor::setInternalMIT(const float t_ff, float p_ref, float v_ref, float kp, float kd)
{
    // 外部接口约定：t_ff 用 Nm，p_ref 用 deg，v_ref 用 deg/s，kp/kd 为标量。
    // 下发前先限幅，再应用方向符号，最后按协议量程换算为 uint16。
    p_ref = std::clamp(p_ref, -kPosHalfDeg, kPosHalfDeg);
    v_ref = std::clamp(DEG2RAD(v_ref), -MIMotor::kVelMaxRps, MIMotor::kVelMaxRps);

    const uint16_t pos_u16 = pos_deg_to_raw(sign_ * p_ref);
    const uint16_t vel_u16 = vel_rps_to_raw(sign_ * v_ref);
    const uint16_t kp_u16  = kp_to_raw(kp);
    const uint16_t kd_u16  = kd_to_raw(kd);
    const uint16_t tor_u16 = tor_nm_to_raw(sign_ * t_ff);

    uint8_t data[8];
    write_u16_be(&data[0], pos_u16);
    write_u16_be(&data[2], vel_u16);
    write_u16_be(&data[4], kp_u16);
    write_u16_be(&data[6], kd_u16);

    // data2（bit23~8 of ExtId）承载力矩值
    const auto hdr = tx_header(1, tor_u16);
    CAN_SendMessage(cfg_.hcan, &hdr, data);
}

// ---- 电流 / 力矩输入 ----

void MIMotor::setCurrent(const float current)
{
    // 纯力矩控制：Kp=Kd=0，仅写入力矩前馈
    setInternalMIT(current, 0, 0, 0, 0);
}

// ---- 参数读写 ----

bool MIMotor::readParam(const uint16_t index, float& value)
{
    uint8_t data[8] = { static_cast<uint8_t>(index & 0xFF),
                        static_cast<uint8_t>((index >> 8) & 0xFF),
                        0,
                        0,
                        0,
                        0,
                        0,
                        0 };
    param_read_valid_   = false;
    param_read_pending_ = true;
    param_read_index_   = index;

    const auto hdr = tx_header(17, cfg_.master_id);
    if (CAN_SendMessage(cfg_.hcan, &hdr, data) == CAN_SEND_FAILED)
    {
        param_read_pending_ = false;
        return false;
    }

    const uint32_t wait_start = HAL_GetTick();
    constexpr uint32_t timeout_ms = 10;
    while ((HAL_GetTick() - wait_start) < timeout_ms)
    {
        if (param_read_valid_ && param_read_index_ == index)
        {
            value = param_read_value_;
            param_read_pending_ = false;
            return true;
        }
    }

    param_read_pending_ = false;
    return false;
}

bool MIMotor::writeParam(const uint16_t index, const float value)
{
    uint8_t data[8] = { static_cast<uint8_t>(index & 0xFF),
                        static_cast<uint8_t>((index >> 8) & 0xFF),
                        0,
                        0,
                        0,
                        0,
                        0,
                        0 };
    if (index == 0x7005)
    {
        data[4] = static_cast<uint8_t>(value);
    }
    else
    {
        write_float_le(&data[4], value);
    }

    const auto hdr = tx_header(18, cfg_.master_id);
    return CAN_SendMessage(cfg_.hcan, &hdr, data) != CAN_SEND_FAILED;
}

// ---- 控制权 ----

bool MIMotor::tryAcquireController(controllers::IController* ctrl)
{
    // 获取控制权时同时使能电机
    if (!enabled_)
        enable();
    return IMotor::tryAcquireController(ctrl);
}

void MIMotor::releaseController(controllers::IController* ctrl)
{
    // 交接控制权时不失能电机，保持运行状态
    IMotor::releaseController(ctrl);
}

// ---- 反馈解码（Type 2） ----

void MIMotor::decode(const uint8_t data[8], const uint16_t fault, const RunState run_state)
{
    watchdog_.feed();

    const uint16_t raw_pos = read_u16_be(&data[0]);
    const uint16_t raw_vel = read_u16_be(&data[2]);
    const uint16_t raw_tor = read_u16_be(&data[4]);
    const uint16_t raw_tmp = read_u16_be(&data[6]);

    const float angle_deg = pos_raw_to_deg(raw_pos);
    const float vel_rps   = vel_raw_to_rps(raw_vel);
    const float torque_nm = tor_raw_to_nm(raw_tor);
    const float temp_c    = static_cast<float>(raw_tmp) / 10.0f;

    // 多圈展开：CyberGear 位置反馈量程为 ±kPosMaxRad（约 ±2 圈），
    // 反馈给的是量程内的单圈位置。当相邻两帧角度跳变超过 kPosHalfDeg 时，
    // 判定为跨越量程边界，累计跨圈计数。
    const float angle_delta = angle_deg - feedback_.angle;
    if (angle_delta < -kPosHalfDeg)
        feedback_.round_cnt++;
    else if (angle_delta > kPosHalfDeg)
        feedback_.round_cnt--;

    feedback_.angle       = angle_deg;
    feedback_.velocity    = RPS2RPM(vel_rps);
    feedback_.torque      = torque_nm;
    feedback_.temperature = temp_c;
    feedback_.run_state   = run_state;
    feedback_.fault       = fault;
    feedback_count_++;

    // 输出轴连续角度 = 方向 × (累计圈角度 + 当前单圈角度 - 零点) × 减速比
    abs_angle_ = sign_ *
                 (static_cast<float>(feedback_.round_cnt) * kPosHalfDeg * 2.0f + angle_deg -
                  angle_zero_) *
                 inv_reduction_rate_;
    velocity_ = sign_ * feedback_.velocity * inv_reduction_rate_;

    // 上电第 50 帧自动归零
    if (feedback_count_ == 50 && cfg_.auto_zero)
        resetAngle();
}

// ---- CAN 滤波器初始化 ----

void MIMotor::CAN_FilterInit(CAN_HandleTypeDef* hcan, const uint32_t filter_bank)
{
    const CAN_FilterTypeDef filter = {
        .FilterIdHigh         = 0x0000,
        .FilterIdLow          = 0x0000 | CAN_ID_EXT,
        .FilterMaskIdHigh     = 0x0000,
        .FilterMaskIdLow      = 0x0000 | CAN_ID_EXT,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterBank           = filter_bank,
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_32BIT,
        .FilterActivation     = ENABLE,
        .SlaveStartFilterBank = 14,
    };

    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
        Error_Handler();
}

// ---- CAN 接收分发 ----

void MIMotor::CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                     const CAN_RxHeaderTypeDef* header,
                                     const uint8_t*             data)
{
    if (!hcan || !header || !data || header->IDE != CAN_ID_EXT || header->DLC < 8)
        return;

    const auto m = find_map(hcan);
    if (!m)
        return;

    // 发送时电机 ID 在 bit7~0；反馈时电机把自身 ID 放在 bit15~8
    const uint8_t id    = static_cast<uint8_t>((header->ExtId >> 8) & 0xFF);
    const auto    motor = m->motors.find(id);
    if (!motor)
        return;

    const uint8_t comm_type = static_cast<uint8_t>((header->ExtId >> 24) & 0x1F);

    // 仅处理 Type 2 反馈帧
    if (comm_type == 2)
    {
        const uint16_t fault =
                static_cast<uint16_t>((header->ExtId >> 16) & 0x3F);
        const auto run_state =
                static_cast<RunState>((header->ExtId >> 22) & 0x3);
        motor->decode(data, fault, run_state);
    }
    else if (comm_type == 17 && motor->param_read_pending_)
    {
        const uint16_t index = read_u16_le(&data[0]);
        if (index != motor->param_read_index_)
            return;

        if (index == 0x7005)
        {
            motor->param_read_value_ = static_cast<float>(data[4]);
        }
        else
        {
            float value = 0.0f;
            std::memcpy(&value, &data[4], sizeof(float));
            motor->param_read_value_ = value;
        }
        motor->param_read_valid_ = true;
    }
}

// ---- HAL FIFO 中断回调包装 ----

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
