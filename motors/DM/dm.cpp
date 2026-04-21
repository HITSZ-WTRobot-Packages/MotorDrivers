/**
 * @file    dm.cpp
 * @author  syhanjin
 * @date    2026-02-27
 * @brief   达妙电机驱动实现
 *
 * 这里实现了：
 * - DM 报文到电机对象的映射
 * - 反馈数据到物理量的换算
 * - MIT / 位置 / 速度三种模式的指令打包
 */
#include "dm.hpp"
#include "can_driver.h"
#include "FixedPointerMap.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180.0f)
#define RAD2DEG(__RAD__) ((__RAD__) / 3.14159265358979323846f * 180.0f)
#define RPM2RPS(__RPM__) ((__RPM__) / 60 * 2 * 3.14159265358979323846f)
#define RPS2RPM(__RPS__) ((__RPS__) * 60 / 2 / 3.14159265358979323846f)

namespace motors
{

// 只是为了写 `cfg_.mode | cfg_.id0` 更直观一些。
uint32_t operator|(const DMMotor::Mode& a, const uint32_t& b)
{
    return static_cast<uint32_t>(a) | b;
}
uint32_t operator|(const uint32_t& b, const DMMotor::Mode& a)
{
    return static_cast<uint32_t>(a) | b;
}

struct FeedbackMap
{
    CAN_HandleTypeDef*                                  hcan = nullptr;
    FixedPointerMap<size_t, DMMotor, MOTORS_DM_MAX_NUM> motors{};
};

static std::array<FeedbackMap, CAN_NUM> map{};

// 按当前库设计，DM 的 master_id 是全局统一值。
// 这样可以简化回调分发和对象映射；代价是所有 DM 电机都必须共享同一个 master_id。
static uint32_t master_id_;

static FeedbackMap* find_map(const CAN_HandleTypeDef* hcan)
{
    for (auto& m : map)
    {
        if (m.hcan == hcan)
            return &m;
    }
    return nullptr;
}

// 注册电机：反馈回调会靠这张表把报文分发到具体对象。
static bool register_motor(CAN_HandleTypeDef* hcan, const size_t id0, DMMotor* motor)
{
    if (!hcan || !motor)
        return false;
    FeedbackMap* m = find_map(hcan);
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
    return m->motors.insert(id0, motor);
}

// 注销电机。
static bool unregister_motor(CAN_HandleTypeDef* hcan, const size_t id0)
{
    if (!hcan)
        return false;

    const auto m = find_map(hcan);
    if (!m)
        return false;

    return m->motors.erase(id0);
}

// 对于 S3519，傻逼达妙会返回减速前的位置；对于其他电机，返回减速后的
static constexpr float get_inv_pos_reduction_rate(const DMMotor::Type type)
{
    switch (type)
    {
    case DMMotor::Type::S3519:
        return 1.0f / 19.203f;
    default:
        return 1.0f;
    }
}

// 这个傻逼 DM 反馈的速度都是减速后的
static constexpr float get_inv_vel_reduction_rate(const DMMotor::Type type)
{
    switch (type)
    {
    default:
        return 1.0f;
    }
}

DMMotor::DMMotor(const Config& cfg) : cfg_(cfg), sign_(cfg_.reverse ? -1.0f : 1.0f)
{
    // id 仅低四位有效
    cfg_.id0 &= 0x0F;

    inv_reduction_rate_ = 1.0f / // 取倒数将除法转为乘法加快运算速度
                          (cfg_.reduction_rate > 0 ? cfg_.reduction_rate : 1.0f); // 外接减速比

    pos_max_deg = RAD2DEG(cfg_.pos_max_rad);

    if (!register_motor(cfg_.hcan, cfg_.id0, this))
        Error_Handler();
}

DMMotor::~DMMotor()
{
    unregister_motor(cfg_.hcan, cfg_.id0);
}

void DMMotor::resetAngle()
{
    feedback_.count = 0;
    angle_zero_     = feedback_.angle;
    abs_angle_      = 0;
}

void DMMotor::decode(const uint8_t data[8])
{
    // feed the watchdog to indicate motor is alive
    // TODO: 允许自定义超时时间，用来降低总线压力
    watchdog_.feed();

    // --- extract raw data ---
    // DM 的反馈格式把状态、位置、速度、力矩压在 8 字节里，部分字段需要拆 bit。
    const auto raw_pos = static_cast<int16_t>((data[1] << 8 | data[2]) - 32767);
    const auto raw_vel = static_cast<int16_t>((data[3] << 4 | data[4] >> 4) - 2047);
    const auto raw_tor = static_cast<int16_t>(((data[4] & 0x0F) << 8 | data[5]) - 2047);

    // --- convert to physical units ---
    const float feedback_angle  = static_cast<float>(raw_pos) / 32767 * cfg_.pos_max_rad;
    const float feedback_vel    = static_cast<float>(raw_vel) / 2047 * cfg_.vel_max_rad;
    const float feedback_torque = static_cast<float>(raw_tor) / 2047 * cfg_.tor_max;

    // --- convert to physical units used by current implementation ---
    const float angle_deg = RAD2DEG(feedback_angle);
    const float vel_rpm   = RPS2RPM(feedback_vel);

    // --- handle angle wrapping ---
    // 达妙的角度反馈是一个有限范围内的位置值，这里通过跨边界检测累计圈数。
    const float angle_delta = angle_deg - feedback_.angle;

    if (angle_delta < -pos_max_deg)
        feedback_.count++;
    else if (angle_delta > pos_max_deg)
        feedback_.count--;

    // --- update feedback struct ---
    feedback_.angle      = angle_deg;
    feedback_.velocity   = vel_rpm;
    feedback_.torque     = feedback_torque;
    feedback_.temp_mos   = static_cast<int8_t>(data[6]);
    feedback_.temp_rotor = static_cast<int8_t>(data[7]);
    feedback_.state      = static_cast<State>((data[0] >> 4) & 0x0F);
    feedback_count_++;

    // --- calculate absolute angle and velocity considering reverse and reduction ---
    // 对外统一暴露的是“输出轴”的角度和速度，因此需要考虑方向与减速比。
    abs_angle_ = sign_ *
                 (static_cast<float>(feedback_.count) * pos_max_deg * 2 +
                  (angle_deg - angle_zero_)) *
                 inv_reduction_rate_ * get_inv_pos_reduction_rate(cfg_.type);
    velocity_ = sign_ * vel_rpm * inv_reduction_rate_ * get_inv_vel_reduction_rate(cfg_.type);

    // --- automatic zeroing after 50 feedbacks ---
    if (feedback_count_ == 50 && cfg_.auto_zero)
    {
        resetAngle();
    }
}

/**
 * 设置低层输出
 * @param current 在 DM MIT 模式下，这里按力矩输入处理，单位 Nm
 */
void DMMotor::setCurrent(const float current)
{
    // MIT 可以退化成单独的力矩输入。
    setInternalMIT(current, 0, 0, 0, 0);
}

void DMMotor::setInternalVelocity(const float rpm)
{
    // 库设计上对外统一使用 rpm；DM 协议实际发送的是 rad/s 的 float。
    const float rps  = sign_ * RPM2RPS(rpm);
    const auto  data = reinterpret_cast<const uint8_t*>(&rps);

    const auto hdr = tx_header(4);
    CAN_SendMessage(cfg_.hcan, &hdr, data);
}

void DMMotor::setInternalPosition(float pos)
{
    // 当前接口位置参考使用 deg；DM 协议里要传 rad。
    pos = sign_ * DEG2RAD(pos);

    uint8_t data[8];
    memcpy(data, &pos, sizeof(float));
    memcpy(data + 4, &cfg_.vel_max_rad, sizeof(float));

    const auto hdr = tx_header(8);
    CAN_SendMessage(cfg_.hcan, &hdr, data);
}

/**
 * MIT 控制指令
 * @param t_ff 前馈力矩
 * @param p_ref 位置参考，单位 deg
 * @param v_ref 速度参考，单位 deg/s
 * @param kp Kp
 * @param kd Kd
 */
void DMMotor::setInternalMIT(const float t_ff, float p_ref, float v_ref, float kp, float kd)
{
    // 先在 MCU 侧做一次限幅，避免把超量程数据打包进协议。
    // MIT 五元组里的 `v_ref` 不是普通速度模式的目标转速，而是与 `p_ref` 配套的速度项，
    // 通常可以理解成位置参考的导数。因此这里按 deg/s -> rad/s 换算，而不是走 rpm 语义。
    p_ref = std::clamp(DEG2RAD(p_ref), -cfg_.pos_max_rad, cfg_.pos_max_rad);
    v_ref = std::clamp(DEG2RAD(v_ref), -cfg_.vel_max_rad, cfg_.vel_max_rad);
    kp    = std::clamp(kp, 0.0f, 500.0f);
    kd    = std::clamp(kd, 0.0f, 5.0f);

    // DM MIT 协议本质上是把若干浮点物理量缩放后压成定长整数。
    const auto p = static_cast<uint16_t>(sign_ * p_ref / cfg_.pos_max_rad * 32767.5f + 32767.5f);
    const auto v = static_cast<uint16_t>(sign_ * v_ref / cfg_.vel_max_rad * 2047.5f + 2047.5f);
    const auto t = static_cast<uint16_t>(sign_ * t_ff / cfg_.tor_max * 2047.5f + 2047.5f);

    const auto kp_ = static_cast<uint16_t>(kp / 500.0f * 4095.0f);
    const auto kd_ = static_cast<uint16_t>(kd / 5.0f * 4095.0f);

    const uint8_t data[8] = { static_cast<uint8_t>(p >> 8),
                              static_cast<uint8_t>(p),
                              static_cast<uint8_t>(v >> 4),
                              static_cast<uint8_t>(v << 4 | kp_ >> 8),
                              static_cast<uint8_t>(kp_),
                              static_cast<uint8_t>(kd_ >> 4),
                              static_cast<uint8_t>(kd_ << 4 | t >> 8),
                              static_cast<uint8_t>(t) };

    const auto hdr = tx_header(8);
    CAN_SendMessage(cfg_.hcan, &hdr, data);
}

void DMMotor::CAN_FilterInit(CAN_HandleTypeDef* hcan,
                             const uint32_t     filter_bank,
                             const uint32_t     master_id)
{
    master_id_ = master_id;
    CAN_FilterTypeDef filter{};
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterBank           = filter_bank;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;

    // standard id: 11-bit, left shift 5
    filter.FilterIdHigh     = static_cast<uint16_t>(master_id << 5);
    filter.FilterIdLow      = 0x0000;
    filter.FilterMaskIdHigh = static_cast<uint16_t>(0x7FF << 5);
    filter.FilterMaskIdLow  = 0xFFFF;
    if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }
}

void DMMotor::CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                     const CAN_RxHeaderTypeDef* header,
                                     const uint8_t*             data)
{
    if (!hcan)
        return;
    const auto m = find_map(hcan);
    if (!m || !header || header->IDE != CAN_ID_STD || header->StdId != master_id_)
        return;

    // data[0] 高 4 位是状态 ERR，低 4 位是电机 ID。
    const uint8_t id = data[0] & 0x0F;

    auto motor = m->motors.find(id);
    if (motor != nullptr)
        motor->decode(data);
}

bool DMMotor::tryAcquireController(controllers::IController* ctrl)
{
    // DM 电机通常需要先发使能报文，控制器拿到控制权时顺带做一次使能。
    if (!enabled_)
        enable();
    return IMotor::tryAcquireController(ctrl);
}

void DMMotor::releaseController(controllers::IController* ctrl)
{
    // 此处不在对电机失能处理，因为使用过程中的控制权往往是交接而不是释放，电机不应当失能
    // disable();
    IMotor::releaseController(ctrl);
}
bool DMMotor::enable()
{
    const auto hdr = tx_header(8);
    if (CAN_SendMessage(cfg_.hcan, &hdr, ENABLE_MSG) == CAN_SEND_FAILED)
        return false;
    enabled_ = true;
    return true;
}
bool DMMotor::disable()
{
    const auto hdr = tx_header(8);
    if (CAN_SendMessage(cfg_.hcan, &hdr, DISABLE_MSG) == CAN_SEND_FAILED)
        return false;
    enabled_ = false;
    return true;
}

CAN_TxHeaderTypeDef DMMotor::tx_header(const uint8_t& DLC) const
{
    CAN_TxHeaderTypeDef hdr{};
    hdr.StdId = cfg_.mode | cfg_.id0;
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = DLC;
    return hdr;
}

extern "C" void DM_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    // FIFO 里可能一次累积多帧，因此循环读取直到清空。
    do
    {
        CAN_RxHeaderTypeDef header;
        uint8_t             data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            Error_Handler();
            return;
        }
        DMMotor::CANBaseReceiveCallback(hcan, &header, data);
    } while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0);
}

extern "C" void DM_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    // FIFO1 的处理过程与 FIFO0 一致。
    do
    {
        CAN_RxHeaderTypeDef header;
        uint8_t             data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &header, data) != HAL_OK)
        {
            Error_Handler();
            return;
        }
        DMMotor::CANBaseReceiveCallback(hcan, &header, data);
    } while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0);
}

} // namespace motors
