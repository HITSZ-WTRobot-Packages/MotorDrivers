/**
 * @file    mi.cpp
 * @author  luoyue
 * @date    2026-05-11
 * @brief   小米 CyberGear 微电机驱动实现
 *
 * 这里实现了：
 * - CAN 扩展帧到电机对象的回调分发
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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>

#define DEG2RAD(__DEG__) ((__DEG__) * 3.14159265358979323846f / 180.0f)
#define RAD2DEG(__RAD__) ((__RAD__) / 3.14159265358979323846f * 180.0f)
#define RPM2RPS(__RPM__) ((__RPM__) / 60 * 2 * 3.14159265358979323846f)
#define RPS2RPM(__RPS__) ((__RPS__) * 60 / 2 / 3.14159265358979323846f)

namespace motors
{

// ---- 派生常量 ----

static constexpr float kPosHalfDeg = RAD2DEG(MIMotor::kPosMaxRad); // ±12.5 rad → ±716.2°

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

    if (cfg_.can.register_callback<&MIMotor::on_can>(*this) != bsp::can::Status::Ok)
        Error_Handler();
}

// ---- 角度 / 零点 ----

void MIMotor::resetAngle()
{
    feedback_.round_cnt = 0;
    angle_zero_         = feedback_.angle;
    abs_angle_          = 0.0f;
}

// ---- CAN 报文头与发送 ----

bsp::can::FrameHeader MIMotor::make_header(const uint8_t comm_type, const uint16_t data_area2) const
{
    bsp::can::FrameHeader hdr{};
    hdr.id      = (static_cast<uint32_t>(comm_type) << 24) |
                  (static_cast<uint32_t>(data_area2) << 8) | (cfg_.id & 0x7F);
    hdr.id_type = bsp::can::IdType::Extended;
    hdr.format  = bsp::can::FrameFormat::Classic;
    hdr.type    = bsp::can::FrameType::Data;
    return hdr;
}

bool MIMotor::send_frame(const uint8_t comm_type, const uint16_t data_area2,
                         const uint8_t (&data)[8])
{
    const bsp::can::FrameView frame{ make_header(comm_type, data_area2),
                                     std::span<const uint8_t>{ data, sizeof data } };
    const bsp::can::SendResult result = cfg_.can.send(frame);

    // Submitted/Queued/ReplacedOldest 均表示帧已被驱动受理（旧版对应 HAL 未返回失败）。
    return result == bsp::can::SendResult::Submitted || result == bsp::can::SendResult::Queued ||
           result == bsp::can::SendResult::ReplacedOldest;
}

// ---- 使能 / 失能 ----

bool MIMotor::sendEnableDisable(const uint8_t comm_type, const uint8_t data0)
{
    const uint8_t data[8] = { data0, 0, 0, 0, 0, 0, 0, 0 };
    return send_frame(comm_type, cfg_.master_id, data);
}

bool MIMotor::selfEnable()
{
    if (sendEnableDisable(3, 0))
    {
        enable_requested_ = true;
        return true;
    }
    return false;
}

void MIMotor::selfDisable()
{
    // 没有失败返回通道：发送拒绝必须遵守现有致命失败约定。
    if (!sendEnableDisable(4, 0))
        Error_Handler();
    enable_requested_ = false;
}

void MIMotor::selfProtect()
{
    // 此时 Core 状态可能仍为 Disabled，不经过公开 set* 门控。
    if (!sendMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f))
        Error_Handler();
}

bool MIMotor::clearFault()
{
    if (isEnabled())
        return false;
    const bool accepted = sendEnableDisable(4, 1);
    if (accepted)
        enable_requested_ = false;
    return accepted;
}

bool MIMotor::setMechanicalZero()
{
    return sendEnableDisable(6, 1);
}

// ---- MIT 控制指令（Type 1） ----

bool MIMotor::sendMIT(const float t_ff, float p_ref, float v_ref, float kp, float kd)
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
    return send_frame(1, tor_u16, data);
}

void MIMotor::applyInternalMIT(const float t_ff, const float p_ref, const float v_ref,
                               const float kp, const float kd)
{
    (void)sendMIT(t_ff, p_ref, v_ref, kp, kd);
}

// ---- 电流 / 力矩输入 ----

void MIMotor::applyCurrent(const float current)
{
    // 纯力矩控制：Kp=Kd=0，仅写入力矩前馈
    applyInternalMIT(current, 0, 0, 0, 0);
}

// ---- 参数读写 ----

bool MIMotor::readParam(const uint16_t index, float& value)
{
    const uint8_t data[8] = { static_cast<uint8_t>(index & 0xFF),
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

    if (!send_frame(17, cfg_.master_id, data))
    {
        param_read_pending_ = false;
        return false;
    }

    const uint32_t     wait_start = HAL_GetTick();
    constexpr uint32_t timeout_ms = 10;
    while ((HAL_GetTick() - wait_start) < timeout_ms)
    {
        if (param_read_valid_ && param_read_index_ == index)
        {
            value               = param_read_value_;
            param_read_pending_ = false;
            return true;
        }
    }

    param_read_pending_ = false;
    return false;
}

bool MIMotor::writeParam(const uint16_t index, const float value)
{
    // 参数目标也是运动命令，不能绕过公开 set* 的同一状态边界。
    if ((index == 0x7006 || index == 0x700A || index == 0x7016) &&
        !isStandalone() && !isControlled())
        return false;

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

    return send_frame(18, cfg_.master_id, data);
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

bsp::can::Status MIMotor::CAN_FilterInit(bsp::can::Can& can, const uint32_t filter_index)
{
    // 与旧版 32 位掩码 bank 等价：key 只带 IDE 位、掩码只选中 IDE 位，即放行全部
    // 扩展帧、拒绝标准帧。IdFilter 不比较 RTR，远程帧也会进入 FIFO，由 on_can 丢弃。
    // index 的资源含义随后端而异：bxCAN 为 filter bank 编号，FDCAN 为扩展过滤器
    // element 下标，两者不可互换。
    bsp::can::IdFilter filter{};
    filter.index   = filter_index;
    filter.id_type = bsp::can::IdType::Extended;
    filter.mode    = bsp::can::FilterMode::Mask;
    filter.action  = bsp::can::FilterAction::Fifo0;
    filter.id1     = 0;
    filter.id2     = 0;
    return can.configure_filter(filter);
}

// ---- CAN 接收回调 ----

void MIMotor::on_can(const bsp::can::FrameView& frame) noexcept
{
    const bsp::can::FrameHeader& header = frame.header;
    // 本协议只使用扩展 ID 的经典数据帧，且所有报文均为 8 字节负载。
    if (header.id_type != bsp::can::IdType::Extended ||
        header.format != bsp::can::FrameFormat::Classic ||
        header.type != bsp::can::FrameType::Data || frame.data.size() < 8)
        return;

    // 发送时电机 ID 在 bit7~0；反馈时电机把自身 ID 放在 bit15~8
    const uint8_t id = static_cast<uint8_t>((header.id >> 8) & 0xFF);
    if (id != cfg_.id)
        return;

    const uint8_t comm_type = static_cast<uint8_t>((header.id >> 24) & 0x1F);

    // 仅处理 Type 2 反馈帧
    if (comm_type == 2)
    {
        const uint16_t fault     = static_cast<uint16_t>((header.id >> 16) & 0x3F);
        const auto     run_state = static_cast<RunState>((header.id >> 22) & 0x3);
        decode(frame.data.data(), fault, run_state);
    }
    else if (comm_type == 17 && param_read_pending_)
    {
        const uint16_t index = read_u16_le(&frame.data[0]);
        if (index != param_read_index_)
            return;

        if (index == 0x7005)
        {
            param_read_value_ = static_cast<float>(frame.data[4]);
        }
        else
        {
            float value = 0.0f;
            std::memcpy(&value, &frame.data[4], sizeof(float));
            param_read_value_ = value;
        }
        param_read_valid_ = true;
    }
}

} // namespace motors
