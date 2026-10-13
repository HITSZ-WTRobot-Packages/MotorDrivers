/**
 * @file    vesc.cpp
 * @author  syhanjin
 * @date    2026-03-06
 * @brief   VESC 电机驱动实现
 */
#include "vesc.hpp"

#include <array>
#include <cstdint>
#include <span>

namespace motors
{

// VESC 状态反馈里的多字节数据使用大端格式。
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
    // 实际参与换算的是“极对数”和减速比：
    // ERPM = 输出轴机械 rpm * 减速比 * 极对数。
    if (cfg_.electrodes == 0)
        cfg_.electrodes = 1;

    cfg_.reduction_rate = cfg_.reduction_rate > 0 ? cfg_.reduction_rate : 1.0f;

    inv_reduction_rate_ = 1.0f / cfg_.reduction_rate;
    erpm2velocity_      = inv_reduction_rate_ / static_cast<float>(cfg_.electrodes);

    // 总线按“注册时借用的对象地址”回调，故必须在 Can::start() 之前注册；
    // 全项目统一约定：本对象须活到该总线不再使用，不提供注销接口。
    if (cfg_.can.register_callback<&VESCMotor::on_can>(*this) != bsp::can::Status::Ok)
        Error_Handler();
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
        // VESC 协议要求把浮点物理量缩放成整数再发送。
        data_value = static_cast<int32_t>(clamp(sign_ * value, kSetDutyMax) * 1.0e5f);
        break;
    case SetCommand::SetCurrent:
        data_value = static_cast<int32_t>(clamp(sign_ * value, kSetCurrentMax) * 1.0e3f);
        break;
    case SetCommand::SetCurrentBrake:
        data_value = static_cast<int32_t>(clamp(value, kSetCurrentBrakeMax) * 1.0e3f);
        break;
    case SetCommand::SetRPM:
        // 对外接口使用输出轴机械 rpm，发送前换算成 VESC 协议要求的 ERPM。
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

    // 标准 VESC 设置帧只使用前 4 个字节传数值，DLC 为 4。
    const std::array<uint8_t, 4> data{ static_cast<uint8_t>(data_value >> 24),
                                       static_cast<uint8_t>(data_value >> 16),
                                       static_cast<uint8_t>(data_value >> 8),
                                       static_cast<uint8_t>(data_value) };

    const bsp::can::FrameView frame{
        .header = { .id      = (static_cast<uint32_t>(cmd) << 8) | cfg_.id,
                    .id_type = bsp::can::IdType::Extended,
                    .format  = bsp::can::FrameFormat::Classic,
                    .type    = bsp::can::FrameType::Data },
        .data   = std::span<const uint8_t>{ data }
    };

    // 与旧实现一致：命令接口无返回值，发送结果交由总线侧诊断处理，不做重试。
    (void)cfg_.can.send(frame);
}

void VESCMotor::setCurrent(const float current)
{
    sendSetCommand(SetCommand::SetCurrent, current);
}

void VESCMotor::setInternalVelocity(const float rpm, const float maxi)
{
    (void)maxi;
    sendSetCommand(SetCommand::SetRPM, rpm);
}

void VESCMotor::decode(const StatusCommand status_cmd, const uint8_t data[8])
{
    // TODO: vesc 大概率会做降频处理，此处需要支持自定义超时时间
    watchdog_.feed();
    ++feedback_count_;

    switch (status_cmd)
    {
    case StatusCommand::VESC_CAN_STATUS_1:
        // 状态 1 里主要是转速、电流和占空比。
        feedback_.erpm          = static_cast<float>(be_to_i32(data + 0));
        feedback_.current_motor = static_cast<float>(be_to_i16(data + 4)) / 10.0f;
        feedback_.duty          = static_cast<float>(be_to_i16(data + 6)) / 1000.0f;
        // 协议反馈给出的是 ERPM，这里再按极对数和减速比还原成输出轴机械 rpm。
        velocity_ = sign_ * erpm2velocity_ * feedback_.erpm;
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

        // 位置反馈是 0~360 度单圈值，需要做跨圈展开。
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

void VESCMotor::on_can(const bsp::can::FrameView& frame) noexcept
{
    // VESC 全部报文都是经典格式的扩展数据帧，状态反馈固定 8 字节负载。
    if (frame.header.id_type != bsp::can::IdType::Extended ||
        frame.header.format != bsp::can::FrameFormat::Classic ||
        frame.header.type != bsp::can::FrameType::Data || frame.data.size() != 8)
        return;

    // 扩展帧 ID 高字节是状态类型，低字节是控制器 ID。
    if (static_cast<uint8_t>(frame.header.id & 0xFFU) != cfg_.id)
        return;

    decode(static_cast<StatusCommand>((frame.header.id >> 8) & 0xFFU), frame.data.data());
}

bsp::can::Status VESCMotor::CAN_FilterInit(bsp::can::Can& can, const uint32_t filter_index)
{
    // VESC 只使用扩展帧。掩码为 0 表示不比较任何标识符位，即接受全部扩展帧
    // （与旧实现“32 位掩码只比较 IDE 位”等价），命中后进入 FIFO0。
    // index 由调用方沿用原有分配：bxCAN 为 filter bank，FDCAN 为 element 下标。
    return can.configure_filter(bsp::can::IdFilter{ .index   = filter_index,
                                                    .id_type = bsp::can::IdType::Extended,
                                                    .mode    = bsp::can::FilterMode::Mask,
                                                    .action  = bsp::can::FilterAction::Fifo0,
                                                    .id1     = 0U,
                                                    .id2     = 0U });
}

} // namespace motors
