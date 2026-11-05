/**
 * @file    dji.cpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   DJI 电机驱动实现
 *
 * 主要完成三件事：
 * - 每个电机在构造时把自己的成员回调注册到所在总线，由总线按帧广播
 * - 把原始反馈换算成输出轴角度与速度
 * - 把多台电机的电流命令聚合打包发送
 */
#include "dji.hpp"
#include <array>
#include <cstdint>

namespace motors
{

constexpr size_t   kMaxMotorPerCan = 8;     // DJI 标准反馈协议一条 CAN 最多挂 8 台电机
constexpr uint32_t kDjiFeedbackBase = 0x200; // 反馈 ID 基址，1 号电机为 0x201
constexpr uint32_t kDjiFeedbackMask = 0x7F0; // 高 7 位匹配，低 4 位忽略

// 辅助函数：把外部习惯使用的 1~8 号编号转成数组下标 0~7。
constexpr size_t id_to_index(const size_t id1)
{
    return id1 - 1;
}
constexpr bool valid_id1(const size_t id1)
{
    return id1 >= 1 && id1 <= kMaxMotorPerCan;
}

namespace
{
/**
 * @brief 一条总线上的固定发送槽位。
 *
 * `can == nullptr` 表示该 entry 空闲；`count` 记录已登记电机数，析构时据此判断
 * 释放 entry，无需再扫描 8 个槽位。每个 `id1` 至多占用一个槽位。
 */
struct TxSlots
{
    bsp::can::Can*                         can = nullptr; ///< 归属总线，nullptr 表示空闲
    std::array<DJIMotor*, kMaxMotorPerCan> slots{};      ///< 下标为 id1 - 1
    size_t                                 count = 0;     ///< 已登记电机数
};

/// 全局发送槽位注册表，最多 CAN_DRIVER_MAX_INSTANCES 条总线。
/// 用 constinit 保证在任何电机对象构造前完成初始化，避免静态初始化顺序问题。
constinit std::array<TxSlots, CAN_DRIVER_MAX_INSTANCES> g_tx_slots{};

/// 按总线查找已登记的槽位条目；未登记返回 nullptr。
const TxSlots* find_tx_slots(const bsp::can::Can& can)
{
    for (const TxSlots& entry : g_tx_slots)
        if (entry.can == &can)
            return &entry;
    return nullptr;
}
} // namespace

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

    // 非法 id1 会让本对象永远匹配不到反馈，必须在注册回调前拒绝。
    if (!valid_id1(cfg_.id1))
        Error_Handler();

    const size_t slot = id_to_index(cfg_.id1);

    // 单次扫描：定位本总线条目，同时记录首个空闲总线槽位（避免二次查找）。
    TxSlots* bus    = nullptr;
    TxSlots* vacant = nullptr;
    for (TxSlots& entry : g_tx_slots)
    {
        if (entry.can == &cfg_.can)
        {
            bus = &entry;
            break;
        }
        if (vacant == nullptr && entry.can == nullptr)
            vacant = &entry;
    }

    // 已登记的总线沿用原条目，否则占用首个空闲槽位；两者皆无即容量耗尽。
    TxSlots* target = bus != nullptr ? bus : vacant;
    if (target == nullptr)
        Error_Handler();

    // 同一总线上的同一个 id1 只允许一台电机。
    if (target->slots[slot] != nullptr)
        Error_Handler();

    /* 先注册接收；失败时不得改动注册表，否则会残留指向未完成构造对象的槽位。 */
    if (cfg_.can.register_callback<&DJIMotor::on_can>(*this) != bsp::can::Status::Ok)
        Error_Handler();

    // 注册成功后才提交发送槽位。
    if (bus == nullptr)
        target->can = &cfg_.can;
    target->slots[slot] = this;
    ++target->count;
}

DJIMotor::~DJIMotor()
{
    // 析构只释放发送槽位与空出的总线条目，不注销接收回调。
    for (TxSlots& entry : g_tx_slots)
    {
        if (entry.can != &cfg_.can)
            continue;
        entry.slots[id_to_index(cfg_.id1)] = nullptr;
        if (--entry.count == 0)
            entry.can = nullptr; // 最后一台电机析构后释放总线槽位
        return;
    }
}

void DJIMotor::selfProtect()
{
    // 钩子执行时状态可能尚未提交；直接清缓存，不能经过公开 setCurrent() 门控。
    iq_cmd_ = 0;
}

void DJIMotor::selfDisable()
{
    selfProtect();
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
    // 只要收到反馈，就说明电机和总线当前仍然在线。
    watchdog_.feed();

    const float feedback_angle = static_cast<float>(read_int16(&data[0])) * 360.0f / 8192.0f;

    const float feedback_rpm = read_int16(&data[2]);

    // TODO: 堵转电流检测
    // const float feedback_current = (float)((int16_t)data[4] << 8 | data[5]) / 16384.0f * 20.0f;

    // DJI 反馈给的是电机反馈侧的单圈机械角度，需要结合相邻帧差分展开成连续角度。
    // 这里按 `feedback_angle_delta` 是否小于 `-180 deg` 或大于 `180 deg` 来判断是否跨过了
    // `0 deg`：
    // - `feedback_angle_delta < -180 deg`：说明角度从接近 `360 deg` 正向跨过了 `0 deg`
    // - `feedback_angle_delta > 180 deg`：说明角度从接近 `0 deg` 反向跨过了 `0 deg`
    //
    // 对当前已支持型号，在 `1 kHz` 反馈频率下，相邻两帧的最大转角为：
    // - M2006 和 M3508 减速后的输出最大速度约为 500 rpm
    // - M3508 内部减速比约为 3591 / 187 ~= 19.2
    //   则反馈侧最大速度约为 500 * 19.2 = 9600 rpm
    //   也就是 160 rps ~= 57600 deg/s，在 1 ms 内约转过 57.6 deg
    // - M2006 内部减速比为 36
    //   则反馈侧最大速度约为 500 * 36 = 18000 rpm
    //   也就是 300 rps = 108000 deg/s，在 1 ms 内约转过 108 deg
    //
    // 因此在 `1 kHz` 反馈下，相邻两帧的真实转角绝对值小于 `180 deg`：
    // - 不会把正常转动误判成一次过零
    // - `round_cnt` 每帧至多变化一次
    // - 不会出现“一帧跨过多圈”的情况
    //

    const float feedback_angle_delta = feedback_angle - feedback_.mech_angle;
    if (feedback_angle_delta < -180)
        feedback_.round_cnt++;
    if (feedback_angle_delta > 180)
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

void DJIMotor::on_can(const bsp::can::FrameView& frame) noexcept
{
    // DJI 反馈帧固定为标准 ID 的经典数据帧，负载 8 字节。
    if (frame.header.id_type != bsp::can::IdType::Standard ||
        frame.header.format != bsp::can::FrameFormat::Classic ||
        frame.header.type != bsp::can::FrameType::Data)
        return;
    if (frame.data.size() != 8)
        return;
    if (frame.header.id != kDjiFeedbackBase + cfg_.id1)
        return;

    decode(frame.data.data());
}

bsp::can::Status DJIMotor::CAN_FilterInit(bsp::can::Can& can, const uint32_t filter_index)
{
    // 掩码含义与旧实现一致：接受 0x200 ~ 0x20F（高 7 位匹配、低 4 位忽略）。
    const bsp::can::IdFilter filter{ .index   = filter_index,
                                     .id_type = bsp::can::IdType::Standard,
                                     .mode    = bsp::can::FilterMode::Mask,
                                     .action  = bsp::can::FilterAction::Fifo0,
                                     .id1     = kDjiFeedbackBase,
                                     .id2     = kDjiFeedbackMask };
    return can.configure_filter(filter);
}

bsp::can::SendResult DJIMotor::SendIqCommand(bsp::can::Can& can, const IqSetCMDGroup cmd_group)
{
    // 非法分组必须在索引槽位前拦截，避免越界读取。
    size_t   group_offset = 0;
    uint32_t frame_id     = 0x200u;
    switch (cmd_group)
    {
    case IqSetCMDGroup::IqCMDGroup_1_4:
        group_offset = 0;
        frame_id     = 0x200u;
        break;
    case IqSetCMDGroup::IqCMDGroup_5_8:
        group_offset = 4;
        frame_id     = 0x1FFu;
        break;
    default:
        return bsp::can::SendResult::InvalidFrame;
    }

    // 未登记的总线视为没有电机，仍发送全零帧。
    const TxSlots* bus = find_tx_slots(can);

    // 缺席、Protected 或 Disabled 的槽位保持零；Standalone 无需父控制器。
    uint8_t iq_data[8] = {};
    for (size_t j = 0; j < 4; ++j)
    {
        const DJIMotor* motor = bus != nullptr ? bus->slots[group_offset + j] : nullptr;
        if (motor == nullptr)
            continue;
        const auto state = motor->state();
        if (state != State::Standalone && state != State::Controlled)
            continue;

        const int16_t iq_cmd = motor->getIqCMD();
        iq_data[j * 2 + 1]   = static_cast<uint8_t>(iq_cmd & 0xFF);      // 电流值低 8 位
        iq_data[j * 2]       = static_cast<uint8_t>(iq_cmd >> 8 & 0xFF); // 电流值高 8 位
    }

    const bsp::can::FrameView frame{
        .header = { .id      = frame_id,
                    .id_type = bsp::can::IdType::Standard,
                    .format  = bsp::can::FrameFormat::Classic,
                    .type    = bsp::can::FrameType::Data },
        .data   = iq_data,
    };
    return can.send(frame);
}

} // namespace motors
