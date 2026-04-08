/**
 * @file    dji.cpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   DJI 电机驱动实现
 *
 * 主要完成三件事：
 * - 把 CAN 反馈帧映射到具体电机对象
 * - 把原始反馈换算成输出轴角度与速度
 * - 把多台电机的电流命令聚合打包发送
 */
#include "dji.hpp"
#include "can_driver.h"
#include <array>
#include <cstdint>

namespace motors
{

constexpr size_t   kMaxMotorPerCan = 8;     // DJI 标准反馈协议一条 CAN 最多挂 8 台电机
constexpr uint32_t kDjiCANIdMin    = 0x201; // 1 号电机反馈 ID
constexpr uint32_t kDjiCANIdMax    = 0x208; // 8 号电机反馈 ID

// 一条 CAN 总线上，反馈 ID 到电机对象的映射。
struct DJI_FeedbackMap
{
    CAN_HandleTypeDef*                     hcan = nullptr; ///< CAN 句柄
    std::array<DJIMotor*, kMaxMotorPerCan> motors{};       ///< 反馈槽位
};

// 全局固定数组，容量与底层 BSP 里的 CAN 数量一致。
static std::array<DJI_FeedbackMap, CAN_NUM> map{};

// 辅助函数：把外部习惯使用的 1~8 号编号转成数组下标 0~7。
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

// 查找某个 hcan 对应的映射表。
static DJI_FeedbackMap* find_map(const CAN_HandleTypeDef* hcan)
{
    for (auto& m : map)
    {
        if (m.hcan == hcan)
            return &m;
    }
    return nullptr;
}

// 把电机注册到对应 CAN 的映射表中，后续回调靠这里分发。
static bool register_motor(CAN_HandleTypeDef* hcan, const size_t id1, DJIMotor* motor)
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

// 从映射表中注销电机。
static bool unregister_motor(CAN_HandleTypeDef* hcan, const size_t id1)
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

// 根据反馈 ID 查找对应的电机对象。
static DJIMotor* get_motor(const CAN_HandleTypeDef* hcan, const CAN_RxHeaderTypeDef* header)
{
    if (header->IDE != CAN_ID_STD)
        return nullptr;
    const uint8_t id0 = header->StdId - kDjiCANIdMin;
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

    /* 注册回调分发表 */
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
    // 只要收到反馈，就说明电机和总线当前仍然在线。
    watchdog_.feed();

    const float feedback_angle = static_cast<float>(read_int16(&data[0])) * 360.0f / 8192.0f;

    const float feedback_rpm = read_int16(&data[2]);

    // TODO: 堵转电流检测
    // const float feedback_current = (float)((int16_t)data[4] << 8 | data[5]) / 16384.0f * 20.0f;

    // DJI 反馈给的是电机反馈侧的单圈机械角度，需要靠越界检测展开成连续角度。
    // 这里使用 90 / 270 deg 的阈值来判断是否跨过了 0 deg。
    //
    // 结合当前已支持型号和 1 kHz 反馈频率，先看相邻两帧的最大转角：
    // - M2006 和 M3508 减速后的输出最大速度约为 500 rpm
    // - M3508 内部减速比约为 3591 / 187 ~= 19.2
    //   则反馈侧最大速度约为 500 * 19.2 = 9600 rpm
    //   也就是 160 rps ~= 57600 deg/s，在 1 ms 内约转过 57.6 deg
    // - M2006 内部减速比为 36
    //   则反馈侧最大速度约为 500 * 36 = 18000 rpm
    //   也就是 300 rps = 108000 deg/s，在 1 ms 内约转过 108 deg
    //
    // 因此在 1 kHz 反馈下，相邻两帧之间都不可能跨过整整一圈，`round_cnt` 每帧至多变化一次，
    // 不会出现“一帧跨过多圈”的情况。
    //
    // TODO: fixbug: 对 M2006 而言，理论单帧最大转角约为 108 deg，已经大于当前 90 deg 的判定裕量；
    // 当前 90 / 270 deg
    // 阈值在极限工况下可能仍有漏判一次过零的风险。这里按要求只补说明，不改原逻辑。
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
    // TODO: fixbug: 当前没有检查 m 是否为空；若 hcan 上还没注册任何 DJI 电机就调用此函数，
    // 后面访问 m->motors 会解引用空指针。这里按要求只补注释，不改原逻辑。

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
    if (header->StdId < kDjiCANIdMin || header->StdId > kDjiCANIdMax)
        return;
    DJIMotor* motor = get_motor(hcan, header);
    if (motor != nullptr)
        motor->decode(data);
}

extern "C" void DJI_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan)
{
    // FIFO 里可能一次堆了多帧，这里用 do-while 全部取完，避免漏帧。
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
    // FIFO1 的处理逻辑与 FIFO0 相同，只是读取的硬件 FIFO 不同。
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
