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

/**
 * @brief 达妙（DM）电机驱动对象
 *
 * DM 驱动支持多种控制模式：
 * - MIT
 * - 内部位置
 * - 内部速度
 *
 * 这类驱动对参数量程比较敏感，因此配置中的量程必须和上位机设定一致，
 * 否则反馈换算和指令打包都会出错。
 *
 * 当前库对 DM 还额外有两个明确约束：
 * - 所有 DM 电机共享同一个 `master_id`
 * - `id0` 建议使用 `0x09 ~ 0x0F`，以便避开 DJI 的标准 ID 区间
 */
class DMMotor : public IMotor
{
public:
    /**
     * @brief 已封装的 DM 电机类型
     */
    enum class Type
    {
        J4310_2EC,   ///< J4310-2EC
        J10010L_2EC, ///< J10010L-2EC
        S3519,       ///< S3519
        S2325_1EC,   ///< S2325-1EC + 3520-1EC(驱动器)

        MotorTypeCount, ///< 类型计数，占位用
    };
    /**
     * @brief DM 驱动器内部控制模式
     *
     * 这里不是 MCU 侧控制器模式，而是驱动器固件当前配置成哪种协议解释方式。
     * 发送时标准帧 ID 为“模式基值 + 电机 ID”：
     * - `Vel = 0x200 + ID`
     * - `Pos = 0x100 + ID`
     * - `MIT = 0x000 + ID`
     */
    enum class Mode
    {
        MIT = 0x000, ///< MIT 模式
        Pos = 0x100, ///< 内部位置模式
        Vel = 0x200, ///< 内部速度模式
    };
    /**
     * @brief 达妙反馈状态码
     */
    enum class State : uint8_t
    {
        Disabled             = 0x0U, // 失能
        Enabled              = 0x1U, // 使能
        OverVoltage          = 0x8U, // 超压
        UnderVoltage         = 0x9U, // 欠压
        OverCurrent          = 0xAU, // 过电流
        MosOverheating       = 0xBU, // MOS 过温
        MotorCoilOverheating = 0xCU, // 电机线圈过温
        Disconnected         = 0xDU, // 通讯丢失
        Overload             = 0xEU, // 过载
    };

    /**
     * @brief 达妙电机配置
     */
    struct Config
    {
        CAN_HandleTypeDef* hcan;        ///< 所在 CAN 总线
        uint8_t            id0;         ///< 电调 ID，仅低 4 位有效，建议从 0x09 到 0x0F
        Type               type;        ///< 电机类型
        Mode               mode;        ///< 驱动器控制模式，需要和上位机配置一致
        float              pos_max_rad; ///< 驱动器配置的位置最大值，单位 rad
        float              vel_max_rad; ///< 驱动器配置的速度最大值，单位 rad/s
        float              tor_max;     ///< 驱动器配置的力矩最大值，单位 Nm

        bool  auto_zero      = true;  ///< 上电收够一段稳定反馈后，是否自动把当前角度设为零点
        bool  reverse        = false; ///< 是否反转输出方向
        float reduction_rate = 1.0f;  ///< 外接减速比
    };

    explicit DMMotor(const Config& cfg);
    ~DMMotor() override;

    /**
     * @brief 获取输出轴角度
     * @return 角度，单位 deg
     */
    [[nodiscard]] float getAngle() const override { return abs_angle_; }
    /**
     * @brief 获取输出轴角速度
     * @return 速度；DM 对外接口单位按库约定使用 rpm
     */
    [[nodiscard]] float getVelocity() const override { return velocity_; }
    /**
     * @brief 把当前输出角度设置为零点
     */
    void resetAngle() override;

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

    /**
     * @brief 解码一帧 DM 反馈
     */
    void decode(const uint8_t data[8]);

    [[nodiscard]] bool isConnected() const override { return watchdog_.isFed(); }

    [[nodiscard]] bool supportsCurrent() const override { return cfg_.mode == Mode::MIT; }
    /**
     * @brief 低层输出接口
     *
     * 当 DM 运行在 MIT 模式时，这个接口会把 MIT 退化为力矩输入使用。
     * 因此这里的入参语义应理解为力矩，单位 Nm。
     */
    void setCurrent(float current) override;

    [[nodiscard]] bool supportsInternalVelocity() const override { return cfg_.mode == Mode::Vel; }
    /**
     * @brief 发送内部速度参考
     *
     * 库设计上对外统一使用 rpm；DM 驱动内部会把它换算成协议要求的 rad/s。
     */
    void setInternalVelocity(float rpm) override;

    [[nodiscard]] bool supportsInternalPosition() const override { return cfg_.mode == Mode::Pos; }
    /**
     * @brief 发送内部位置参考
     *
     * 当前位置参考使用 deg；DM 协议内部会换算成 rad。
     */
    void setInternalPosition(float pos) override;

    [[nodiscard]] bool supportsInternalMIT() const override { return cfg_.mode == Mode::MIT; }
    /**
     * @brief 发送 MIT 控制指令
     *
     * 当前实现约定：
     * - `p_ref` 使用 deg
     * - `v_ref` 使用 deg/s
     * - `t_ff` 使用 Nm
     *
     * 这里的 `v_ref` 不是普通速度模式下的目标转速，而是 MIT 五元组里的速度参考项，通常和 `p_ref`
     * 配套使用，可理解为 `p_ref` 的导数。
     *
     * 也正因为如此，`setInternalMIT()` 的单位约定和普通 `setInternalVelocity()` 不同：前者当前用
     * `deg + deg/s` 这一组量，再在驱动内部换算成协议要求的 `rad + rad/s`。
     */
    void setInternalMIT(float t_ff, float p_ref, float v_ref, float kp, float kd) override;

    /**
     * @brief 初始化 DM 反馈的 CAN 滤波器
     * @param hcan CAN 句柄
     * @param filter_bank 滤波器编号
     * @param master_id 本机主控 ID，用于接收属于自己的反馈；当前库要求所有 DM 电机共用同一个值
     */
    static void CAN_FilterInit(CAN_HandleTypeDef* hcan, uint32_t filter_bank, uint32_t master_id);
    /**
     * @brief DM 的统一 CAN 接收入口
     *
     * 如果项目里已经有统一 CAN 分发器，推荐把它注册到分发器中；否则也可以在 HAL 的 FIFO
     * 回调里直接调用它。
     */
    static void CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                       const CAN_RxHeaderTypeDef* header,
                                       const uint8_t*             data);

    /**
     * @brief 获取控制权时，同时尝试使能电机
     */
    bool tryAcquireController(controllers::IController* ctrl) override;
    /**
     * @brief 释放控制权
     */
    void releaseController(controllers::IController* ctrl) override;

    /**
     * @brief 发送使能报文
     * @return 是否发送成功
     */
    bool enable();
    /**
     * @brief 发送失能报文
     * @return 是否发送成功
     */
    bool disable();
    void ping()
    {
        // DM 是“一收一回”模式，未使能时库内有意使用失能包代替心跳包。
        // 这样既能维持通信，又不会误使能电机。
        if (!enabled_)
            disable();
    }

    /**
     * @brief 最近一次反馈出来的状态码
     */
    [[nodiscard]] State state() const { return feedback_.state; }

private:
    Config cfg_; ///< 构造时保存的配置

    bool enabled_{ false }; ///< DM 驱动器是否已发送使能报文
                            ///< TODO: 这里通过 DM 反馈判定是否真的使能成功

    uint32_t          feedback_count_ = 0; ///< 反馈计数，用于上电自动清零
    service::Watchdog watchdog_;
    struct
    {
        float angle{ 0 };      ///< 当前圈内角度，单位 deg
        float velocity{ 0 };   ///< 电机侧速度，库内统一按 rpm 保存
        float torque{ 0 };     ///< 反馈力矩，单位 Nm
        float temp_mos{ 0 };   ///< MOS 温度
        float temp_rotor{ 0 }; ///< 转子温度

        State state{ 0 }; ///< 驱动器状态码

        int32_t count{ 0 }; ///< 圈数累计，用于展开成连续角度
    } feedback_{};
    float angle_zero_{ 0 };    ///< 零点角度，单位 deg
    float inv_reduction_rate_; ///< 外接减速比倒数
    float pos_max_deg;         ///< 位置量程上限，单位 deg

    float abs_angle_ = 0; ///< 输出轴绝对角度，单位 deg
    float velocity_  = 0; ///< 输出轴速度，库内统一按 rpm 保存

    float sign_; ///< 方向符号，正转为 1，反转为 -1

    /**
     * @brief 生成发送报文头
     * @param DLC 数据长度
     */
    [[nodiscard]] CAN_TxHeaderTypeDef tx_header(const uint8_t& DLC) const;

    // DM 协议定义好的固定使能 / 失能指令。
    static constexpr uint8_t ENABLE_MSG[]  = { 0xFF, 0XFF, 0XFF, 0xFF, 0XFF, 0XFF, 0XFF, 0XFC };
    static constexpr uint8_t DISABLE_MSG[] = { 0xFF, 0XFF, 0XFF, 0xFF, 0XFF, 0XFF, 0XFF, 0XFD };
};

} // namespace motors

extern "C"
{
/**
 * @brief DM FIFO0 中断回调包装
 *
 * 如果项目没有统一 CAN 分发器，可以直接使用这个 HAL 包装；否则更推荐统一把报文分发到
 * `CANBaseReceiveCallback()`。
 */
void DM_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
/**
 * @brief DM FIFO1 中断回调包装
 *
 * 如果项目没有统一 CAN 分发器，可以直接使用这个 HAL 包装；否则更推荐统一把报文分发到
 * `CANBaseReceiveCallback()`。
 */
void DM_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
}
