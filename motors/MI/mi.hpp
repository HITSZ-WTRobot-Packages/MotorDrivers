/**
 * @file    mi.hpp
 * @author  luoyue
 * @date    2026-05-11
 * @brief   小米 CyberGear 微电机驱动封装
 *
 * CyberGear 使用 CAN 2.0B 扩展帧，默认波特率 1 Mbps。
 * 电机上电后默认进入运控模式（MIT），通过 Type 1 帧下发位置/速度/力矩/Kp/Kd。
 *
 * 通信协议概要（扩展帧 ID 29-bit 布局）：
 * - bit28~24 = 通信类型（5 bits）
 * - bit23~8  = 数据区2（16 bits）
 * - bit7~0   = 电机 CAN ID（8 bits，出厂默认 127）
 *
 * 通信类型：
 * - Type 1：MIT 运控指令，data2 承载力矩，data1 承载位置/速度/Kp/Kd
 * - Type 2：电机反馈（角度 / 速度 / 力矩 / 温度），data2 承载状态与故障
 * - Type 3：使能
 * - Type 4：失能
 * - Type 6：设置机械零位（掉电丢失）
 * - Type 17 / 18：参数读写
 *
 * 参考 DM 驱动的封装风格：反馈通过 FixedPointerMap 分发到具体电机对象，
 * 控制指令由电机对象自行打包发送。控制权通过 tryAcquireController /
 * releaseController 管理，获取控制权时自动使能电机。
 */
#pragma once

#include "can_driver.h"
#include "motor_if.hpp"
#include "watchdog.hpp"

#ifndef MOTORS_MI_MAX_NUM
#    define MOTORS_MI_MAX_NUM (8)
#endif

namespace motors
{

/**
 * @brief 小米 CyberGear 电机驱动对象
 *
 * 负责：
 * - 注册到某条 CAN 总线的反馈映射表
 * - 解析 Type 2 反馈报文，展开多圈连续角度
 * - 打包并发送 MIT 控制指令（Type 1）
 * - 发送使能 / 失能 / 归零指令
 * - 提供参数读写接口（Type 17 / 18）
 *
 * 当前库约定：
 * - 电机上电后默认处于 MIT 运控模式
 * - 角度对外统一使用 deg，速度使用 rpm，力矩使用 Nm
 * - MIT 接口的 v_ref 使用 deg/s（MIT 语义下的速度前馈，非普通转速指令）
 */
class MIMotor final : public IMotor
{
public:
    /**
     * @brief CyberGear 运控模式
     *
     * 对应驱动器内部 run_mode 参数（index 0x7005）。
     */
    enum class Mode : uint8_t
    {
        MIT = 0, ///< 运控模式（默认），通过 Type 1 帧下发完整 MIT 指令
        Pos = 1, ///< 位置模式，写入 loc_ref（0x7016）
        Vel = 2, ///< 速度模式，写入 spd_ref（0x700A）
        Cur = 3, ///< 电流模式，写入 iq_ref（0x7006）
    };

    /**
     * @brief 反馈状态码
     *
     * 从反馈帧 Extended ID 的 bit22~23 提取。
     */
    enum class RunState : uint8_t
    {
        Reset = 0, ///< 复位状态
        Cali  = 1, ///< 标定状态
        Run   = 2, ///< 运行状态
    };

    /**
     * @brief 故障标志位（bit16~21 of Extended ID）
     */
    enum Fault : uint16_t
    {
        FaultNone          = 0,
        FaultUnderVoltage  = 1 << 0, ///< 欠压
        FaultOverCurrent   = 1 << 1, ///< 过流
        FaultOverTemp      = 1 << 2, ///< 过温
        FaultMagEncoder    = 1 << 3, ///< 磁编码器故障
        FaultHallEncoder   = 1 << 4, ///< HALL 编码器故障
        FaultNotCalibrated = 1 << 5, ///< 未标定
    };

    /**
     * @brief CyberGear 电机配置
     */
    struct Config
    {
        CAN_HandleTypeDef* hcan;      ///< 所在 CAN 总线
        uint8_t            id;        ///< 电机 CAN ID，范围 0~127，出厂默认 127
        uint8_t            master_id; ///< 主机 CAN ID，默认 0

        bool  auto_zero      = true;  ///< 上电收到足够稳定反馈后，是否自动把当前角度设为零点
        bool  reverse        = false; ///< 是否把输出方向整体反向
        float reduction_rate = 1.0f;  ///< 外接减速比
    };

    explicit MIMotor(const Config& cfg);
    ~MIMotor() override;

    // ---- IMotor 反馈接口 ----

    [[nodiscard]] float getAngle() const override { return abs_angle_; }
    [[nodiscard]] float getVelocity() const override { return velocity_; }
    void               resetAngle() override;
    [[nodiscard]] bool isConnected() const override { return watchdog_.isFed(); }

    // ---- IMotor 控制能力声明 ----

    [[nodiscard]] controllers::ControlMode defaultControlMode() const override
    {
        return controllers::ControlMode::InternalMIT;
    }

    [[nodiscard]] bool supportsCurrent() const override { return true; }
    /**
     * @brief 低层力矩输入
     *
     * 在 MIT 模式下退化为纯力矩控制：发包时 Kp=Kd=0，仅写入力矩前馈。
     * @param current 力矩，单位 Nm
     */
    void setCurrent(float current) override;

    [[nodiscard]] bool supportsInternalMIT() const override { return true; }
    /**
     * @brief 发送 MIT 控制指令（Type 1）
     *
     * 一次将位置、速度、力矩、刚度、阻尼打包下发。
     *
     * 注意这里的 v_ref 不是普通速度模式下的目标转速，而是 MIT 五元组里的速度前馈项，
     * 通常与 p_ref 配套使用，可理解为位置参考的导数。因此单位采用 deg/s 而非 rpm。
     *
     * @param t_ff 前馈力矩，单位 Nm，范围 -12 ~ +12
     * @param p_ref 位置参考，单位 deg
     * @param v_ref 速度参考，单位 deg/s（MIT 语义下的速度前馈）
     * @param kp 刚度，范围 0.0 ~ 500.0
     * @param kd 阻尼，范围 0.0 ~ 5.0
     */
    void setInternalMIT(float t_ff, float p_ref, float v_ref, float kp, float kd) override;

    // ---- 电机控制 ----

    /**
     * @brief 发送使能报文（Type 3）
     * @return 是否发送成功
     */
    bool enable();
    /**
     * @brief 发送失能报文（Type 4）
     * @param clear_fault 是否同时清除故障
     * @return 是否发送成功
     */
    bool disable(bool clear_fault = false);
    /**
     * @brief 设置当前角度为机械零位（Type 6，掉电丢失）
     * @return 是否发送成功
     */
    bool setMechanicalZero();

    /**
     * @brief 心跳保活
     *
     * 未使能时用失能包代替心跳，维持通信但不使能电机。
     */
    void ping()
    {
        if (!enabled_)
            disable();
    }

    // ---- 参数读写 ----

    /**
     * @brief 读取电机内部参数（Type 17）
     * @param index 参数索引号
     * @param value [out] 读回的 float 值
     * @return 是否发送成功
     */
    bool readParam(uint16_t index, float& value);
    /**
     * @brief 写入电机内部参数（Type 18，掉电丢失）
     * @param index 参数索引号
     * @param value 写入的 float 值
     * @return 是否发送成功
     */
    bool writeParam(uint16_t index, float value);

    // ---- 状态查询 ----

    [[nodiscard]] float    temperature() const { return feedback_.temperature; }
    [[nodiscard]] float    torque() const { return feedback_.torque; }
    [[nodiscard]] RunState runState() const { return feedback_.run_state; }
    [[nodiscard]] uint16_t fault() const { return feedback_.fault; }

    // ---- 控制权 ----

    bool tryAcquireController(controllers::IController* ctrl) override;
    void releaseController(controllers::IController* ctrl) override;

    // ---- CAN 静态方法 ----

    /**
     * @brief 初始化 CyberGear 反馈对应的 CAN 滤波器
     *
     * 接收所有扩展帧，由软件层按 Type 和 ID 分发。
     */
    static void CAN_FilterInit(CAN_HandleTypeDef* hcan, uint32_t filter_bank);
    /**
     * @brief 统一 CAN 接收入口
     *
     * 如果项目里已有 CAN 分发器，推荐注册到分发器中；
     * 否则可在 HAL FIFO 回调里直接调用。
     */
    static void CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                       const CAN_RxHeaderTypeDef* header,
                                       const uint8_t*             data);

    // ---- 协议常量（mi.cpp 换算函数需要引用） ----
    static constexpr float kPosMaxRad = 12.5f;  ///< 位置量程，单位 rad（对应 P_MIN=-12.5, P_MAX=12.5）
    static constexpr float kVelMaxRps = 30.0f;  ///< 速度量程，单位 rad/s
    static constexpr float kTorqueMax = 12.0f;  ///< 力矩量程，单位 Nm
    static constexpr float kKpMax     = 500.0f; ///< Kp 量程
    static constexpr float kKdMax     = 5.0f;   ///< Kd 量程

private:
    /**
     * @brief 解码 Type 2 反馈帧
     */
    void decode(const uint8_t data[8], uint16_t fault, RunState run_state);

    /**
     * @brief 生成发送报文头
     * @param comm_type 通信类型（bit28~24）
     * @param data_area2 数据区2（bit23~8）
     */
    [[nodiscard]] CAN_TxHeaderTypeDef tx_header(uint8_t comm_type, uint16_t data_area2) const;

    /**
     * @brief 发送使能 / 失能 / 归零内部实现
     */
    bool sendEnableDisable(uint8_t comm_type, uint8_t data0);

    Config cfg_;

    bool enabled_ = false; ///< 是否已发送使能报文

    volatile bool param_read_pending_ = false;
    volatile bool param_read_valid_   = false;
    uint16_t      param_read_index_   = 0;
    float         param_read_value_   = 0.0f;

    service::Watchdog watchdog_;
    uint32_t           feedback_count_ = 0; ///< 反馈计数，用于上电自动归零

    struct Feedback
    {
        float    angle{ 0 };       ///< 电机当前位置（单圈范围内），单位 deg
        float    velocity{ 0 };    ///< 电机侧速度，单位 rpm
        float    torque{ 0 };      ///< 反馈力矩，单位 Nm
        float    temperature{ 0 }; ///< 电机温度，单位 degC
        RunState run_state{ RunState::Reset };
        uint16_t fault{ 0 };

        int32_t round_cnt{ 0 }; ///< 跨圈累计，用于展开成连续角度
    } feedback_{};

    float angle_zero_        = 0.0f; ///< 零点角度，单位 deg
    float abs_angle_         = 0.0f; ///< 输出轴绝对角度，单位 deg
    float velocity_          = 0.0f; ///< 输出轴速度，单位 rpm
    float inv_reduction_rate_;       ///< 外接减速比倒数
    float sign_;                     ///< 方向符号，正转为 1，反转为 -1
};

} // namespace motors

extern "C"
{
/**
 * @brief MI FIFO0 中断回调包装
 *
 * 如果项目没有统一 CAN 分发器，可直接使用此 HAL 包装；
 * 否则推荐统一把报文分发到 CANBaseReceiveCallback()。
 */
void MI_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
/**
 * @brief MI FIFO1 中断回调包装
 *
 * 如果项目没有统一 CAN 分发器，可直接使用此 HAL 包装；
 * 否则推荐统一把报文分发到 CANBaseReceiveCallback()。
 */
void MI_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
}
