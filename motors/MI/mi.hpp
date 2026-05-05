/**
 * @file    mi.hpp
 * @author  Luoyue777
 * @date    2026-05-05
 * @brief   小米 CyberGear 微电机驱动封装
 *
 * 小米电机使用 CAN 扩展帧（29-bit ID）通信，与 DJI/DM 的标准帧不同。
 * 扩展 ID 位域布局：motor_id[7:0] | data[15:0] | mode[4:0] | res[2:0]
 *
 * 主要通信类型（mode 字段）：
 * - 0：获取设备 ID
 * - 1：运控模式指令（同时下发力矩、位置、速度、Kp、Kd）
 * - 2：电机自动反馈（角度、速度、力矩、温度）
 * - 3：使能
 * - 4：停止
 * - 6：设置机械零位
 * - 7：更改 CAN ID
 * - 0x11：读取单个参数
 * - 0x12：写入单个参数
 *
 * 运行模式：
 * - MIT（运控模式）：通过 mode=1 帧直接下发五元组控制量
 * - Pos（位置模式）：通过参数 0x7016 写入目标位置
 * - Spd（速度模式）：通过参数 0x700A 写入目标速度
 * - Cur（电流模式）：MIT 退化，仅使用前馈力矩
 *
 * 单位约定遵循库标准：角度 deg、速度 rpm、力矩 Nm。
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
 * @brief 小米 CyberGear 电机对象
 *
 * 负责：
 * - 注册到 CAN 总线的反馈映射表
 * - 解析扩展帧反馈报文（mode 2 自动反馈、mode 0x11 参数读取响应）
 * - 发送 MIT / 位置 / 速度 / 力矩控制指令
 * - 使能 / 失能 / 心跳维持
 */
class MIMotor final : public IMotor
{
public:
    /**
     * @brief 已封装的电机型号
     */
    enum class Type
    {
        CyberGear, ///< 小米 CyberGear 微电机

        MotorTypeCount, ///< 类型计数，占位用
    };

    /**
     * @brief 电机运行模式
     *
     * 对应驱动器内部模式切换，通过参数 0x7005 写入。
     */
    enum class RunMode : uint8_t
    {
        MIT = 0x00, ///< 运控模式
        Pos = 0x01, ///< 位置模式
        Spd = 0x02, ///< 速度模式
        Cur = 0x03, ///< 电流模式
    };

    /**
     * @brief 电机故障状态码
     */
    enum class State : uint8_t
    {
        OK            = 0, ///< 无故障
        BatLow        = 1, ///< 欠压故障
        OverCurrent   = 2, ///< 过流
        OverTemp      = 3, ///< 过温
        MagneticErr   = 4, ///< 磁编码故障
        HallErr       = 5, ///< HALL 编码故障
        NoCalibration = 6, ///< 未标定
    };

    /**
     * @brief 小米电机配置
     */
    struct Config
    {
        CAN_HandleTypeDef* hcan;      ///< 所在 CAN 总线
        uint8_t            id;        ///< 电机 CAN ID（1~127）
        uint8_t            master_id; ///< 主机 ID，默认 0xF4；所有 MI 电机共享
        Type               type;      ///< 电机型号
        RunMode            run_mode;  ///< 驱动器运行模式

        bool  auto_zero      = true;  ///< 上电收够 50 帧反馈后自动把当前角度设为零点
        bool  reverse        = false; ///< 是否反转输出方向
        float reduction_rate = 1.0f;  ///< 外接减速比

        /// @name 量程参数
        /// 协议里角度反馈范围为 ±4π rad，指令限幅默认使用 ±12.5 rad。
        /// @{
        float pos_max_rad = 12.5f; ///< 位置量程上限，单位 rad
        float vel_max_rad = 30.0f; ///< 速度量程上限，单位 rad/s
        float tor_max     = 12.0f; ///< 力矩量程上限，单位 Nm
        float kp_max      = 500.0f; ///< Kp 上限
        float kd_max      = 5.0f;   ///< Kd 上限
        /// @}
    };

    explicit MIMotor(const Config& cfg);
    ~MIMotor() override;

    /**
     * @brief 获取输出轴角度
     * @return 角度，单位 deg
     */
    [[nodiscard]] float getAngle() const override { return abs_angle_; }
    /**
     * @brief 获取输出轴速度
     * @return 速度，单位 rpm
     */
    [[nodiscard]] float getVelocity() const override { return velocity_; }
    /**
     * @brief 把当前反馈角度重置为零点
     */
    void resetAngle() override;

    [[nodiscard]] bool isConnected() const override { return watchdog_.isFed(); }

    /**
     * @brief 返回该电机推荐的默认控制模式
     *
     * MIT 模式 → InternalMIT，位置模式 → InternalPos，速度模式 → InternalVel，
     * 电流模式 → ExternalPID。
     */
    [[nodiscard]] controllers::ControlMode defaultControlMode() const override;

    /**
     * @brief 是否支持电流 / 力矩输入
     *
     * MI 始终可以通过 MIT 控制帧退化为纯力矩输入（kp=kd=pos=vel=0）。
     */
    [[nodiscard]] bool supportsCurrent() const override { return true; }
    /**
     * @brief 下发力矩指令
     *
     * 通过 MIT 控制帧退化实现：kp=kd=0、位置参考=0、速度参考=0，仅传递前馈力矩。
     * @param current 力矩，单位 Nm
     */
    void setCurrent(float current) override;

    /**
     * @brief 是否支持内部速度控制
     */
    [[nodiscard]] bool supportsInternalVelocity() const override
    {
        return cfg_.run_mode == RunMode::Spd;
    }
    /**
     * @brief 发送内部速度指令
     *
     * 通过参数 0x700A 写入目标速度。
     * @param rpm 速度参考，单位 rpm
     */
    void setInternalVelocity(float rpm) override;

    /**
     * @brief 是否支持内部位置控制
     */
    [[nodiscard]] bool supportsInternalPosition() const override
    {
        return cfg_.run_mode == RunMode::Pos;
    }
    /**
     * @brief 发送内部位置指令
     *
     * 通过参数 0x7016 写入目标位置。
     * @param pos 位置参考，单位 deg
     */
    void setInternalPosition(float pos) override;

    /**
     * @brief 是否支持 MIT 控制
     */
    [[nodiscard]] bool supportsInternalMIT() const override
    {
        return cfg_.run_mode == RunMode::MIT;
    }
    /**
     * @brief 发送 MIT 控制指令
     *
     * 通过 mode=1 运控帧一次下发五元组：力矩、位置、速度、Kp、Kd。
     *
     * @param t_ff 前馈力矩，单位 Nm
     * @param p_ref 位置参考，单位 deg
     * @param v_ref 速度参考，单位 deg/s
     * @param kp 位置刚度
     * @param kd 速度阻尼
     */
    void setInternalMIT(float t_ff, float p_ref, float v_ref, float kp, float kd) override;

    /**
     * @brief 解码一帧 MI 反馈报文
     * @param header CAN 报文头（扩展帧）
     * @param data  8 字节数据段
     */
    void decode(const CAN_RxHeaderTypeDef* header, const uint8_t data[8]);

    /**
     * @brief 获取控制权时，同时尝试使能电机
     */
    bool tryAcquireController(controllers::IController* ctrl) override;
    /**
     * @brief 释放控制权
     */
    void releaseController(controllers::IController* ctrl) override;

    /**
     * @brief 发送使能序列：停止 → 设运行模式 → 使能
     * @return 是否全部发送成功
     */
    bool enable();
    /**
     * @brief 发送停止帧
     * @return 是否发送成功
     */
    bool disable();
    /**
     * @brief 心跳维持
     *
     * 未使能时发送停止帧维持通信，使能后电机自动上报反馈无需额外心跳。
     */
    void ping()
    {
        if (!enabled_)
            disable();
    }

    /**
     * @brief 最近一次反馈的状态码
     */
    [[nodiscard]] State state() const { return feedback_.state; }

    /**
     * @brief 最近一次反馈的电机温度
     * @return 温度，单位 ℃
     */
    [[nodiscard]] float temperature() const { return feedback_.temperature; }

    /**
     * @brief 初始化 MI 反馈对应的 CAN 滤波器
     *
     * 配置为接收所有目标为 master_id 的扩展帧。
     * @param hcan        CAN 句柄
     * @param filter_bank 滤波器编号
     * @param master_id   本机主控 ID
     */
    static void CAN_FilterInit(CAN_HandleTypeDef* hcan, uint32_t filter_bank, uint32_t master_id);
    /**
     * @brief MI 统一 CAN 接收入口
     *
     * 如果项目里已有统一 CAN 分发器，推荐注册到分发器；否则在 HAL FIFO 回调里直接调用。
     */
    static void CANBaseReceiveCallback(const CAN_HandleTypeDef*   hcan,
                                       const CAN_RxHeaderTypeDef* header,
                                       const uint8_t*             data);

private:
    Config cfg_; ///< 构造时保存的配置

    bool enabled_{ false };

    uint32_t          feedback_count_ = 0; ///< 反馈计数，用于上电自动清零
    service::Watchdog watchdog_;

    struct
    {
        float angle{ 0 };       ///< 电机侧角度，单位 deg
        float velocity{ 0 };    ///< 电机侧速度，单位 rpm
        float torque{ 0 };      ///< 反馈力矩，单位 Nm
        float temperature{ 0 }; ///< 电机温度，单位 ℃
        State state{ State::OK }; ///< 故障状态
    } feedback_{};

    float angle_zero_{ 0 };    ///< 零点角度，单位 deg
    float abs_angle_{ 0 };     ///< 输出轴绝对角度，单位 deg
    float velocity_{ 0 };      ///< 输出轴速度，单位 rpm
    float inv_reduction_rate_; ///< 总减速比的倒数
    float sign_;               ///< 方向符号，正转为 1，反转为 -1

    // ---- 协议常量 ----

    /// @name CAN 通信类型（扩展 ID mode 字段）
    /// @{
    enum MsgMode : uint8_t
    {
        GetDeviceID = 0,   ///< 获取设备 ID
        Control     = 1,   ///< 运控指令
        Feedback    = 2,   ///< 自动反馈
        EnableFrame = 3,   ///< 使能
        StopFrame   = 4,   ///< 停止
        SetZero     = 6,   ///< 设置机械零位
        ChangeID    = 7,   ///< 更改 CAN ID
        ReadParam   = 0x11, ///< 读取单个参数
        WriteParam  = 0x12, ///< 写入单个参数
    };
    /// @}

    /// @name 功能码索引
    /// @{
    static constexpr uint16_t INDEX_RUN_MODE  = 0x7005; ///< 运行模式
    static constexpr uint16_t INDEX_SPEED_CMD = 0x700A; ///< 速度指令
    static constexpr uint16_t INDEX_POS_CMD   = 0x7016; ///< 位置指令
    static constexpr uint16_t INDEX_SPD_LIMIT = 0x7017; ///< 位置模式限速
    static constexpr uint16_t INDEX_CUR_LIMIT = 0x7018; ///< 速度模式限流
    static constexpr uint16_t INDEX_MECH_POS  = 0x7019; ///< 机械位置
    static constexpr uint16_t INDEX_MECH_VEL  = 0x701B; ///< 机械速度
    /// @}

    /// @brief 扩展 ID 打包：motor_id | (data << 8) | (mode << 24)
    static constexpr uint32_t packExtId(uint8_t motor_id, uint16_t data, uint8_t mode)
    {
        return static_cast<uint32_t>(motor_id) | (static_cast<uint32_t>(data) << 8) |
               (static_cast<uint32_t>(mode) << 24);
    }
    static constexpr uint16_t getExtIdData(uint32_t ext_id) { return (ext_id >> 8) & 0xFFFF; }
    static constexpr uint8_t  getExtIdMode(uint32_t ext_id) { return (ext_id >> 24) & 0x1F; }

    /**
     * @brief 发送一帧 CAN 扩展帧
     * @param mode       通信类型
     * @param data_field 扩展 ID 的 data 字段（16-bit）
     * @param payload    8 字节数据段
     */
    void sendFrame(MsgMode mode, uint16_t data_field, const uint8_t payload[8]);

    /**
     * @brief 写入单个参数（通信类型 0x12）
     * @param index 功能码
     * @param value 4 字节参数值
     */
    void writeParam(uint16_t index, const uint8_t value[4]);

    /**
     * @brief 切换运行模式
     */
    void setRunMode(RunMode mode);
};

} // namespace motors

extern "C"
{
/**
 * @brief MI FIFO0 中断回调包装
 *
 * 如果项目没有统一 CAN 分发器，可以直接使用这个 HAL 包装；否则推荐统一分发到
 * `CANBaseReceiveCallback()`。
 */
void MI_CAN_Fifo0ReceiveCallback(CAN_HandleTypeDef* hcan);
/**
 * @brief MI FIFO1 中断回调包装
 */
void MI_CAN_Fifo1ReceiveCallback(CAN_HandleTypeDef* hcan);
}
