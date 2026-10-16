/**
 * @file    dji.hpp
 * @author  syhanjin
 * @date    2026-01-28
 * @brief   DJI 电机驱动封装
 *
 * 这里封装的是 DJI 常见 CAN 电机反馈与电流控制协议。
 *
 * 从能力设计上看，DJI 这一路主要提供：
 * - 角度反馈
 * - 速度反馈
 * - 低层电流输入
 *
 * 和 DM / VESC 不同，DJI 电机的电流命令不是“调用一次就马上发一帧”，
 * 而是先写入各个电机对象的目标值，再由 `SendIqCommand()` 按 4 台电机一组统一打包发送。
 *
 * 当前 DJI 实现里，速度接口单位为 rpm。
 */
#pragma once

#include "can_driver.hpp"
#include "motor_if.hpp"
#include "watchdog.hpp"

namespace motors
{

/**
 * @brief DJI 电机对象
 *
 * 负责：
 * - 构造时按 `Config.id1` 自动登记到所在 `bsp::can::Can` 总线的固定发送槽位，
 *   并注册本对象的成员接收回调（接收只由总线广播，不经过发送槽位）
 * - 解析 0x201 ~ 0x208 的反馈报文
 * - 保存零点、圈数、速度等状态
 * - 提供聚合发送所需的目标电流缓存
 *
 * 生命周期：`bsp::can::Can` 不支持注销回调，因此本对象必须在所在总线
 * `Can::start()` 之前构造，并存活到该总线不再使用为止；析构只释放发送槽位，
 * 不注销接收回调。本类不可复制、不可移动，因为回调借用本对象地址。
 */
class DJIMotor final : public IMotor
{
public:
    /**
     * @brief 电流指令发送分组
     *
     * DJI 协议一次报文能携带 4 台电机的电流命令，因此按 1~4、5~8 分成两组。
     */
    enum class IqSetCMDGroup : size_t
    {
        IqCMDGroup_1_4 = 0U, ///< 控制 1~4 号电机
        IqCMDGroup_5_8 = 4U, ///< 控制 5~8 号电机
    };

    /**
     * @brief 已封装的 DJI 电机类型
     */
    enum class Type
    {
        M3508_C620 = 0U, ///< M3508 + C620
        M2006_C610,      ///< M2006 + C610

        MOTOR_TYPE_COUNT, ///< 类型计数，占位用
    };

    /**
     * @brief DJI 电机配置
     */
    struct Config
    {
        bsp::can::Can& can;  ///< 所在 CAN 总线（借用，须比本对象长寿）
        Type           type; ///< 电机类型
        uint8_t        id1;  ///< 电机编号，范围 1~8，对应反馈 ID 0x201 ~ 0x208

        bool  auto_zero      = true;  ///< 上电收够一段稳定反馈后，是否自动把当前角度设为零点
        bool  reverse        = false; ///< 是否把输出方向整体反向
        float reduction_rate = 1.0f;  ///< 外接减速比，最终输出角度和速度会按它换算
    };

    explicit DJIMotor(const Config& cfg);
    /// 释放本对象在总线上的发送槽位；不注销接收回调（须在总线停止使用后调用）。
    ~DJIMotor() override;

    DJIMotor(const DJIMotor&)            = delete;
    DJIMotor& operator=(const DJIMotor&) = delete;
    DJIMotor(DJIMotor&&)                 = delete;
    DJIMotor& operator=(DJIMotor&&)      = delete;

    /**
     * @brief 获取输出轴角度
     * @return 角度，单位 deg
     */
    float getAngle() const override { return abs_angle_; }
    /**
     * @brief 获取输出轴速度
     * @return 速度；当前 DJI 实现单位为 rpm
     */
    float getVelocity() const override { return velocity_; }
    /**
     * @brief 把当前反馈角度重置为零点
     */
    void resetAngle() override;

    /**
     * @brief 解码 8 字节 DJI 反馈报文，并把单圈机械角度展开成连续角度
     * @param data CAN 数据段
     */
    void decode(const uint8_t data[8]);
    bool isConnected() const override { return watchdog_.isFed(); }

    [[nodiscard]] controllers::ControlMode defaultControlMode() const override
    {
        return controllers::ControlMode::ExternalPID;
    }

    bool supportsCurrent() const override { return true; }

    void setCurrent(const float current) override
    {
        iq_cmd_ = static_cast<int16_t>(cfg_.reverse ? -current : current);
    }
    /**
     * @brief 取出缓存中的电流命令
     *
     * 这个值会被 `SendIqCommand()` 统一打包发送。
     */
    int16_t getIqCMD() const { return iq_cmd_; }

    /**
     * @brief 初始化 DJI 反馈对应的 CAN 滤波器
     *
     * 必须在 `can.start()` 之前调用。`filter_index` 的含义随后端而定：bxCAN 为
     * filter bank 编号，FDCAN 为所选标准/扩展过滤器数组内的 element 下标；两者
     * 硬件布局不同，索引不可互换。
     *
     * @param can          目标 CAN 总线
     * @param filter_index 过滤器资源索引（bxCAN bank / FDCAN element）
     * @return 配置结果，成功为 `bsp::can::Status::Ok`
     */
    [[nodiscard]] static bsp::can::Status CAN_FilterInit(bsp::can::Can& can, uint32_t filter_index);

    /**
     * @brief 把某组 DJI 电机当前缓存的电流值打包发送到总线
     *
     * 发送槽位由各电机在构造时按 `id1` 自动登记，调用方无需提供电机列表。本函数
     * 只读取本组对应的 4 个槽位：缺席、或未受控（`currentController() == nullptr`）
     * 的槽位写零；即使在 `can` 上没有任何 DJI 电机，也会发送一帧全零电流。非法分组
     * 返回 `InvalidFrame`，不影响任何有效分组的发送行为。
     *
     * @param can       目标 CAN 总线（须已完成 `start()`）
     * @param cmd_group 分组
     * @return 帧的受理结果；本函数不重试也不等待真正发送完成
     */
    static bsp::can::SendResult SendIqCommand(bsp::can::Can& can, IqSetCMDGroup cmd_group);

private:
    /**
     * @brief 总线接收回调：过滤出属于本对象的 0x200 + id1 反馈帧并解码
     */
    void on_can(const bsp::can::FrameView& frame) noexcept;

    Config cfg_; ///< 构造时保存的配置

    float angle_zero_ = 0; ///< 零点角度，单位 deg

    float inv_reduction_rate_; ///< 总减速比的倒数，用于把内部量换算到输出轴

    /* Feedback */
    uint32_t          feedback_count_ = 0; ///< 接收到的反馈数据数量，用于上电自动清零
    service::Watchdog watchdog_;
    struct
    {
        float mech_angle{ 0 }; ///< 电机反馈侧单圈机械角度，单位 deg
        float rpm{ 0 };        ///< 电机侧转速，单位 rpm
        // float current; //< 电流大小
        // float temperature; //< 温度

        int32_t round_cnt{ 0 }; ///< 跨圈累计，用于把单圈机械角度展开成连续角度
    } feedback_{};

    /* Data */
    float abs_angle_ = 0; ///< 输出轴绝对角度，单位 deg
    float velocity_  = 0; ///< 输出轴角速度，单位 rpm

    /* Output */
    int16_t iq_cmd_ = 0; ///< 当前缓存的电流指令值
};

} // namespace motors
