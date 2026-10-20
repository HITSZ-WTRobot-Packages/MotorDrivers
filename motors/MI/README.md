# MI 驱动说明

本目录是 `MotorDrivers::MIMotor` 包，身份与依赖见 [cpkg.toml](cpkg.toml)，依赖 `MotorDrivers::Core`、`bsp::CANDriver2`、`services::Watchdog`。全仓资源与启动契约见 [根 README](../../README.md#can-接入方式)，统一能力与控制权见 [Core](../../core/README.md)。具体符号、配置字段和枚举以 [mi.hpp 的 Doxygen](mi.hpp) 为准。

## 协议与能力

当前覆盖小米 CyberGear，使用 Classic CAN 2.0B 扩展数据帧：

| 通信类型 | 当前用途 |
| --- | --- |
| Type 1 | MIT 运控 |
| Type 2 | 电机反馈 |
| Type 3 | 使能 |
| Type 4 | 失能 / 清故障 |
| Type 6 | 设置机械零位，掉电丢失 |
| Type 17 / 18 | 参数读写 |

统一能力中默认模式是 `InternalMIT`，支持 MIT 与低层力矩输入，不支持内部位置或内部速度接口。`setCurrent()` 是 `Kp = Kd = 0` 的纯前馈力矩输入，参数为 `Nm`，不是安培。

两个通用控制器无论显式选择 `InternalMIT`，还是由 `Default` 解析得到该模式，都不会调用 `setInternalMIT()`：速度控制器走外部速度 PID，位置控制器走外部位置 + 速度串级 PID，最后通过低层力矩输入下发。需要刚度、阻尼和前馈的轨迹 / 阻抗控制，应直接使用五元组，并由宿主协调与控制器之间的写入。

## CAN 接入与对象寿命

宿主通过 CubeMX 配置 HAL1 时序、IRQ、扩展帧接收资源及过滤器容量，完成 HAL 初始化后构造 `Can`；电机借用尚未启动的 `Can&`，不绑定固定实例或隐式全局总线。

每个电机构造时注册一个成员回调，按通信类型与自身 ID 处理 Type 2 / Type 17 报文，不需手工 HAL FIFO 包装回调或电机映射表。回调槽与应用及其他协议共享，默认每条总线 `CAN_DRIVER_MAX_CALLBACKS = 14`。启动后构造或容量不足会触发 `Error_Handler()`。

现有 `CAN_FilterInit(can, index)` 实际配置放行全部扩展 ID、送 FIFO0 的硬件过滤器，不是只校验；宿主须在启动前协调索引并检查返回状态。可与 VESC 共享扩展帧过滤器，协议和 ID 冲突仍由宿主排除。bxCAN bank 与 FDCAN 扩展 element 不能混用；这不是绕开 CubeMX 手动重新初始化外设的方案。配置校验不足或要改变此边界时须先 Ask，详见根文档。

电机与 `Can`、HAL 句柄均须存活到总线不再使用；电机不可复制或移动，没有析构解绑接口。对象构造和过滤器准备完成、`Can::start()` 成功后，才发送使能、参数或控制命令。

## 配置与物理量

电机 `id` 按当前实现的 `0 ~ 127` 范围使用，须等于设备实际 ID；主机 ID 默认常用 `0`。`reduction_rate` 是外部减速比，用于将反馈角度和速度换算到输出轴。`reverse` 同时处理反馈与控制方向，宿主不要重复反转。

| 物理量 / 输入 | 单位与限制 |
| --- | --- |
| 输出轴连续角度 | `deg` |
| 输出轴速度 | `rpm` |
| 反馈力矩 / 低层力矩输入 | `Nm` |
| 温度 | `degC` |
| MIT 前馈力矩 | `Nm`，协议量程 `-12 ~ +12` |
| MIT 位置参考 | `deg` |
| MIT 速度参考 | `deg/s`，不是普通速度接口的 `rpm` |
| MIT 刚度 / 阻尼 | 刚度 `0 ~ 500`，阻尼 `0 ~ 5` |

MIT 速度项与位置参考配套，常可理解为位置参考的导数。驱动按协议范围限幅并编码，不代替宿主机构限位、安全限幅或急停。

## 反馈与零点

Type 2 更新角度、速度、力矩、温度、运行状态、故障位及连接看门狗。运行状态包括复位、校准、运行；故障包括欠压、过流、过温、磁编码器、霍尔编码器和未校准。具体枚举与字段见源码 Doxygen，连接在线不代表无故障。

反馈位置在协议范围内循环，驱动利用相邻帧跳变维护连续角度；丢失过多反馈时不能恢复未观测的跨圈。`auto_zero` 在收到一定数量反馈后建立一次软件零点。

`resetAngle()` 仅重定义软件零点；`setMechanicalZero()` 会改变电机内部机械零位且掉电丢失。一般控制优先使用软件归零；写机械零位前须确认机构位置和安全状态。

## Core 生命周期、保护与保活

`MIMotor` 继承 `core::control::IController<0>`，父子关系由 Core 管理，不再提供旧控制权申请 / 释放接口。无控制者根 `enable()` 成功后进入 `Protected`；直接控制需再调用 `standalone()`，父控制器接管则由 Core 建立 `Controlled`。公开 `setCurrent()` / `setInternalMIT()` 继承状态门控包装，仅在 `Standalone` / `Controlled` 受理，不区分直接调用与父控制器调用。

`selfEnable()` 发送 Type 3，按底层发送受理结果返回 `bool`；使能请求标记不等于 Core `state()`，也不等于反馈 `runState()`。`selfProtect()` 直接发送 MIT 全零五元组（力矩与 `Kp = Kd = 0`），`selfDisable()` 直接发送 Type 4 并清除请求标记；钩子不经过公共运动门控，不启动总线、不注册回调、不自行操作父子生命周期。保护 / 失能帧被拒收时调用既有 `Error_Handler()`，不重试、不等待硬件确认。

旧 `disable(clear_fault)` 已移除：现在 `disable(bool reverse = false)` 是 Core 生命周期接口，参数不再表示清故障。清故障请在 Core `Disabled` 时调用明确的 `clearFault()`（Type 4，data[0] = 1）；启用状态调用直接返回 `false`，不会更改 Core 状态。

`ping()` 在 Core `Disabled` 或本地未提交使能请求时直接发送 Type 4 保活，不调用 Core `disable()` / `enable()`，不会使能电机。成功解析反馈后喂 Watchdog，`isConnected()` 只读其状态。

返回 `bool` 的协议发送接口确认的是发送受理，不是电机执行完成；受理包括排队及替换旧排队帧。五元组和低层输入接口保留原有不返回发送失败的业务语义，不能用“调用完成”作为线上成功确认。

宿主必须串行化生命周期、业务调用和周期发包；Core 快照不保证业务并发安全，也不鉴别调用者来源。析构前必须显式 `disable()` 并处理返回值，不能依赖析构释放；回调寿命约束仍然有效。

## 参数读写与模式

Type 17 / 18 提供参数读写。`run_mode` 索引为 `0x7005`，可写入 `Mode::MIT` 并通过参数读取核对；模式还包括 `Pos`、`Vel`、`Cur`，但统一控制接口目前仅完整接入 MIT。通过参数切换其他模式不会让内部速度、位置接口自动可用。

参数写入和机械零位均不构成掉电保存承诺。`writeParam()` 成功表示发送已受理，不是读回一致；宿主应核对需要确认的状态。目标参数 `iq_ref`（`0x7006`）、`spd_ref`（`0x700A`）、`loc_ref`（`0x7016`）同样仅在 `Standalone` / `Controlled` 可写，其他状态返回 `false`，不能用参数写入绕过运动门控；维护参数与读取不新增状态限制。

`readParam()` 同步等待最多 `10 ms`，依赖 HAL tick 推进和 Type 17 回包通过同一接收分发路径到达。不要在 ISR 中调用，也不要在高频控制循环内连续阻塞读取。参数访问应由宿主串行协调。

## 借用总线的最小示例

以下展示构造、独立控制会话与周期 MIT 调用，不配置外设、过滤器或启动总线。宿主须在启动前构造对象，在启动及安全条件满足后选择 MIT 模式并开启会话；示意 ID 不是板卡或总线实例配置。

```cpp
#include "mi.hpp"

motors::MIMotor& attach_motor(bsp::can::Can& can)
{
    // 只调用一次；对象与所借用的 can 都必须覆盖总线使用期。
    static motors::MIMotor motor({.can = can, .id = 3, .master_id = 0});
    return motor;
}

bool begin_control(motors::MIMotor& motor)
{
    // enable 成功仅建立 Protected；显式 standalone 后才接受公开目标。
    if (!motor.enable())
        return false;
    return motor.standalone();
}

void control_step(motors::MIMotor& motor, float target_deg, float target_dps,
                  float torque_ff, float kp, float kd)
{
    // 宿主须先成功 begin_control，且串行化此处与生命周期操作。
    if (motor.isConnected())
        motor.setInternalMIT(torque_ff, target_deg, target_dps, kp, kd);
}

bool end_control(motors::MIMotor& motor)
{
    // 析构前必须完成；失败时宿主须按生命周期事务结果处理。
    return motor.disable();
}
```

未使能时如需先建立一收一回通信，可按设备协议执行 `ping()`；首次调试从较小的刚度、阻尼、角度和前馈开始，先确认方向、零点、限位与急停。

## 排查入口

- 不在线：检查 CubeMX 时序与设备波特率、扩展帧资源、启动结果、设备 ID，以及 Type 2 反馈是否到达。
- 能使能但不运动：确认设备 MIT 模式、周期五元组、故障状态和参考 / 增益；不要只依据本地使能标记。
- 方向反转或比例不对：核对 `reverse`、外部减速比，避免上层重复转换。
- 参数偶发失败：检查接收路径与调度，避免 ISR 或高频循环阻塞访问；`10 ms` 超时不是自动重试保证。