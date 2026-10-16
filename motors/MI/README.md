# MI 驱动说明

本页说明 `motors/MI/` 中小米 CyberGear 驱动在 `MotorDrivers` 框架下的通用接入方式、能力边界和使用约定。统一电机抽象、控制器接口和全库单位约定，请先看 [../../README.md](../../README.md)。

本文只面向驱动框架本身，不依赖任何具体 `UserCode` 示例工程。

## 当前覆盖

- 小米 CyberGear 微电机
- CAN 2.0B 扩展帧
- 反馈解析、使能 / 失能、MIT 控制、机械零位、参数读写
- 每个电机占用所属 `Can` 的一个回调槽；容量由 `CAN_DRIVER_MAX_CALLBACKS` 决定，默认每条总线 `14` 个，与其他电机及应用回调共享

已实现的协议帧包括：

- Type 1：MIT 运控指令
- Type 2：电机反馈
- Type 3：使能
- Type 4：失能 / 清故障
- Type 6：设置机械零位（掉电丢失）
- Type 17 / 18：参数读写

## 能力模型

`MIMotor` 当前在统一接口中暴露的能力为：

```cpp
defaultControlMode() == controllers::ControlMode::InternalMIT
supportsCurrent() == true
supportsInternalMIT() == true
internalVelocitySupportLevel() == InternalVelocitySupportLevel::NotSupported
internalPositionSupportLevel() == InternalPositionSupportLevel::NotSupported
```

因此推荐把 MI 电机按 MIT 电机使用：

- 轨迹、阻抗或位置保持场景，直接调用 `setInternalMIT()`。
- 只需要底层力矩输入时，调用 `setCurrent()`。

这里的 `setCurrent()` 不是传统电流环接口，而是把 MIT 指令退化为纯力矩输入：`Kp = 0`、`Kd = 0`，只下发前馈力矩。因此参数按 `Nm` 理解。

## CAN 接入

MI 驱动使用 `bsp::CANDriver2` 的经典 CAN 扩展数据帧。电机构造时自动通过
`Can::register_callback` 注册成员回调，按通信类型和自身电机 ID 处理 Type 2 / Type 17 报文；
不再使用电机映射表，也不需要手工注册静态接收入口或编写 HAL FIFO 包装回调。

以下示例使用 FDCAN1，须在板级 HAL 初始化之后调用一次；bxCAN 项目换成对应 HAL CAN 句柄。

```cpp
#include "fdcan.h"
#include "mi.hpp"

bsp::can::Can motor_can{hfdcan1};
motors::MIMotor* motor = nullptr;

void motor_can_init()
{
    static motors::MIMotor storage({.can = motor_can, .id = 3, .master_id = 0});
    motor = &storage;

    if (motors::MIMotor::CAN_FilterInit(motor_can, 0) != bsp::can::Status::Ok)
        Error_Handler();
    if (motor_can.start() != bsp::can::Status::Ok)
        Error_Handler();
}
```

电机必须在 `Can::start()` 前构造，且与 `Can`、HAL 句柄一起存活到总线不再使用。
电机不可复制或移动；启动后构造电机或回调容量不足会触发 `Error_Handler()`，没有析构解绑接口。
滤波器与全局接收策略的资源要求见 [全库 CAN 接入说明](../../README.md#can-接入方式)。

## 对象配置

最小构造配置如下：

```cpp
#include "mi.hpp"

motors::MIMotor* motor = nullptr;

void motor_init()
{
    motors::MIMotor::Config cfg{.can = motor_can};
    cfg.id             = 3;
    cfg.master_id      = 0;
    cfg.auto_zero      = true;
    cfg.reverse        = false;
    cfg.reduction_rate = 1.0f;

    motor = new motors::MIMotor(cfg);
}
```

配置字段说明：

| 字段 | 含义 |
| --- | --- |
| `can` | 电机借用的 `bsp::can::Can&`；必须显式初始化，不能默认构造 `Config{}` |
| `id` | 电机 CAN ID，当前实现按 `0 ~ 127` 使用 |
| `master_id` | 主机 ID，常用默认值为 `0` |
| `auto_zero` | 收到稳定反馈后是否自动把当前位置作为软件零点 |
| `reverse` | 是否整体反转反馈方向和控制方向 |
| `reduction_rate` | 外部减速比，用于把电机侧反馈换算到输出轴 |

构造对象时，驱动向 `cfg.can` 注册对象成员回调；总线广播反馈后，每个对象只匹配自身电机 ID。以上 `motor_init()` 须在总线启动前调用，`motor_can` 由板级定义。

## 单位约定

MI 驱动对外沿用 `MotorDrivers` 的统一物理量约定：

| 接口 | 单位 / 语义 |
| --- | --- |
| `getAngle()` | 输出轴连续角度，`deg` |
| `getVelocity()` | 输出轴速度，`rpm` |
| `torque()` | 反馈力矩，`Nm` |
| `temperature()` | 温度，`degC` |
| `setCurrent(current)` | 力矩输入，`Nm` |
| `setInternalMIT(t_ff, p_ref, v_ref, kp, kd)` | MIT 五元组 |

`setInternalMIT()` 的参数约定：

| 参数 | 单位 / 范围 |
| --- | --- |
| `t_ff` | 前馈力矩，`Nm`，协议量程 `-12 ~ +12` |
| `p_ref` | 位置参考，`deg` |
| `v_ref` | MIT 速度参考，`deg/s` |
| `kp` | 刚度，`0 ~ 500` |
| `kd` | 阻尼，`0 ~ 5` |

注意：`v_ref` 不是普通速度模式的目标转速，不使用 `rpm`。它是 MIT 状态参考中的速度项，通常和 `p_ref` 配套使用，可理解为位置参考的导数。

驱动内部会在发送前限幅，并转换为协议需要的 `uint16` 表示。

## 反馈与状态

收到 Type 2 反馈后，驱动会更新：

- 角度
- 速度
- 力矩
- 温度
- 运行状态
- 故障位
- 连接看门狗

常用读取接口：

```cpp
float angle = motor->getAngle();
float speed = motor->getVelocity();
float torque = motor->torque();
float temp = motor->temperature();
auto state = motor->runState();
uint16_t fault = motor->fault();
bool online = motor->isConnected();
```

`runState()` 返回：

```cpp
enum class RunState : uint8_t
{
    Reset = 0,
    Cali  = 1,
    Run   = 2,
};
```

`fault()` 返回故障位组合，可检查：

```cpp
FaultUnderVoltage
FaultOverCurrent
FaultOverTemp
FaultMagEncoder
FaultHallEncoder
FaultNotCalibrated
```

## 连续角度与零点

CyberGear 反馈的位置在协议量程内循环。驱动会根据相邻反馈帧的角度跳变维护跨圈计数，并对外提供连续角度。

零点相关接口有两个层级：

```cpp
motor->resetAngle();         // 只重置软件零点
motor->setMechanicalZero();  // 写入电机机械零位，掉电丢失
```

一般控制流程中优先使用 `resetAngle()`。`setMechanicalZero()` 会改变电机内部零位，调用前应确认机构位置和安全状态。

当 `Config::auto_zero = true` 时，驱动会在上电后收到一定数量反馈后自动执行一次软件归零。

## 使能与失能

基本控制前后流程：

```cpp
motor->disable(true);  // 停止并清故障
motor->enable();       // 使能

// 周期性控制
motor->setInternalMIT(0.0f, target_deg, target_dps, kp, kd);

motor->disable(false); // 停止控制
```

`disable(clear_fault)` 的参数含义：

- `true`：失能并请求清故障
- `false`：仅失能

`ping()` 用于未使能状态下的通信保活。它发送失能包，不会使能电机：

```cpp
if (motor)
    motor->ping();
```

连接检测统一通过 `Watchdog` 完成：成功解析反馈后喂狗，`isConnected()` 只读取看门狗状态。

## 参数读写

驱动提供 Type 17 / 18 参数读写接口：

```cpp
bool ok = motor->writeParam(index, value);

float value = 0.0f;
ok = motor->readParam(index, value);
```

`run_mode` 的索引是 `0x7005`，可按下面方式切换到 MIT 模式并读回校验：

```cpp
motor->writeParam(0x7005, static_cast<float>(motors::MIMotor::Mode::MIT));

float run_mode = -1.0f;
if (motor->readParam(0x7005, run_mode))
{
    // static_cast<int>(run_mode) == static_cast<int>(motors::MIMotor::Mode::MIT)
}
```

模式枚举：

```cpp
enum class Mode : uint8_t
{
    MIT = 0,
    Pos = 1,
    Vel = 2,
    Cur = 3,
};
```

当前驱动的统一控制接口只完整接入了 `MIT` 能力。虽然可以通过参数写入切换其他模式，但普通位置、速度和电流模式尚未通过 `setInternalPosition()`、`setInternalVelocity()` 等统一接口封装。

`readParam()` 当前等待回包超时时间为 `10 ms`，应避免在高频中断控制回调中调用。

## MIT 控制示例

下面是一个框架级最小示例，展示如何在周期任务中使用 MIT 控制。控制频率、目标生成和安全限幅应由具体项目决定。

```cpp
#include "mi.hpp"

motors::MIMotor* motor = nullptr;

void motor_init()
{
    motors::MIMotor::Config cfg{.can = motor_can};
    cfg.id             = 3;
    cfg.master_id      = 0;
    cfg.auto_zero      = true;
    cfg.reverse        = false;
    cfg.reduction_rate = 1.0f;

    motor = new motors::MIMotor(cfg);

}

void motor_start() // motor_init() 后且 motor_can.start() 成功后调用
{
    motor->disable(true);
    motor->writeParam(0x7005, static_cast<float>(motors::MIMotor::Mode::MIT));
    motor->enable();
}

void motor_control_update()
{
    if (!motor || !motor->isConnected())
        return;

    const float target_deg = 0.0f;
    const float target_dps = 0.0f;
    const float torque_ff = 0.0f;
    const float kp = 80.0f;
    const float kd = 1.0f;

    motor->setInternalMIT(torque_ff, target_deg, target_dps, kp, kd);
}

void motor_stop()
{
    if (!motor)
        return;

    motor->setInternalMIT(0.0f, motor->getAngle(), 0.0f, 20.0f, 0.5f);
    motor->disable(false);
}
```

首次调试建议使用较小的 `kp`、`kd`、目标角度和力矩前馈，确认方向、零点、限位和急停可靠后再提高参数。

## 与控制器配合

`MIMotor` 的默认控制模式是 `InternalMIT`。如果控制器支持 `InternalMIT`，可以让控制器调用 `setInternalMIT()`；如果只需要低层力矩输入，也可以让外部控制器走 `setCurrent()`。

控制器获取控制权时，`tryAcquireController()` 会在电机未使能时自动发送 `enable()`：

```cpp
if (motor->tryAcquireController(controller))
{
    // controller 可以开始更新控制输出
}
```

释放控制权时，`releaseController()` 不会自动失能电机，目的是避免控制器交接时打断电机状态。是否失能应由上层状态机决定。

## 常见问题

### `isConnected()` 一直为 false

优先检查：

- CAN 波特率是否为目标电机配置值
- 是否启用了扩展帧接收
- 是否在 `Can::start()` 前构造了电机，且总线已成功启动
- `Config::id` 是否等于电机实际 ID
- 电机是否正在发送 Type 2 反馈

### 能使能但不运动

优先检查：

- 是否已经切到 `Mode::MIT`
- 是否周期性调用了 `setInternalMIT()`
- `kp`、`kd`、`t_ff` 是否过小
- `reverse` 和机构方向是否符合预期
- `fault()` 是否存在故障位

### 角度方向反了

设置：

```cpp
cfg.reverse = true;
```

不要在上层同时再反一次目标，否则反馈方向和控制方向容易不一致。

### 角度比例不对

检查 `reduction_rate`。它应该填写电机侧到输出轴的外部减速比。驱动会用它把电机侧角度和速度换算为输出轴量。

### 参数读写偶发失败

`readParam()` 有 `10 ms` 超时，并依赖 Type 17 回包能进入同一个接收分发路径。不要在中断里调用，也不要在高频控制循环中连续阻塞读取参数。