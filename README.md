# MotorDrivers

`MotorDrivers` 是一套面向嵌入式电机控制项目的统一电机驱动库。

它的目标不是替代某一种底层总线驱动，而是把不同电机 / 驱动器的反馈、控制输入、控制权管理和控制器接入方式整理成一套可复用接口，让上层业务代码尽量只面向“能力”编程。

设计上，这个库并不只为 CAN 电机而写；更核心的目标是把各种电机合并到同一套抽象下，尽量达成“更换电机或驱动实现时，上层代码不用跟着大改”。当前仓库里已经落地的具体实现以 CAN 电机为主，因此文档中会看到较多 CAN 相关内容。

它主要解决四类问题：

- 不同电机 / 驱动接入方式的反馈解析差异
- 不同驱动支持能力不一致时，如何用统一接口表达
- 外部控制器与驱动器内部控制模式如何切换
- 多个控制器同时访问同一电机对象时，如何仲裁控制权

## 当前状态

### 已支持

#### 电机

- `DJI`
    - [x] `M3508_C620`
    - [x] `M2006_C610`
- `DM`
    - [x] `J4310_2EC`
    - [x] `J10010L_2EC`
    - [x] `S3519`
- `VESC`
    - [x] 通用 VESC CAN 设备支持
      当前已覆盖电流控制、内部速度控制与常见状态反馈解析

#### 控制器

- [x] 通用速度控制器 `MotorVelController`
- [x] 通用位置控制器 `MotorPosController`

### TODO

- `DM`
    - [ ] 更多 DM 电机型号
- `Others`
    - [ ] 其他类型电机 / 驱动接入

## 使用范围

这个仓库可以独立作为一个子仓库维护，但当前实现仍默认依赖 HAL、Watchdog、PIDMotor 等基础组件，
并不是“拉下来就能在任意平台直接编译”的完整 SDK。它更适合下面这些场景：

- 同一套机器人 / 执行器工程里混用多种电机，当前已接入实现主要是 CAN 电机
- 想让“底层驱动接入”和“控制算法”分层维护
- 想让业务代码尽量只和“角度、速度、力矩、控制模式”打交道

不适合直接拿来做的事情：

- 完全不依赖 HAL 和基础组件的纯独立发行库
- 上位机调参工具

## 目录结构

```text
MotorDrivers/
|-- core/           # 统一接口：IMotor、IController、ControlMode
|-- controllers/    # 通用控制器：位置环、速度环
|-- motors/         # 具体电机实现
`-- README.md
```

## 设计理念

### 1. 驱动层和控制层分离

驱动类只负责：

- 解析反馈报文
- 维护状态
- 按协议格式下发控制命令

控制器只负责：

- 根据参考值和反馈计算输出
- 选择应该使用哪一种电机输入能力

这样做的好处是：同一台电机既可以走“外部 PID + 电流输入”，也可以切换成“驱动器内部速度环 / 位置环”，上层业务代码不需要重写整套调用流程。

进一步说，驱动层和控制层分离的直接目标，就是尽量把“换电机”这件事限制在驱动接入层完成。只要新电机也能按照
`IMotor` 的能力模型暴露接口，上层控制器和业务模块就不应该因为底层换成了另一家电机而整体重写。

### 2. 用“能力”而不是“型号特判”来设计接口

统一接口并不只是“为了看起来整齐”，而是为了把“这台电机能做什么”与“它到底是谁家的、怎么接进来的”分开。

`motors::IMotor` 把能力分成三层：

- 反馈能力：`getAngle()`、`getVelocity()`、`resetAngle()`、`isConnected()`
- 控制输入能力：`setCurrent()`、`setInternalVelocity()`、`setInternalPosition()`、`setInternalMIT()`
- 控制权能力：`tryAcquireController()`、`releaseController()`、`currentController()`

其中“控制输入能力”又是显式可查询的：

- `supportsCurrent()`
- `supportsInternalVelocity()`
- `supportsInternalPosition()`
- `supportsInternalMIT()`

这意味着控制器不会通过“电机型号 if-else”来决定怎么控制，而是先问驱动“你支持哪一种输入”，再选择可行路径。

设计上这样拆分，是为了回答几个很实际的问题：

- 这台电机能不能直接收电流 / 力矩命令？
- 这台驱动器内部有没有速度环？
- 这台驱动器内部有没有位置环？
- 这台驱动器能不能一次收 MIT 五元组？

如果这些能力不拆开，上层控制器就只能按品牌、型号甚至接入方式写大量条件分支，复用性会很差，也就很难做到换电机时上层基本不改。

### 3. 为什么需要四种控制输入

这个库不是只提供一个 `setTarget()`，而是保留四种输入形式，因为它们对应的是四种不同的控制深度。

- `setCurrent()`：最低层输入，适合 MCU 自己做外部闭环。
  典型场景是 DJI 电机的外部速度环、DM 在 MIT 模式下退化成力矩环使用。
- `setInternalVelocity()`：把速度参考直接交给驱动器内部速度环。
  适合驱动器内部速度环已经调好，MCU 只需要给目标角速度的场景，例如 达妙电机的速度模式、VESC 的速度指令、以及未来自研电调的速度控制。
- `setInternalPosition()`：把位置参考直接交给驱动器的位置指令输入。
  适合执行器本身已经支持直接收位置目标时使用，例如 达妙电机位置模式、未来自研电调的位置控制。
- `setInternalMIT()`：一次发送位置、速度、刚度、阻尼、前馈。
  适合阻抗控制、轨迹跟踪或希望直接利用 MIT 协议能力的场景。目前仅达妙电机支持。
  这里的 `v_ref` 不是普通速度模式里的“目标转速”，而是和 `p_ref` 配套的速度项，很多时候就是
  `p_ref` 的导数，因此它的单位可能和普通速度接口不同。

`setCurrent()` 也必须做能力检查，不是多此一举。原因有两点：

- 不是所有驱动都支持“最低层输入”。有些驱动只开放内部速度 / 位置模式，外部根本拿不到电流或力矩入口。
- 即使都叫 `setCurrent()`，物理意义也不一定完全一样。
  DJI 更接近电流 / `Iq` 指令，VESC 是电流输入，DM 在 MIT 模式下则是把 MIT 退化成力矩输入使用。

所以 `supportsCurrent()` 的意义不是“接口存不存在”，而是“当前这台电机在当前配置下，是否允许你把它当作低层电流 / 力矩接口来用”。

### 4. 控制模式是“控制器如何使用能力”的约定

`controllers::ControlMode` 不是另一套重复接口，而是说明控制器应该怎样组合这些能力：

- `ExternalPID`：控制器自己算输出，再调用 `setCurrent()`
- `InternalVel`：控制器把目标速度交给 `setInternalVelocity()`
- `InternalPos`：电机只能接收位置指令，控制器把目标位置交给 `setInternalPosition()`
- `InternalVelPos`：电机同时支持速度指令和位置指令；速度控制器可以走速度输入，位置控制器可以直接走位置输入
- `InternalMIT`：控制器使用 `setInternalMIT()`
- `Default`：由电机驱动返回默认模式

这样做的目的，是让控制器和驱动之间的边界保持清晰：控制器决定“怎么闭环”，驱动决定“我能接受什么输入”。

### 5. 连接检测统一只通过 Watchdog

库内的连接检测有且只有一种方式：`Watchdog`。

约定是：

- 驱动在成功解析反馈后喂狗
- `isConnected()` 只读取 `Watchdog` 状态

不再额外引入第二套“连接标志”或手工状态位，避免多套来源互相打架。

## 单位约定

这个库希望尽量用统一物理量描述接口：

- 角度默认使用 `deg`
- 速度默认使用 `rpm`
- 力矩默认使用 `Nm`

如果变量名明确带有 `_rad`、`_rps` 之类后缀，则以变量名表达的单位为准。尤其是：

- DM 的量程配置 `pos_max_rad`、`vel_max_rad` 直接对应驱动器里的 `rad` / `rad/s`
- DM 对外的速度接口仍按库的统一约定使用 `rpm`，由 DM 驱动内部完成 `rad/s <-> rpm` 的换算与适配
- DM 的 `setCurrent()` 在 `MIT` 模式下，语义是“把 MIT 退化为力矩输入”，不是传统意义上的电流环
- DM 的 `setInternalMIT()` 需要特别看待：
  其中 `p_ref` 与 `v_ref` 是一组配套的 MIT 状态参考，`v_ref` 通常可理解为 `p_ref` 的导数，而不是普通
  速度模式下的目标转速。因此当前实现里 `p_ref` 用 `deg`，`v_ref` 用 `deg/s`，再由驱动内部换算成协议
  需要的 `rad` / `rad/s`

## CAN 接入方式

当前已落地的三类驱动都是 CAN 设备，因此它们都提供两类接入点：

- `CAN_FilterInit(...)`：初始化对应协议的滤波器
- `CANBaseReceiveCallback(...)`：处理一帧已经取出的 CAN 报文

如果你的项目已经有统一 CAN 接收层，推荐把 `CANBaseReceiveCallback(...)` 注册进去；如果没有，也可以在
HAL 的 FIFO 回调里直接调用它们。

如果你使用的是 `BasicComponents` 仓库里的
[`bsp/can_driver`](https://github.com/HITSZ-WTRobot-Packages/BasicComponents/tree/docs/lots-of-documents/bsp/can_driver)，
它的接收路径就是“先从 FIFO 里取出一帧，再按注册顺序依次直接调用所有已注册回调”。这种情况下，可以把三家的
`CANBaseReceiveCallback(...)` 直接注册给它。

### 最小接收样例（直接调用）

下面这个样例只展示“如何在 HAL FIFO 回调里把一帧 CAN 报文直接交给三类驱动”，便于移植到不同项目里。

```cpp
#include "dji.hpp"
#include "dm.hpp"
#include "vesc.hpp"

void motor_can_filter_init()
{
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    motors::DMMotor::CAN_FilterInit(&hcan1, 1, 0x12);
    motors::VESCMotor::CAN_FilterInit(&hcan1, 2);
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        CAN_RxHeaderTypeDef header{};
        uint8_t             data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            Error_Handler();
            return;
        }

        motors::DJIMotor::CANBaseReceiveCallback(hcan, &header, data);
        motors::DMMotor::CANBaseReceiveCallback(hcan, &header, data);
        motors::VESCMotor::CANBaseReceiveCallback(hcan, &header, data);
    }
}
```

如果你的项目已经封装好了统一 CAN 接收层，也可以不写上面的 FIFO 轮询，而是直接把三家的
`CANBaseReceiveCallback(...)` 注册进去。以 `BasicComponents` 的 `bsp/can_driver` 为例，可以写成：

```cpp
#include "can_driver.hpp"
#include "dji.hpp"
#include "dm.hpp"
#include "vesc.hpp"

void motor_can_init()
{
    motors::DJIMotor::CAN_FilterInit(&hcan1, 0);
    motors::DMMotor::CAN_FilterInit(&hcan1, 1, 0x12);
    motors::VESCMotor::CAN_FilterInit(&hcan1, 2);

    CAN_InitMainCallback(&hcan1);
    CAN_RegisterCallback(&hcan1, motors::DJIMotor::CANBaseReceiveCallback);
    CAN_RegisterCallback(&hcan1, motors::DMMotor::CANBaseReceiveCallback);
    CAN_RegisterCallback(&hcan1, motors::VESCMotor::CANBaseReceiveCallback);
    CAN_Start(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
```

## 调用样例

### 关于构造顺序

实际项目里，`motor` 和 `controller` 往往不会写在同一个文件里；如果直接把它们都做成跨文件的全局对象，
就会遇到 C++ 静态 / 动态初始化顺序不确定的问题。

因此更推荐的做法是：

- 全局只保留指针或句柄
- 在显式的初始化函数里按顺序 `new`
- 先构造电机对象，再构造控制器对象，最后再 `enable()`

很多嵌入式项目还会全局重载 `operator new`，把这里的 `new` 定向到静态内存池、RTOS 堆或预留内存区，
这样既能控制初始化顺序，也不会依赖默认运行时堆。这里只先说明推荐用法，不展开具体重载实现。

下面的样例都只使用本仓库里的真实接口名，但为了便于阅读，参数值只作为示意。你需要按自己的电机
型号、减速比、限幅和控制频率去填。

### 样例 1：DJI 轮电机走外部速度环

`DJI` 这一路的典型用法是：`MotorVelController` 在 MCU 里做速度环，驱动对象只负责反馈解析和缓存
电流目标；真正发 CAN 帧时，要额外调用一次 `SendIqCommand(...)` 聚合发送。

```cpp
#include "dji.hpp"
#include "motor_vel_controller.hpp"

namespace
{
motors::DJIMotor*                    wheel_lf     = nullptr;
controllers::MotorVelController*    wheel_lf_ctrl = nullptr;
} // namespace

void wheel_init()
{
    wheel_lf = new motors::DJIMotor({
            .hcan           = &hcan1,
            .type           = motors::DJIMotor::Type::M3508_C620,
            .id1            = 1,
            .auto_zero      = true,
            .reverse        = false,
            .reduction_rate = 1.0f,
    });

    wheel_lf_ctrl = new controllers::MotorVelController(
            wheel_lf,
            {
                    .pid = {
                            .Kp             = 500.0f,
                            .Ki             = 5.0f,
                            .Kd             = 0.0f,
                            .abs_output_max = 8000.0f,
                    },
                    .ctrl_mode = controllers::ControlMode::ExternalPID,
            });

    wheel_lf_ctrl->enable();
    wheel_lf_ctrl->setRef(300.0f); // 目标速度 300 rpm
}

void wheel_task_1khz()
{
    wheel_lf_ctrl->update();

    motors::DJIMotor::SendIqCommand(
            &hcan1, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
}
```

如果同一条 CAN 上同时挂了 `id1 = 5 ~ 8` 的 DJI 电机，还需要额外对 `IqCMDGroup_5_8` 再调用一次
`SendIqCommand(...)`。

### 样例 2：DM 位置模式配合位置控制器

`DM` 的典型用法取决于驱动器当前配置成哪种模式。下面这个例子假设驱动器本身已经配置成位置模式，
因此 `MotorPosController` 直接把角度目标交给 `setInternalPosition()`。

```cpp
#include "dm.hpp"
#include "motor_pos_controller.hpp"

namespace
{
motors::DMMotor*                   lift_motor = nullptr;
controllers::MotorPosController*   lift_ctrl  = nullptr;
} // namespace

void lift_init()
{
    lift_motor = new motors::DMMotor({
            .hcan           = &hcan1,
            .id0            = 0x09,
            .type           = motors::DMMotor::Type::J4310_2EC,
            .mode           = motors::DMMotor::Mode::Pos,
            .pos_max_rad    = 12.5f,
            .vel_max_rad    = 30.0f,
            .tor_max        = 10.0f,
            .auto_zero      = true,
            .reverse        = false,
            .reduction_rate = 1.0f,
    });

    lift_ctrl = new controllers::MotorPosController(
            lift_motor,
            {
                    .position_pid = {
                            .Kp             = 0.0f,
                            .Ki             = 0.0f,
                            .Kd             = 0.0f,
                            .abs_output_max = 0.0f,
                    },
                    .velocity_pid = {
                            .Kp             = 0.0f,
                            .Ki             = 0.0f,
                            .Kd             = 0.0f,
                            .abs_output_max = 0.0f,
                    },
                    .ctrl_mode          = controllers::ControlMode::InternalPos,
                    .internal_set_ratio = 1,
            });

    lift_ctrl->enable();   // 获取控制权时会顺带发送 DM 使能包
    lift_ctrl->setRef(90); // 目标角度 90 deg
}

void lift_task_1khz()
{
    lift_ctrl->update();
}

void lift_idle_task_1khz()
{
    // 仅在“当前未使能但仍想维持一收一回通信节奏”时需要。
    if (lift_motor)
        lift_motor->ping();
}
```

这个例子里 `pos_max_rad`、`vel_max_rad`、`tor_max` 是驱动器协议量程，必须和上位机配置保持一致；
而控制器和业务层看到的角度 / 速度 / 力矩，仍按本库的统一约定解释。

### 样例 3：VESC 速度模式和直接调试

`VESC` 同时支持电流输入和内部速度输入。常规业务里可以配合 `MotorVelController` 使用内部速度模式；
调试阶段也可以直接对电机对象下发命令。

```cpp
#include "motor_vel_controller.hpp"
#include "vesc.hpp"

namespace
{
motors::VESCMotor*                 conveyor_motor = nullptr;
controllers::MotorVelController*   conveyor_ctrl  = nullptr;
} // namespace

void conveyor_init()
{
    conveyor_motor = new motors::VESCMotor({
            .hcan           = &hcan1,
            .id             = 1,
            .electrodes     = 7,    // 14 极电机应填写 7，而不是 14
            .reduction_rate = 1.0f,
            .auto_zero      = true,
            .reverse        = false,
    });

    conveyor_ctrl = new controllers::MotorVelController(
            conveyor_motor,
            {
                    .pid = {
                            .Kp             = 0.0f,
                            .Ki             = 0.0f,
                            .Kd             = 0.0f,
                            .abs_output_max = 0.0f,
                    },
                    .ctrl_mode          = controllers::ControlMode::InternalVel,
                    .internal_set_ratio = 1,
            });

    conveyor_ctrl->enable();
    conveyor_ctrl->setRef(1200.0f); // 目标输出轴速度 1200 rpm
}

void conveyor_task_1khz()
{
    conveyor_ctrl->update();
}

void conveyor_debug()
{
    if (!conveyor_motor)
        return;

    conveyor_motor->setCurrent(5.0f);         // 直接给电流
    conveyor_motor->setInternalVelocity(800); // 直接给速度
}
```

## 各驱动专项说明

主 README 只保留统一抽象、接入方式和最小样例。各电机家族自己的协议特点、约束和特殊说明，分别放在
各自目录下：

- [DJI 驱动说明](motors/DJI/README.md)
  重点说明聚合发送、反馈展开和圈数统计
- [DM 驱动说明](motors/DM/README.md)
  重点说明 `master_id`、`id0` 规划、`ping()` 语义和模式约束
- [VESC 驱动说明](motors/VESC/README.md)
  重点说明 `ERPM`、`electrodes` 的真实含义和状态反馈字段解释

## 建议阅读顺序

如果第一次接触这个库，建议按下面顺序看：

1. `core/motor_if.hpp`
2. `controllers/`
3. 本 README 里的“设计理念”“单位约定”“调用样例”
4. `motors/DJI/README.md`
5. `motors/DM/README.md`
6. `motors/VESC/README.md`

这样更容易先理解能力模型，再理解各类电机的特殊约束。
