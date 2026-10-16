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
    - [x] `S2325_1EC`
    - [x] `J4340_2EC`
    - [x] `J8009P_2EC`
- `MI`
    - [x] 小米 CyberGear 微电机
      当前已覆盖 MIT 运控、使能 / 失能、机械零位与参数读写
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
- `internalVelocitySupportLevel()`：`NotSupported` / `RefOnly` / `RefAndMaxi`
- `internalPositionSupportLevel()`：`NotSupported` / `RefOnly` / `RefAndMaxv`
- `supportsInternalMIT()`

后两个返回等级而不是布尔值，这样除了“能不能用内部速度 / 位置指令”，还能表达“支不支持随指令携带最大电流 / 最大速度限制”。

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
  适合阻抗控制、轨迹跟踪或希望直接利用 MIT 协议能力的场景。目前达妙电机在 `MIT` 模式下、以及小米
  CyberGear 支持；达妙在其它模式下 `supportsInternalMIT()` 返回 `false`。
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
- `InternalMIT`：电机以 MIT 格式接收控制量。注意两个通用控制器都不会直接调用 `setInternalMIT()`：
  无论显式选择 `InternalMIT`，还是 `Default` 解析为 `InternalMIT`，它们都会退化为 MCU 侧外部 PID，
  最终通过 `setCurrent()` 下发。速度控制器 `MotorVelController` 只用速度 PID；位置控制器
  `MotorPosController` 用位置 PID 输出目标角速度、再由速度 PID 输出电流，即位置 + 速度串级 PID，
  并非两者都做串级。要真正使用 MIT 五元组，应在业务周期任务里直接调用 `setInternalMIT()`
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

DJI、DM、MI、VESC 均依赖 `BasicComponents` 的
[`bsp/can_driver2`](https://github.com/HITSZ-WTRobot-Packages/BasicComponents/tree/main/bsp/can_driver2)。
`Config::can` 借用一个 `bsp::can::Can&`，不再接收 HAL 句柄。
每个电机在构造时通过 `Can::register_callback` 注册自己的成员回调，在 ISR 中匹配自身协议与 ID 后解码；
不再维护电机反馈映射表，也不再提供 `CANBaseReceiveCallback` 或手工 HAL FIFO 分发入口。

初始化顺序：

1. 完成板级 HAL 初始化，并为 CAN/FDCAN 配置好时序、IRQ 和接收资源。
2. 构造 `Can` 和所有电机，配置过滤器；这两项均必须在 `Can::start()` 前完成。
3. 检查 `Can::start()` 的返回值，再使能控制器或发送电机命令。

电机、`Can` 和 HAL 句柄都必须存活到总线不再使用；启动后不能新增电机，也不能销毁、复制或移动已注册电机。
注册失败（启动后注册或回调容量不足）会调用 `Error_Handler()`。所有协议与应用回调共享每条总线的
`CAN_DRIVER_MAX_CALLBACKS` 容量，默认 `14`；同一总线内的协议电机 ID 唯一性由调用方保证。
DJI 在构造时检查同一总线上的重复 `id1`，并自动登记发送槽位；该登记仅用于组包，不参与反馈分发。
不能再把旧驱动或另一个 `Can` 绑定到同一外设。

### 最小接入样例

以下使用 FDCAN1；bxCAN 项目改用自己的 HAL CAN 句柄。所有构造入口都须在 HAL 初始化完成后调用。

```cpp
#include "fdcan.h"
#include "dji.hpp"

// 板级在 HAL 初始化完成后首次调用。这里只构造 Can，不创建电机、不配置过滤器、也不启动总线。
// Can 放进函数内静态存储，首次调用时构造并一直存活到程序结束，返回的引用在整个运行期有效。
bsp::can::Can& motor_can_init()
{
    static bsp::can::Can motor_can{hfdcan1};
    return motor_can;
}

// 独立的最小 DJI 示例：在 HAL 初始化后调用一次，不与下文控制样例的初始化流程混用。
void motor_demo_init()
{
    bsp::can::Can& motor_can = motor_can_init();

    // 总线已构造但尚未启动；静态电机在构造时注册回调，并保持到程序结束。

    static motors::DJIMotor motor({
            .can = motor_can,
            .type = motors::DJIMotor::Type::M3508_C620,
            .id1 = 1,
    });

    if (motors::DJIMotor::CAN_FilterInit(motor_can, 0) != bsp::can::Status::Ok)
        Error_Handler();
    if (motor_can.start() != bsp::can::Status::Ok)
        Error_Handler();
}
```

仅运行最小 DJI 示例时，板级在 HAL 初始化后调用一次 `motor_demo_init()`；它会构造示例电机、配置过滤器并启动总线。
使用下文控制样例时，不调用 `motor_demo_init()`，而是先保存 `bsp::can::Can& motor_can = motor_can_init();`。
此时总线尚未启动，也没有隐式创建 DJI 电机；将引用传给所有需要的 `*_init(can)`，完成电机和过滤器初始化后，
再检查 `motor_can.start()` 的返回值，最后使能控制器并运行周期任务。启动后不能再调用创建电机的 `*_init(can)`。

`CAN_FilterInit` 返回 `bsp::can::Status`，调用方必须检查结果。各协议的过滤条件如下：

| 驱动 | 调用 | 接收 ID 条件 |
| --- | --- | --- |
| DJI | `DJIMotor::CAN_FilterInit(can, index)` | 标准 ID，`0x200 / 0x7F0` 掩码 |
| DM | `DMMotor::CAN_FilterInit(can, index, master_id)` | 标准 ID 精确匹配 `master_id`；所有 DM 共用该值 |
| MI | `MIMotor::CAN_FilterInit(can, index)` | 全部扩展 ID |
| VESC | `VESCMotor::CAN_FilterInit(can, index)` | 全部扩展 ID |

MI 与 VESC 混用时，一个放行扩展帧的过滤器即可。`index` 在 bxCAN 中是 filter bank，
在 FDCAN 中是对应标准/扩展过滤器数组的 element 下标，不能直接互换。
FDCAN 的过滤器数量与 FIFO0 资源须在 HAL 初始化前分配；未匹配帧是否拒收由板级
`Can::configure_filter(bsp::can::GlobalFilter{})` 策略决定。bxCAN 的 CAN1/CAN2 bank 分界由
`CAN_DRIVER_BXCAN_CAN2_START_BANK` 统一配置，且两路过滤器都须在任一路启动前配置。

下文各控制样例通过 `bsp::can::Can&` 参数借用板级总线，不引用隐式全局，也不运行独立的 `motor_demo_init()`。
板级先完成 HAL 初始化，再调用仅构造总线的 `motor_can_init()` 并持有返回的引用；依次调用所有需要的 `*_init(can)`，
然后配置过滤器并检查 `start()` 的结果，成功后才调用相应的 `*_start()` 使能控制器或发送命令。
函数内静态的 `Can` 存活到程序结束；所有电机也必须覆盖总线的使用期。已经启动的总线不能追加构造电机。

## 调用样例

### 关于构造顺序

实际项目里，`motor` 和 `controller` 往往不会写在同一个文件里；如果直接把它们都做成跨文件的全局对象，
就会遇到 C++ 静态 / 动态初始化顺序不确定的问题。

因此更推荐的做法是：

- 全局只保留指针或句柄
- 在显式的初始化函数里按顺序 `new`
- 先构造电机和控制器，再启动总线，最后才 `enable()` 或发送参考值

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

void wheel_init(bsp::can::Can& can)
{
    wheel_lf = new motors::DJIMotor({
            .can            = can,
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

}

void wheel_start() // 在所属总线 start() 成功之后调用
{
    wheel_lf_ctrl->enable();
    wheel_lf_ctrl->setRef(300.0f); // 目标速度 300 rpm
}

void wheel_task_1khz(bsp::can::Can& can)
{
    wheel_lf_ctrl->update();

    // 这里是“尽力而为”的用法：显式 (void) 丢弃返回值，不重试、也不等待发送完成，
    // 因此本周期完全可能未被受理（例如总线未 start()，或硬件已满且未启用软件发送队列）。
    // 如需关注发送质量，可改为检查返回的 bsp::can::SendResult：
    // Submitted / Queued / ReplacedOldest 表示已被受理，但不保证已真正在物理总线上发出；
    // 其中 ReplacedOldest 还意味着丢弃了一个更早排队的帧。Busy / NotStarted 等则代表本周期未被受理。
    (void)motors::DJIMotor::SendIqCommand(
            can, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
}
```

如果同一条 CAN 上同时挂了 `id1 = 5 ~ 8` 的 DJI 电机，还需要额外对 `IqCMDGroup_5_8` 再调用一次
`SendIqCommand(...)`。是否检查返回值由应用自行决定：本例选择尽力而为，不因发送失败而重试或停机。

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

void lift_init(bsp::can::Can& can)
{
    lift_motor = new motors::DMMotor({
            .can            = can,
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

}

void lift_start() // 在所属总线 start() 成功之后调用
{
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

void conveyor_init(bsp::can::Can& can)
{
    conveyor_motor = new motors::VESCMotor({
            .can            = can,
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

}

void conveyor_start() // 在所属总线 start() 成功之后调用
{
    conveyor_ctrl->enable();
    conveyor_ctrl->setRef(1200.0f); // 目标输出轴速度 1200 rpm
}

void conveyor_task_1khz()
{
    conveyor_ctrl->update();
}

void conveyor_debug_set_current()
{
    if (!conveyor_motor)
        return;

    conveyor_motor->setCurrent(5.0f); // 只下发电流
}

void conveyor_debug_set_velocity()
{
    if (!conveyor_motor)
        return;

    conveyor_motor->setInternalVelocity(800.0f, motors::IMotor::kNoLimit); // 只下发速度
}
```

调试时按需单独调用这两个函数即可：`conveyor_debug_set_current()` 只会下发电流指令，
`conveyor_debug_set_velocity()` 只会下发内部速度指令，两者互不影响。

### 样例 4：MI 走内部 MIT 运控模式

`MI` 的默认控制模式就是 `InternalMIT`（`defaultControlMode() == InternalMIT`、`supportsInternalMIT() == true`），
因此最典型的用法是在周期任务里直接下发 MIT 五元组；位置保持、轨迹跟踪和阻抗控制都由这个接口完成。
注意这个“默认模式”只描述驱动能接受什么输入：`MotorVelController` / `MotorPosController` 并不会因为
默认模式是 `InternalMIT` 就去调用 `setInternalMIT()`，具体见本节末尾。

```cpp
#include "mi.hpp"

namespace
{
motors::MIMotor* joint_motor = nullptr;
} // namespace

void joint_init(bsp::can::Can& can)
{
    joint_motor = new motors::MIMotor({
            .can            = can,
            .id             = 3,
            .master_id      = 0,
            .auto_zero      = true,
            .reverse        = false,
            .reduction_rate = 1.0f,
    });
}

void joint_start() // 在所属总线 start() 成功之后调用
{
    joint_motor->disable(true); // 先停转并清故障
    joint_motor->writeParam(0x7005, static_cast<float>(motors::MIMotor::Mode::MIT));
    joint_motor->enable();
}

void joint_task_1khz()
{
    if (!joint_motor || !joint_motor->isConnected())
        return;

    // t_ff 单位 Nm，p_ref 单位 deg，v_ref 单位 deg/s，kp / kd 为 MIT 刚度与阻尼
    const float t_ff = 0.0f;
    const float p_ref = 0.0f;
    const float v_ref = 0.0f;
    joint_motor->setInternalMIT(t_ff, p_ref, v_ref, 80.0f, 1.0f);
}

void joint_idle_task_1khz()
{
    // 未使能时用失能包保活，维持一收一回通信节奏
    if (joint_motor)
        joint_motor->ping();
}
```

如果改用 `MotorVelController` / `MotorPosController`，无论 `ctrl_mode` 显式写成 `InternalMIT`，还是写成
`Default`（MI 的默认模式会解析成 `InternalMIT`），这两个控制器都不会调用 `setInternalMIT()`，而是退化为
MCU 侧外部 PID，最终通过 `setCurrent()` 下发。二者退化之后的结构并不相同：`MotorVelController` 只用速度 PID；
`MotorPosController` 先用位置 PID 算出目标角速度，再用速度 PID 算出电流，即位置 + 速度串级 PID，
并不是两个控制器都做串级。需要用到驱动器内部的刚度 / 阻尼 / 前馈时，应在业务周期任务里直接调用
`setInternalMIT()`。

## 各驱动专项说明

主 README 只保留统一抽象、接入方式和最小样例。各电机家族自己的协议特点、约束和特殊说明，分别放在
各自目录下：

- [DJI 驱动说明](motors/DJI/README.md)
  重点说明聚合发送、反馈展开和圈数统计
- [DM 驱动说明](motors/DM/README.md)
  重点说明 `master_id`、`id0` 规划、`ping()` 语义和模式约束
- [MI 驱动说明](motors/MI/README.md)
  重点说明 CyberGear 协议帧、MIT 参数范围与参数读写
- [VESC 驱动说明](motors/VESC/README.md)
  重点说明 `ERPM`、`electrodes` 的真实含义和状态反馈字段解释

## 建议阅读顺序

如果第一次接触这个库，建议按下面顺序看：

1. `core/motor_if.hpp`
2. `controllers/`
3. 本 README 里的“设计理念”“单位约定”“调用样例”
4. `motors/DJI/README.md`
5. `motors/DM/README.md`
6. `motors/MI/README.md`
7. `motors/VESC/README.md`

这样更容易先理解能力模型，再理解各类电机的特殊约束。
