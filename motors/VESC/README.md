# VESC 驱动说明

本目录是 `MotorDrivers::VESC` 包，身份与依赖见 [cpkg.toml](cpkg.toml)，依赖 `MotorDrivers::Core`、`bsp::CANDriver2`、`services::Watchdog`。全仓接入见 [根 README](../../README.md#can-接入方式)，单位与能力见 [Core](../../core/README.md)。具体 API 和字段注释以 [vesc.hpp 的 Doxygen](vesc.hpp) 为准。

## 当前覆盖

当前 `VESC` 驱动重点覆盖：

- 电流控制
- 内部速度控制
- 常见状态反馈解析

## 能力模型

对应到统一接口上，当前实现主要提供：

- 角度反馈
- 速度反馈
- 电流输入
- 内部速度输入

也就是说：

- `supportsCurrent() == true`
- `internalVelocitySupportLevel() == InternalVelocitySupportLevel::RefOnly`
- `internalPositionSupportLevel() == InternalPositionSupportLevel::NotSupported`
- `supportsInternalMIT() == false`
- `defaultControlMode() == ExternalPID`

虽然 VESC 协议里本身存在位置相关指令，但当前这个类没有把它作为稳定接口对外开放。

## 接入与硬件边界

驱动借用尚未启动的 `bsp::can::Can&`，每台电机按自身 ID 注册一个接收回调；没有析构解绑接口，不可复制、移动或在总线仍使用时销毁。控制器和总线寿命要求见根文档。

宿主通过 CubeMX 配置 HAL1 外设时序、IRQ、扩展帧接收资源，并分配过滤器索引。现有 `CAN_FilterInit(can, index)` 实际配置放行全部扩展 ID、送 FIFO0 的过滤器，不是只校验；须在总线启动前完成并检查状态。MI 与 VESC 可共享放行扩展帧的过滤器，但仍须避免协议 / ID 冲突。索引的 bxCAN bank 与 FDCAN 扩展 element 语义不同，不提供固定实例或手动外设初始化示例。

电流接口按安培理解，速度接口按输出轴 `rpm`。默认控制模式仍是 `ExternalPID`，若希望驱动器内部速度闭环，应显式选择内部速度路径并确认电调配置。直接下发电流与速度是替代的控制动作，不应一边由控制器更新、一边另行直接写入。

## 生命周期与停机边界

`VESCMotor` 继承 Core `IController<0>`，初始为 `Disabled`；`enable()` 成功后进入
`Protected` 并提交零电流命令。独立操作必须随后成功调用 `standalone()`；由控制器接管时，
控制关系由 Core 管理。公开 `setCurrent()` / `setInternalVelocity()` 仅在 `Standalone` /
`Controlled` 接受写入，不鉴别调用者来自控制器还是直接调用；宿主负责避免业务互相覆盖。
原始设置帧发送 helper 仅供驱动内部使用，不能从公开 API 绕过状态门控。

示例中 `can` 与 `motor` 已按上述顺序构造、配置过滤器并成功启动总线；所有调用均由宿主串行执行：

```cpp
if (!motor.enable() || !motor.standalone())
    Error_Handler();
motor.setCurrent(1.0f); // A；正常业务维持原来的即时发送语义
if (!motor.protect())
    Error_Handler(); // 立即提交零电流，不再接受新的公开目标写入
if (!motor.disable())
    Error_Handler(); // 销毁前必须显式失能，且不再使用借用对象的 CAN 回调
```

保护和失能均绕过公开命令门控，直接发送原 VESC `SetCurrent` 的零值帧，不改扩展帧 ID、
4 字节大端负载、单位或缩放；不发送独立的电调硬件失能指令，也不表示机械制动。
仅 `Submitted` / `Queued` / `ReplacedOldest` 视为安全帧受理成功；其他结果调用
`Error_Handler()`，不静默吞掉、不重试、不等待硬件反馈。普通业务命令仍为无返回值发送，
不新增重试。发送受理不等于物理执行或停转，必须另行满足硬件安全要求。

钩子不会启动总线或注册回调，宿主必须在使能前成功 `Can::start()`。Core 快照只管理状态与
控制关系，不保证业务并发；生命周期、业务命令由宿主串行化。析构不自动停机或释放控制关系，
所有相关节点须先显式失能，电机与总线仍须满足借用回调的寿命要求。

## `ERPM` 和 `electrodes` 的真实含义

`VESC` 最容易误解的地方，是速度并不是直接按机械 `rpm` 在协议里传输，而是按 `ERPM`
（Electrical RPM，电角转速）传输。

当前库对外仍统一使用输出轴机械 `rpm`，驱动内部会完成换算：

- 发送速度时：输出轴机械 `rpm -> ERPM`
- 解析反馈时：`ERPM -> 输出轴机械 rpm`

其中配置项 `electrodes` 的真实含义是“极对数”，不是总极数。

例如：

- 电机总极数为 `14`
- 则这里应填写 `7`

`electrodes` 是当前接口字段名，填写时以极对数语义为准。

## 减速比与速度换算

`VESC` 驱动里速度换算同时考虑：

- 电机极对数
- 外接减速比

可以把它简单理解成：

- `ERPM = 输出轴机械 rpm * 减速比 * 极对数`

因此：

- `getVelocity()` 返回的是输出轴速度
- `setInternalVelocity()` 传入的也是输出轴速度
- `setInternalVelocity()` 的 `maxi` 参数被忽略：VESC 内部速度模式只支持给定目标转速（`RefOnly`）

## 状态反馈说明

当前接收 `Status 1 ~ 5`，解释状态时须保留以下限制：

- `Status 1` 的 `ERPM`、总电流、占空比解释相对明确。
- `Status 4` 的位置字段，当前按 `0 ~ 360 deg` 单圈位置使用。
- `Status 5` 的 tachometer 字段目前仅按原始计数保存，不是已标定的输出轴角度。
- 一部分字段解释仍基于公开资料与推测，不能把它们当成已验证的测量语义。具体字段和不确定性见头文件现有注释。

## 连接检测

和整个库的统一约定一致，`VESC` 驱动的连接检测只通过 `Watchdog`：

- 成功解析反馈后喂狗
- `isConnected()` 只读取 `Watchdog` 状态

不再维护第二套“在线标志”。
