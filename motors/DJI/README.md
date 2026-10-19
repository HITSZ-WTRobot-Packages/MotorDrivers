# DJI 驱动说明

本目录是 `MotorDrivers::DJI` 包，身份与依赖见 [cpkg.toml](cpkg.toml)，依赖 `MotorDrivers::Core`、`bsp::CANDriver2`、`services::Watchdog`。全仓接入见 [根 README](../../README.md#can-接入方式)，统一单位与控制权见 [Core](../../core/README.md)。具体 API 和实现以 [dji.hpp 的 Doxygen](dji.hpp) 为准。

## 当前覆盖

- `M3508_C620`
- `M2006_C610`

## 能力模型

`DJI` 这一路在当前实现中主要提供：

- 角度反馈
- 速度反馈
- 低层电流输入

对应到统一接口上：

- `supportsCurrent() == true`
- `internalVelocitySupportLevel() == InternalVelocitySupportLevel::NotSupported`
- `internalPositionSupportLevel() == InternalPositionSupportLevel::NotSupported`
- `supportsInternalMIT() == false`
- `defaultControlMode() == ExternalPID`

这意味着 `DJI` 电机最典型的搭配方式是外部控制器自己闭环，然后通过 `setCurrent()` 给出输出。

## 接入与硬件边界

宿主先通过 CubeMX 配置 HAL1 CAN 外设、IRQ 与接收资源，完成 HAL 初始化后构造 `Can`，再把尚未启动的 `Can&` 借给电机。过滤器索引由宿主分配：现有 `CAN_FilterInit(can, index)` 实际配置标准 ID `0x200 / 0x7F0` 掩码并送 FIFO0，必须在启动前调用并检查返回状态，不是只校验配置。bxCAN bank 与 FDCAN element 的含义不同；完整资源及公共维护规则见根文档，不在本包重复初始化外设。

`setCurrent()` 缓存的是协议电流指令，不是统一以安培解释的物理电流。PID 与限幅应按电调协议范围设置；驱动不在此接口中替宿主实现安全限幅。

## 聚合发送

`DJI` 和 `DM` / `VESC` 的一个重要区别是：`setCurrent()` 不会马上发出一帧 CAN 报文。

当前实现的流程是：

- 电机构造时按借用总线与电机 ID 自动登记发送槽位，不分配堆内存。
- 每个对象缓存自己的目标电流。
- `DJIMotor::SendIqCommand(can, group)` 聚合本组 4 台电机并发送，调用方无需维护电机列表。

DJI 协议一次报文携带 4 台电机的电流命令，因此分为两组：

- `IqCMDGroup_1_4`：`id1 = 1 ~ 4`，发送 ID `0x200`。
- `IqCMDGroup_5_8`：`id1 = 5 ~ 8`，发送 ID `0x1FF`。

可在每个控制周期发送两组，这样修改电机的构造配置 `id1` 时，不需要修改发送任务：

```cpp
const auto result_1_4 = motors::DJIMotor::SendIqCommand(
        can, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_1_4);
const auto result_5_8 = motors::DJIMotor::SendIqCommand(
        can, motors::DJIMotor::IqSetCMDGroup::IqCMDGroup_5_8);
```

缺少电机或未受控的槽位发送零电流；总线上没有 DJI 电机时也发送对应组的全零命令。
同一总线上的重复 `id1` 会在构造时触发 `Error_Handler()`，不同总线可以使用相同 `id1`。
发送登记最多支持 `CAN_DRIVER_MAX_INSTANCES` 条总线；注册接收回调失败不会提交发送槽位。
返回值为 `bsp::can::SendResult`，受理或排队不代表已经在线上发送成功。

发送槽位只用于组包，反馈仍由 `Can::register_callback` 注册的成员回调接收，不经过发送登记表。
所有电机须在所属 `Can::start()` 前构造并保持存活；析构仅释放发送槽位，不注销接收回调。
最后一台电机析构后释放该总线的发送登记容量。详见 [CAN 接入说明](../../README.md#can-接入方式)。

## ID 与反馈范围

当前驱动约定：

- `id1` 的取值范围是 `1 ~ 8`
- 对应反馈标准帧 ID `0x201 ~ 0x208`

也就是说，`id1 = 1` 对应 `0x201`，`id1 = 8` 对应 `0x208`。

## 单圈机械角度如何展开成连续角度

`DJI` 的反馈只直接给出电机反馈侧的单圈机械角度，因此驱动需要结合相邻帧差分，把它展开成连续角度，
再按减速比换算到输出轴。

展开以相邻帧的最短角差为依据，要求两帧间反馈侧真实转角绝对值小于 `180 deg`。丢帧、反馈过慢或速度超出这一假设时可能丢圈，驱动无法恢复没有观测到的完整圈数；这不是任意速度下的绝对多圈编码器。

作为量级参考，按输出轴约 `500 rpm` 和 `1 kHz` 连续反馈估算：M3508_C620 内减速比约 `3591 / 187 ≈ 19.2`，反馈侧每毫秒约转 `57.6 deg`；M2006_C610 内减速比 `36`，约转 `108 deg`。这些条件下满足角差要求，不是对宿主反馈完整性或控制频率的保证。实际展开与圈数处理见源码 Doxygen。

`auto_zero` 在收到一段反馈后建立软件零点；`reduction_rate` 描述外接减速比。反馈方向和控制方向由 `reverse` 一起处理，宿主不要重复反转。

## 连接检测

和整个库的统一约定一致，`DJI` 驱动的连接检测只通过 `Watchdog`：

- 成功解析反馈后喂狗
- `isConnected()` 只读取 `Watchdog` 状态

不再额外维护第二套“在线标志”。
