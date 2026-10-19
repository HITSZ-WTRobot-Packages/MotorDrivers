# DM 驱动说明

本目录是 `MotorDrivers::DM` 包，身份与依赖见 [cpkg.toml](cpkg.toml)，依赖 `MotorDrivers::Core`、`bsp::CANDriver2`、`services::Watchdog`。全仓接入见 [根 README](../../README.md#can-接入方式)，统一能力与控制权见 [Core](../../core/README.md)。具体接口以 [dm.hpp 的 Doxygen](dm.hpp) 为准。

## 当前覆盖

- `J4310_2EC`
- `J10010L_2EC`
- `S3519`
- `S2325_1EC`
- `J4340_2EC`
- `J8009P_2EC`

## 模式与能力的对应关系

`DM` 驱动器本身支持多种工作模式，而库里的统一能力接口会随模式一起变化。

- `Mode::MIT`
  - `defaultControlMode() == InternalMIT`
  - `supportsCurrent() == true`
  - `supportsInternalMIT() == true`
  - `setCurrent()` 的语义不是传统电流环，而是把 MIT 退化成“只发力矩”的低层输入
- `Mode::Pos`
  - `defaultControlMode() == InternalPos`
  - `internalPositionSupportLevel() == InternalPositionSupportLevel::RefAndMaxv`
  - 可调用 `setInternalPosition(pos, maxv)` 下发位置参考与最大速度
- `Mode::Vel`
  - `defaultControlMode() == InternalVel`
  - `internalVelocitySupportLevel() == InternalVelocitySupportLevel::RefOnly`
  - 可调用 `setInternalVelocity(rpm, maxi)`，但 DM 速度协议不支持 `maxi`，该参数被忽略

这也是为什么 `DM` 不会默认走 `InternalVelPos`：它不是“同一配置下既能发速度又能发位置”，而是驱动器固件
当前被配置成哪一种解释方式，就只暴露那一类能力。

宿主必须把电调实际模式与 `Config::mode` 配成一致；选择控制器模式不会自动修改电调固件模式。两个通用控制器即使使用 `InternalMIT`，也只走外部 PID 后调用 `setCurrent()`，不发送 MIT 五元组。直接使用五元组应由宿主协调控制权。

## CAN 帧格式

`Config::can_fd_brs` 按电调自身的通信配置选择发送帧格式：

- `false`（默认）：Classic CAN 数据帧，BRS 关闭。
- `true`：CAN FD 数据帧，BRS 开启。

使能、失能、`ping()`、MIT、位置和速度指令都使用这一配置；ID 与协议负载保持不变，
速度指令为 4 字节，其余指令为 8 字节。例如：

```cpp
motors::DMMotor::Config cfg{
        .can = can,
        .id0 = 0x09,
        .type = motors::DMMotor::Type::J4310_2EC,
        .mode = motors::DMMotor::Mode::MIT,
        .pos_max_rad = 12.5f,
        .vel_max_rad = 30.0f,
        .tor_max = 10.0f,
        .can_fd_brs = true,
};
```

开启后要求 FDCAN 后端，以及宿主通过 CubeMX 配置 HAL1 的 FD+BRS 模式和与电调一致的数据段位时序；驱动不提供外设重新初始化方案，也不保证已完整校验所有宿主配置。
在 bxCAN 后端开启会在构造时触发 `Error_Handler()`，不会尝试发送 FD 帧。
接收回调同时支持 Classic 与 FD 的标准 ID 数据帧，反馈仍须为 8 字节，且匹配 `master_id` 和自身 `id0`。

## 协议约束

### 1. `master_id` 必须全局一致

当前驱动设计明确要求：所有 `DM` 电机共享同一个 `master_id`。

`CAN_FilterInit(can, index, master_id)` 使用这一全局值配置过滤器及反馈匹配。
每个电机通过 `Can::register_callback` 注册成员回调，先匹配标准帧的 `master_id`，再匹配
`data[0]` 低 4 位的 `id0`；不再维护电机对象映射表。不同总线配置过滤器时也必须使用相同的 `master_id`。
电机须在总线启动前构造，并保持存活到总线不再使用；详见 [CAN 接入说明](../../README.md#can-接入方式)。

宿主负责分配过滤器资源。现有 `CAN_FilterInit` 会写入全局主机 ID，并实际配置精确匹配该标准 ID、进入 FIFO0 的硬件过滤器；不是只校验。必须在总线启动前完成并检查返回的 `bsp::can::Status`。索引是 bxCAN bank 或 FDCAN 标准过滤器 element，不可互换；不能因调用失败而假定全局主机 ID 未改变。配置校验与 CubeMX 边界的维护要求见根文档。

### 2. `id0` 建议使用 `0x09 ~ 0x0F`

`DM` 返回状态时：

- `data[0] = ERR << 4 | ID`
- 其中低 4 位才是真实电机 ID

发送控制帧时，标准帧 ID 为：

- 速度模式：`0x200 + ID`
- 位置模式：`0x100 + ID`
- MIT 模式：`0x000 + ID`

因此当前库建议把 `id0` 放在 `0x09 ~ 0x0F`，尽量避开 DJI 常见反馈 ID `0x201 ~ 0x208`。

## `ping()` 为什么复用使能 / 失能包

`DM` 当前按“一收一回”模式使用，因此库里需要一个周期性保活动作来维持通信节奏。

`DMMotor::ping()` 的实现并不是单独发一个“心跳包”，而是复用现成的使能 / 失能报文，按当前状态二选一：

- 尚未使能（`enabled_ == false`）：发送失能包，既能继续维持一收一回的通信节奏，又不会因为保活动作误把电机重新使能
- 已使能、但最近一次反馈的状态是 `State::Disabled`：重发使能包，把掉使能的电机拉回使能态

这样既符合当前驱动的状态机设计，也不需要额外定义一条“心跳”报文。

因此 `ping()` 的典型使用场景是：

- 电机当前还没被控制器接管，你又希望它保持通信在线
- 电机已被接管，但需要周期性确认并恢复使能状态

申请控制权时，驱动在检查控制权能否取得之前就可能尝试发送使能包，因此申请失败也不能当作“没有硬件副作用”的保证。释放控制权不会自动失能；现有 `enable()` 返回成功也不代表发送已受理或电机确认使能，宿主应结合反馈与故障状态判断。

## 单位与量程

这一路最容易混淆的是“协议量程”和“库对外接口单位”。

库对外的统一约定仍然是：

- 角度默认 `deg`
- 速度默认 `rpm`
- 力矩默认 `Nm`

但 `DM` 配置里这几个字段直接对应驱动器协议量程：

- `pos_max_rad`
- `vel_max_rad`
- `tor_max`

也就是说：

- 配置项里的 `pos_max_rad`、`vel_max_rad` 必须按驱动器要求填写 `rad` / `rad/s`
- 业务层调用 `getAngle()`、`getVelocity()`、`setInternalVelocity()` 时，仍按库统一约定理解
- `setCurrent()` 在 `MIT` 模式下应理解成力矩输入，单位 `Nm`

`setInternalMIT()` 需要特别说明：

- 这里的 `p_ref` 和 `v_ref` 是一组 MIT 状态参考
- `v_ref` 通常不是“内部速度模式下的目标转速”，而是和 `p_ref` 配套的速度项
- 在很多轨迹跟踪 / 阻抗控制场景里，`v_ref` 可以直接理解为 `p_ref` 的导数

也正因为语义不同，当前实现没有让 `setInternalMIT()` 复用普通速度接口的 `rpm` 约定，而是使用：

- `p_ref` 按 `deg`
- `v_ref` 按 `deg/s`

然后再由驱动内部换算成协议需要的 `rad` / `rad/s`。

## `S3519` 的位置反馈口径

`DM` 各型号的位置反馈口径并不完全一致：

- 除 `S3519` 外，反馈位置都是减速后的值
- `S3519` 返回的是减速前的位置，驱动内部按固定的 `19.203` 折算回输出轴

速度反馈对所有型号都是减速后的值，因此速度不做额外折算。

## 连接检测

和整个库的统一约定一致，`DM` 驱动的连接检测只通过 `Watchdog`：

- 成功解析反馈后喂狗
- `isConnected()` 只读取 `Watchdog` 状态

不再维护第二套“在线标志”。
