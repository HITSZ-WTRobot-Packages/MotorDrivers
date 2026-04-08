# DM 驱动说明

本页只说明 `motors/DM/` 这一路驱动自己的特点与约束。统一抽象、单位约定和最小调用样例，请先看
[../../README.md](../../README.md)。

## 当前覆盖

- `J4310_2EC`
- `J10010L_2EC`
- `S3519`

## 模式与能力的对应关系

`DM` 驱动器本身支持多种工作模式，而库里的统一能力接口会随模式一起变化。

- `Mode::MIT`
  - `defaultControlMode() == InternalMIT`
  - `supportsCurrent() == true`
  - `supportsInternalMIT() == true`
  - `setCurrent()` 的语义不是传统电流环，而是把 MIT 退化成“只发力矩”的低层输入
- `Mode::Pos`
  - `defaultControlMode() == InternalPos`
  - `supportsInternalPosition() == true`
- `Mode::Vel`
  - `defaultControlMode() == InternalVel`
  - `supportsInternalVelocity() == true`

这也是为什么 `DM` 不会默认走 `InternalVelPos`：它不是“同一配置下既能发速度又能发位置”，而是驱动器固件
当前被配置成哪一种解释方式，就只暴露那一类能力。

## 协议约束

### 1. `master_id` 必须全局一致

当前驱动设计明确要求：所有 `DM` 电机共享同一个 `master_id`。

这样做的原因不是协议绝对要求，而是当前库在回调分发、对象映射和多总线管理上都按这个前提设计。若允许
不同 `DM` 电机使用不同 `master_id`，整套驱动结构会明显复杂很多。

### 2. `id0` 建议使用 `0x09 ~ 0x0F`

`DM` 返回状态时：

- `data[0] = ERR << 4 | ID`
- 其中低 4 位才是真实电机 ID

发送控制帧时，标准帧 ID 为：

- 速度模式：`0x200 + ID`
- 位置模式：`0x100 + ID`
- MIT 模式：`0x000 + ID`

因此当前库建议把 `id0` 放在 `0x09 ~ 0x0F`，尽量避开 DJI 常见反馈 ID `0x201 ~ 0x208`。

## `ping()` 为什么发送失能包

`DM` 当前按“一收一回”模式使用，因此在未使能状态下，库里仍然需要一个周期性保活动作来维持通信节奏。

`DMMotor::ping()` 的实现并不是单独发一个“心跳包”，而是有意使用“失能包”代替。原因是：

- 能继续维持一收一回的通信节奏
- 不会因为保活动作误把电机重新使能
- 更符合当前驱动的状态机设计

因此 `ping()` 的典型使用场景是：

- 电机当前还没被控制器接管
- 你又希望它保持通信在线

如果控制器已经成功获取控制权，`tryAcquireController()` 会顺带尝试发送使能包。

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

## 连接检测

和整个库的统一约定一致，`DM` 驱动的连接检测只通过 `Watchdog`：

- 成功解析反馈后喂狗
- `isConnected()` 只读取 `Watchdog` 状态

不再维护第二套“在线标志”。
