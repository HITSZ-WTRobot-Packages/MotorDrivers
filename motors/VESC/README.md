# VESC 驱动说明

本页只说明 `motors/VESC/` 这一路驱动自己的特点与约束。统一抽象、单位约定和最小调用样例，请先看
[../../README.md](../../README.md)。

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
- `supportsInternalVelocity() == true`
- `supportsInternalPosition() == false`
- `supportsInternalMIT() == false`

虽然 VESC 协议里本身存在位置相关指令，但当前这个类没有把它作为稳定接口对外开放。

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

字段名 `electrodes` 沿用了旧命名，当前没有改接口名，是为了兼容现有代码。

## 减速比与速度换算

`VESC` 驱动里速度换算同时考虑：

- 电机极对数
- 外接减速比

可以把它简单理解成：

- `ERPM = 输出轴机械 rpm * 减速比 * 极对数`

因此：

- `getVelocity()` 返回的是输出轴速度
- `setInternalVelocity()` 传入的也是输出轴速度

## 状态反馈说明

当前实现已经整理了 `Status 1 ~ 5` 的主要字段注释，但需要特别注意：

- `Status 1` 里的 `ERPM`、总电流、占空比解释相对明确
- `Status 4` 里的位置字段，当前实现把它按 `0 ~ 360 deg` 单圈位置使用
- `Status 5` 里的 `tachometer_value` 目前仅按原始计数保存

其中一部分字段解释仍带“推测”成分。源码里已经用中文注释明确标出哪些字段是“按当前公开资料 + 当前
实现”整理出来的，哪些地方还不能完全确认。

## 连接检测

和整个库的统一约定一致，`VESC` 驱动的连接检测只通过 `Watchdog`：

- 成功解析反馈后喂狗
- `isConnected()` 只读取 `Watchdog` 状态

不再维护第二套“在线标志”。
