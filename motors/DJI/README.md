# DJI 驱动说明

本页只说明 `motors/DJI/` 这一路驱动自己的特点与约束。统一抽象、单位约定和最小调用样例，请先看
[../../README.md](../../README.md)。

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
- `supportsInternalVelocity() == false`
- `supportsInternalPosition() == false`
- `supportsInternalMIT() == false`
- `defaultControlMode() == ExternalPID`

这意味着 `DJI` 电机最典型的搭配方式是外部控制器自己闭环，然后通过 `setCurrent()` 给出输出。

## 聚合发送

`DJI` 和 `DM` / `VESC` 的一个重要区别是：`setCurrent()` 不会马上发出一帧 CAN 报文。

当前实现的流程是：

- 每个 `DJIMotor` 对象先缓存自己的目标电流
- 然后由 `DJIMotor::SendIqCommand(...)` 把同一组里的 4 台电机一起打包发送

这是因为 DJI 标准协议一次报文能携带 4 台电机的电流命令，因此驱动里按两组处理：

- `IqCMDGroup_1_4`
- `IqCMDGroup_5_8`

如果一条总线上同时挂了 `id1 = 1 ~ 8` 的电机，通常需要每个控制周期分别对两组都调用一次
`SendIqCommand(...)`。

## ID 与反馈范围

当前驱动约定：

- `id1` 的取值范围是 `1 ~ 8`
- 对应反馈标准帧 ID `0x201 ~ 0x208`

也就是说，`id1 = 1` 对应 `0x201`，`id1 = 8` 对应 `0x208`。

## 单圈机械角度如何展开成连续角度

`DJI` 的反馈只直接给出电机反馈侧的单圈机械角度，因此驱动需要结合相邻帧差分，把它展开成连续角度，
再按减速比换算到输出轴。

展开规则如下：

- `feedback_angle_delta = new_angle - old_angle`
- 如果 `feedback_angle_delta < -180 deg`，认为角度从接近 `360 deg` 正向跨过了 `0 deg`，因此
  `round_cnt++`
- 如果 `feedback_angle_delta > 180 deg`，认为角度从接近 `0 deg` 反向跨过了 `0 deg`，因此
  `round_cnt--`

对当前已支持的两个型号，按“减速后的输出最大速度约 500 rpm、反馈频率 1 kHz”估算，相邻两帧的最大转角为：

- `M3508_C620`
  内部减速比约为 `3591 / 187 ~= 19.2`
- 输出轴最大约 `500 rpm`
- 反馈侧最大约 `500 * 19.2 = 9600 rpm`
- 也就是约 `160 rps ~= 57600 deg/s`
- 在 `1 ms` 内约转过 `57.6 deg`

- `M2006_C610`
  内部减速比为 `36`
- 输出轴最大约 `500 rpm`
- 反馈侧最大约 `500 * 36 = 18000 rpm`
- 也就是约 `300 rps = 108000 deg/s`
- 在 `1 ms` 内约转过 `108 deg`

因此在 `1 kHz` 反馈下：

- 相邻两帧的真实转角绝对值小于 `180 deg`
- 不会把正常转动误判成一次过零
- 不会出现“一帧跨过多圈”的情况
- `round_cnt` 每帧至多变化一次

## 连接检测

和整个库的统一约定一致，`DJI` 驱动的连接检测只通过 `Watchdog`：

- 成功解析反馈后喂狗
- `isConnected()` 只读取 `Watchdog` 状态

不再额外维护第二套“在线标志”。
