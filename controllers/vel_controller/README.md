# MotorController::VelController

本目录是通用速度控制器包，包名和依赖见 [cpkg.toml](cpkg.toml)：依赖 `MotorDrivers::Core`、`libs::PIDMotor`。类型为 `controllers::MotorVelController`，完整接口见 [motor_vel_controller.hpp 的 Doxygen](motor_vel_controller.hpp)。

## 参考量与模式

参考值和电机反馈均为输出轴 `rpm`。`ExternalPID` 使用速度 PID 输出低层电流 / 力矩；`InternalVel`、`InternalVelPos` 直接发送内部速度参考。`InternalPos` 不能启用速度控制器。`InternalMIT`（包括由 `Default` 解析得到的情况）仍走外部速度 PID，不调用 MIT 五元组接口。

PID 输出、前馈和 `max_current` 的量纲须匹配电机低层输入，不可统一视为安培。外部路径按临时输出上限限幅；内部速度路径把上限交给驱动，不支持该限制的协议会忽略它。内部速度路径不使用外部 PID 和前馈。

## 调度与安全边界

宿主先构造电机再构造控制器，确保电机存活更久。总线启动成功后才设置内部速度参考、启用控制器和进行周期调用；`enable()` 必须检查返回值。

`update()` 由宿主固定周期调用。内部速度命令按 `internal_set_ratio` 分频发送，配置应至少为 1；外部 PID 按更新周期运行。修改内部速度参考时会立即发一次命令，不等待分频，而且 `setRef()` 不检查控制器是否已启用。不能依赖 `disable()` 屏蔽直接设参考值的硬件输出。

控制器析构释放控制权但不拥有或删除电机。停用或释放控制权不保证驱动失能或停转；宿主负责设备状态与安全策略。CAN 发送是否受理也不由 `setRef()` 或 `update()` 的返回值确认。

模式选择参见 [controllers 综述](../README.md)，单位与能力参见 [Core](../../core/README.md)，协议限制应同时阅读实际电机包。
