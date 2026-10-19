# MotorController::PosController

本目录是通用位置控制器包，包名和依赖见 [cpkg.toml](cpkg.toml)：依赖 `MotorDrivers::Core`、`libs::PIDMotor`。类型为 `controllers::MotorPosController`，完整接口见 [motor_pos_controller.hpp 的 Doxygen](motor_pos_controller.hpp)。

## 参考量与控制路径

位置参考是输出轴 `deg`，最大速度限制为输出轴 `rpm`。位置 PID 输出目标角速度，速度 PID 输出所选驱动的低层电流 / 力矩；PID 量纲和限幅必须据此配置。

- `InternalPos` / `InternalVelPos`：直接向驱动发送位置与最大速度；只有驱动声明支持最大速度限制时才可依赖该限制。
- `InternalVel`：MCU 的位置 PID 计算速度，再使用驱动器内部速度输入；此路径发送速度时不附加电流 / 力矩上限。
- `ExternalPID` / `InternalMIT`：位置 PID 与速度 PID 串级，最终调用 `setCurrent()`；不发送 MIT 五元组。`Default` 按驱动默认模式解析。

## 调度与安全边界

宿主先构造电机再构造控制器，确保电机存活更久。完成总线启动并检查成功后才启用、设置内部位置参考和周期更新；应检查 `enable()` 是否成功取得控制权。

`update()` 由宿主按固定周期调用。直接位置输入按 `internal_set_ratio` 分频；位置 PID 按 `pos_vel_freq_ratio` 分频。使用内部速度输入时，速度发送频率跟随位置环，而非内部位置发送分频。外部串级路径的速度 PID 每次有效更新均运行。两个分频配置都应至少为 1，PID 参数应匹配实际计算周期。

修改内部位置参考时立即发一次位置命令，不等待分频，而且 `setRef()` 不检查控制器是否已启用。`update()` 的未使能保护不能作为所有输出的安全门；宿主必须协调设参考、直接驱动调用和控制器更新。

控制器析构释放控制权但不删除电机。停用 / 释放控制权不等于电机失能或停转，内部限制不代替机构限位和宿主安全策略。CAN 发送结果也不会通过这两个无返回值的控制接口确认。

模式选择参见 [controllers 综述](../README.md)，统一单位与控制权参见 [Core](../../core/README.md)，设备模式、量程与零点要求见实际电机包。
