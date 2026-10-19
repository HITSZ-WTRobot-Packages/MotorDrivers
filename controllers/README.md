# 通用电机控制器

本目录是控制器包的综述，不是 cpkg 包。控制器借用 `MotorDrivers::Core` 定义的电机接口，PID 算法依赖 `libs::PIDMotor`；不拥有电机，不初始化 HAL 或 CAN。

| 包 | 文档 | 使用场景 |
| --- | --- | --- |
| `MotorController::VelController` | [vel_controller](vel_controller/README.md) | 输出轴速度参考 |
| `MotorController::PosController` | [pos_controller](pos_controller/README.md) | 输出轴角度参考 |
| `MotorDrivers::Controller` | [_](_/README.md) | 同时依赖两个控制器的聚合入口 |

## 跨包控制路径

| 选择的模式 | 速度控制器 | 位置控制器 |
| --- | --- | --- |
| `ExternalPID` | 速度 PID → 电流 / 力矩 | 位置 PID → 速度 PID → 电流 / 力矩 |
| `InternalVel` | 直接发送速度参考 | 位置 PID → 驱动内部速度输入 |
| `InternalPos` | 不支持，`enable()` 返回 false | 直接发送位置参考 |
| `InternalVelPos` | 使用内部速度输入 | 使用内部位置输入 |
| `InternalMIT` | 外部速度 PID → 电流 / 力矩 | 外部位置 + 速度串级 PID → 电流 / 力矩 |

`Default` 先解析为驱动默认模式，再走对应路径。MIT 默认模式不意味着通用控制器会调用 `setInternalMIT()`；需要 MIT 刚度、阻尼、前馈时，应直接使用电机包的五元组能力，并与控制器写入协调。

## 共同接入约束

宿主显式按“电机 → 控制器”构造，完成 CAN 接入并检查启动成功后再启用、设参考值和周期更新。不要依赖跨翻译单元的全局动态初始化顺序，也不要求必须使用动态分配。

`enable()` 的结果表示是否取得控制权，不是设备在线或指令已发送的确认；`update()` 在控制器未使能时不输出，但内部速度 / 位置路径的 `setRef()` 会立即调用电机接口，且不检查使能状态。宿主必须在安全条件满足时才设置这类参考值。`disable()` 释放控制权，不保证电机立即停转或失能。

宿主提供固定调用周期、适配该周期的 PID 和有效分频（至少为 1）。输出单位、限幅、驱动内部模式与能力须匹配；控制器不会自动配置电调模式。电机须比控制器长寿，任务 / ISR 间的并发访问由宿主协调。

单位与控制权见 [Core](../core/README.md)，总线资源与寿命见 [根 README](../README.md#can-接入方式)。各包 README 只维护消费者契约，具体参数和实现见各自头文件 Doxygen。
