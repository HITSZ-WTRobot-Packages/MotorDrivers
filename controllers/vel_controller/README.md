# MotorController::VelController

本目录是通用速度控制器包，包名和依赖见 [cpkg.toml](cpkg.toml)：依赖 `MotorDrivers::Core`、`libs::PIDMotor`。类型为 `controllers::MotorVelController`，完整接口见 [motor_vel_controller.hpp 的 Doxygen](motor_vel_controller.hpp)。

## 参考量与模式

参考值和电机反馈均为输出轴 `rpm`。`ExternalPID` 使用速度 PID 输出低层电流 / 力矩；`InternalVel`、`InternalVelPos` 直接发送内部速度参考。`InternalPos` 不能启用速度控制器。`InternalMIT`（包括由 `Default` 解析得到的情况）仍走外部速度 PID，不调用 MIT 五元组接口。

PID 输出、前馈和 `max_current` 的量纲须匹配电机低层输入，不可统一视为安培。外部路径按临时输出上限限幅；内部速度路径把上限交给驱动，不支持该限制的协议会忽略它。内部速度路径不使用外部 PID 和前馈。

## 调度与安全边界

宿主先构造电机再构造控制器，确保电机存活更久；所有电机须在总线启动前完成构造和回调注册。总线启动成功后，按 `enable()` → `Protected` → `standalone()` → `setRef(...)` 激活直接控制，并检查 `enable()`、`standalone()` 和参考入口的返回值。`enable()` 会由 Core 使能并取得绑定电机，控制器自身首先进入保护态，不再直接接受业务参考；速度控制器在最终模式解析为 `InternalPos` 时返回失败。已有父控制器接管时，由父节点推进为 `Controlled`，不要另行调用 `standalone()`。

两个参考入口均为 `[[nodiscard]] bool`，原参考参数、默认值和量纲不变：

- `setRef(velocity, max_current = kNoLimit, ff = 0.0f)`：仅自身 `Standalone` 接受；`Controlled` 拒绝直接入口。
- `setRefFrom(source, velocity, max_current = kNoLimit, ff = 0.0f)`：仅自身 `Controlled` 且 `currentController() == &source` 接受；`Standalone` 拒绝此入口。父节点成员函数使用 `child.setRefFrom(*this, velocity, max_current, ff)`，并处理返回值；不要求父节点已使能，因此父节点保护钩子在自身状态尚未提交时仍可提交安全参考。
- 自身 `Disabled` / `Protected` 时两个入口都返回 `false`；拒绝发生在任何参考、限幅、前馈、分频状态修改及输出之前。

返回 `true` 仅表示接受参考，不代表 CAN 发送受理或硬件执行。内部速度路径接受新参考后仍立即发一次命令，不等待分频。`source` 是调用方显式声明，不是强身份认证；调用方可用公开的 owner 指针冒充父节点。此规则不保护 raw `IMotor` setter 或 `getMotor()` 直接访问电机的旁路。

`update()` 由宿主固定周期调用，只检查自身已使能；`Protected` 也继续执行安全参考闭环。使能时持有绑定电机的关系由 Core 保证，不额外查询控制者。内部速度命令按 `internal_set_ratio` 分频发送，配置应至少为 1；外部 PID 按更新周期运行。保护钩子清除 PID、参考、前馈、临时限幅与分频计数，速度参考归零，并立即向电机提交内部速度零或外部电流 / 力矩零；后续外部 PID 可为维持零速输出制动力，不保证电流始终为零。

`protect()` 把无控制者的 `Standalone` 节点转回 `Protected`，保留使能与子节点关系。`disable()`（即 `disable(false)`）沿控制链向上失能并释放直接子节点，绑定电机留在 `Protected` 而非自动失能。需要关闭整棵活动子树时，在无控制者的根上调用 `disableTree()`（等价于 `disable(true)`）；受控节点不能这样向下停机。所有生命周期调用都须检查返回值。`selfDisable()` 只清控制器业务状态，不自行释放或操作电机生命周期。

控制器不拥有或删除电机，析构不再自动释放控制权。销毁任意节点前，宿主必须显式失能成功；仅 `controller.disable()` 后还应妥善失能仍为 `Protected` 的电机，或在无控制者根上使用 `disableTree()`。停机或保护不等于机械停转，CAN 发送受理也不等于物理执行，安全策略与硬件联锁由宿主负责。

Core 的状态 / 控制关系只是瞬时快照，不保护业务数据或周期发包。宿主须串行化生命周期、`setRef()` / `setRefFrom()`、PID 配置 / 访问、`update()` 与直接电机命令，不得把控制权当作并发同步。

模式选择参见 [controllers 综述](../README.md)，统一激活与停机顺序见 [聚合包](../_/README.md)，单位与能力参见 [Core](../../core/README.md)，协议限制应同时阅读实际电机包。
