# MotorController::PosController

本目录是通用位置控制器包，包名和依赖见 [cpkg.toml](cpkg.toml)：依赖 `MotorDrivers::Core`、`libs::PIDMotor`。类型为 `controllers::MotorPosController`，完整接口见 [motor_pos_controller.hpp 的 Doxygen](motor_pos_controller.hpp)。

## 参考量与控制路径

位置参考是输出轴 `deg`，最大速度限制为输出轴 `rpm`。位置 PID 输出目标角速度，速度 PID 输出所选驱动的低层电流 / 力矩；PID 量纲和限幅必须据此配置。

- `InternalPos` / `InternalVelPos`：直接向驱动发送位置与最大速度；只有驱动声明支持最大速度限制时才可依赖该限制。
- `InternalVel`：MCU 的位置 PID 计算速度，再使用驱动器内部速度输入；此路径发送速度时不附加电流 / 力矩上限。
- `ExternalPID` / `InternalMIT`：位置 PID 与速度 PID 串级，最终调用 `setCurrent()`；不发送 MIT 五元组。`Default` 按驱动默认模式解析。

## 调度与安全边界

宿主先构造电机再构造控制器，确保电机存活更久；所有电机须在总线启动前完成构造和回调注册。完成总线启动并检查成功后，按 `enable()` → `Protected` → `standalone()` → `setRef(...)` 激活直接控制，并检查 `enable()`、`standalone()` 和参考入口的返回值。Core 会使能并取得绑定电机；控制器 `enable()` 成功不再意味着可以直接设置业务参考。已有父控制器接管时，由父节点推进为 `Controlled`，不要另行调用 `standalone()`。

`update()` 由宿主按固定周期调用。直接位置输入按 `internal_set_ratio` 分频；位置 PID 按 `pos_vel_freq_ratio` 分频。使用内部速度输入时，速度发送频率跟随位置环，而非内部位置发送分频。外部串级路径的速度 PID 每次有效更新均运行。两个分频配置都应至少为 1，PID 参数应匹配实际计算周期。

两个参考入口均为 `[[nodiscard]] bool`，原参考参数、默认值和量纲不变：

- `setRef(position, max_velocity = kNoLimit)`：仅自身 `Standalone` 接受；`Controlled` 拒绝直接入口。
- `setRefFrom(source, position, max_velocity = kNoLimit)`：仅自身 `Controlled` 且 `currentController() == &source` 接受；`Standalone` 拒绝此入口。父节点成员函数使用 `child.setRefFrom(*this, position, max_velocity)`，并处理返回值；不要求父节点已使能，因此父节点保护钩子在自身状态尚未提交时仍可提交安全参考。
- 自身 `Disabled` / `Protected` 时两个入口都返回 `false`；拒绝发生在任何参考、限幅、分频状态修改及输出之前。

返回 `true` 仅表示接受参考，不代表 CAN 发送受理或硬件执行。内部位置路径接受新参考时仍立即发一次位置命令，不等待分频。`source` 是调用方显式声明，不是强身份认证；调用方可用公开的 owner 指针冒充父节点。此规则不保护 raw `IMotor` setter 或 `getMotor()` 直接访问电机的旁路。

`update()` 只检查自身已使能，`Protected` 下继续使用安全参考闭环。使能时持有电机的关系由 Core 保证，不额外查询控制者。保护钩子把位置参考采样为当前 `getAngle()`，清两个 PID、临时速度上限和两个分频计数。仅内部模式立即发送保护指令：内部位置路径保持该角度，内部速度路径发送零速。外部路径只更新保护参考与状态，不立即计算 PID 或下发电流 / 力矩，由后续 `update()` 按既定调度计算并下发闭环输出。保护期间位置参考固定为进入保护时的采样角度，后续位置闭环持续纠正偏移，不随反馈重新采样；保护是主动位置保持，不是泄力。

`protect()` 把无控制者的 `Standalone` 节点转回 `Protected`，保留使能与子节点关系。`disable()`（即 `disable(false)`）沿控制链向上失能并释放直接子节点，绑定电机保留为 `Protected`。在无控制者根上调用 `disableTree()`（等价于 `disable(true)`）才会向下关闭活动子树；受控节点不能这样向下停机。所有生命周期调用须检查返回值。`selfDisable()` 仅清自身业务状态，不自行释放或改变电机生命周期。

控制器不拥有或删除电机，析构不再自动释放控制权。销毁任意节点前，宿主必须显式失能成功；若只调用 `controller.disable()`，还须妥善失能保持 `Protected` 的电机，或在无控制者根上使用 `disableTree()`。保护 / 停机不等于机械停转，内部限制不代替机构限位和宿主安全策略；CAN 发送受理也不等于物理执行。

Core 的状态 / 控制关系只是瞬时快照，不保护业务数据或周期发包。宿主须串行化生命周期、`setRef()` / `setRefFrom()`、PID 配置 / 访问、`update()` 和直接电机命令，不得把控制权仲裁当作线程同步。

模式选择参见 [controllers 综述](../README.md)，统一激活与停机顺序见 [聚合包](../_/README.md)，统一单位与控制权参见 [Core](../../core/README.md)，设备模式、量程与零点要求见实际电机包。
