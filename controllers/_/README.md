# MotorDrivers::Controller

本目录是控制器聚合包，包名和依赖见 [cpkg.toml](cpkg.toml)。它依赖：

- [MotorController::VelController](../vel_controller/README.md)
- [MotorController::PosController](../pos_controller/README.md)

该包没有独立的控制器实现或另一套 API；宿主需要同时使用两种控制器时可依赖此入口，只需要一种时可直接依赖相应包。它不代替电机协议包，也不自动选择电机或配置外设。

## 激活、保护与停机

两种控制器均借用电机，电机须比控制器存活更久。电机在 CAN 启动前构造并注册回调；总线启动成功后，直接控制顺序为 `enable()` → `Protected` → `standalone()` → `setRef(...)`。`enable()`、`standalone()` 和参考入口都必须检查返回值；若由父控制器接管，则由 Core 取得控制权并推进为 `Controlled`，不再调用 `standalone()`。旧的 `enable()` 后立刻 `setRef()` 用法必须增加 `standalone()`，旧 `enabled()` 查询改用 Core 的 `isEnabled()` / 状态查询；不再手工申请或释放控制权。

两个参考入口均返回 `[[nodiscard]] bool`，原参考参数、默认值和量纲不变；`setRefFrom()` 仅在首参数增加 `const core::control::ControllerNode& source`：

| 控制器自身状态 | `setRef(...)` | `setRefFrom(source, ...)` |
| --- | --- | --- |
| `Disabled` / `Protected` | 拒绝 | 拒绝 |
| `Standalone` | 接受 | 拒绝 |
| `Controlled` | 拒绝 | 仅 `currentController() == &source` 时接受 |

父节点成员函数应调用 `child.setRefFrom(*this, ...)` 并处理返回值，不能继续直接调用受控 child 的 `setRef(...)`。只匹配当前父节点，不额外要求父节点已使能，允许父节点保护钩子在自身状态尚未提交时提交安全参考。拒绝返回 `false`，发生在任何参考、限幅、前馈、分频状态修改及输出之前；`true` 仅表示接受参考，不代表发送受理或硬件执行。接受后的内部模式立即发送行为不变。

`source` 是调用方显式声明，不是强身份认证；公开 owner 指针可被用于故意冒充父节点。此门控也不保护 raw `IMotor` setter 或 `getMotor()` 电机访问旁路，不能代替业务串行化。

`protect()` 建立安全参考并从 `Standalone` 回到 `Protected`，速度归零、位置保持采样角度；`update()` 在自身已使能时继续执行，包括保护态的安全闭环。使能时持有绑定电机的关系由 Core 保证，不做额外校验。

- `disable()` / `disable(false)`：失能当前节点及其上游控制链，释放直接子节点；绑定电机保持使能并进入 `Protected`，不等于关闭整棵树。
- `disableTree()` / `disable(true)`：仅可从无控制者的根调用，向下失能活动子树；必须检查返回值。
- 析构不自动释放关系。销毁控制器和电机前，宿主须显式失能成功；仅向上失能控制器后，仍需处理保持 `Protected` 的电机。

宿主必须串行化生命周期、`setRef()` / `setRefFrom()`、周期更新、PID 访问和直接电机命令；Core 快照不提供业务并发保护。保护和失能不保证机械停转，CAN 受理不等于物理执行，实际限制以电机协议包和宿主安全策略为准。

控制路径、对象寿命和启停边界见 [controllers 综述](../README.md)，全仓包导航与 CAN 接入见 [根 README](../../README.md)。
