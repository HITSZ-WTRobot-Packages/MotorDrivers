# MotorDrivers

`MotorDrivers` 是面向宿主 STM32 项目的共享电机驱动子仓库，仅作为 Git SubModule 使用，不独立编译或发行独立固件 SDK。依赖解析与固件构建由宿主的 cpkg 流程完成；本仓库没有独立构建入口，也不承诺任意平台或所有 STM32 系列均可直接编译。

它把不同驱动的反馈、控制输入、控制权和控制器接入统一为能力模型，让上层尽量不按品牌分支。电机与控制器的生命周期统一接入 `Core::IController`。抽象不限定总线，但当前具体驱动均使用 CAN。HAL1、CANDriver2、Watchdog、PIDMotor 等依赖以各包 `cpkg.toml` 为准。

## 目录与包导航

含 `cpkg.toml` 的目录才是包；根目录、`controllers/` 和 `motors/` 是综述层，不是包。

| 包名 | 就地文档 | 职责 |
| --- | --- | --- |
| `MotorDrivers::Core` | [core](core/README.md) | 电机能力、控制模式、控制器基类、单位与控制权 |
| `MotorController::VelController` | [controllers/vel_controller](controllers/vel_controller/README.md) | 通用速度控制器 |
| `MotorController::PosController` | [controllers/pos_controller](controllers/pos_controller/README.md) | 通用位置控制器 |
| `MotorDrivers::Controller` | [controllers/_](controllers/_/README.md) | 聚合两个控制器包 |
| `MotorDrivers::DJI` | [motors/DJI](motors/DJI/README.md) | M3508_C620、M2006_C610；聚合电流发送 |
| `MotorDrivers::DM` | [motors/DM](motors/DM/README.md) | J4310_2EC、J10010L_2EC、S3519、S2325_1EC、J4340_2EC、J8009P_2EC |
| `MotorDrivers::MIMotor` | [motors/MI](motors/MI/README.md) | CyberGear MIT 运控、零位和参数访问 |
| `MotorDrivers::VESC` | [motors/VESC](motors/VESC/README.md) | 电流、内部速度与状态反馈 |

[controllers 综述](controllers/README.md) 说明控制路径的选择；具体符号与实现说明以包内源码的 Doxygen 为准，不在根目录维护另一套 API 手册。

## 跨包控制模型

- 驱动负责协议收发、反馈与状态；控制器负责由参考值和反馈计算输出，并选用电机支持的输入能力。
- 统一接口按反馈、控制输入和控制权区分能力。支持电流 / 力矩、内部速度、内部位置或 MIT 五元组，不意味着每个电机同时支持所有能力。
- 控制器可在 MCU 外部闭环，也可把参考值交给驱动器内部闭环。更换驱动仍须核对能力、单位、减速比、零点和限幅，不能仅替换类型名。
- 反馈角度通常是输出轴 `deg`，速度为 `rpm`；`setCurrent()` 不统一等于安培，MIT 速度项也不等于普通速度接口的 `rpm`。完整约定见 [Core](core/README.md#单位与能力边界) 及各电机包。
- 两个通用控制器不会直接发送 MIT 五元组；`InternalMIT` 走外部 PID。需要刚度、阻尼与前馈的 MIT 控制，应直接使用相应驱动能力，并由宿主协调控制权。
- 连接检测由各驱动的 Watchdog 维护：成功解析反馈后喂狗，`isConnected()` 读取其状态；连接在线不等同于电机已经使能或没有故障。
- `IMotor` 是 `core::control::IController<0>` 叶节点；`controllers::IController` 是以电机为唯一候选子节点的 `IController<1>`。构造只借用对象，成功使能后才建立活动控制关系。
- 根节点 `enable()` 成功后处于 `Protected`；独立运行须检查 `standalone()` 成功后再设目标。`Core::IController` 不创建任务，也不调度 PID 或协议周期发送。

## CAN 接入方式

四个电机包都依赖 `bsp::CANDriver2`。`Config::can` 借用 `bsp::can::Can&`，不接收 HAL 句柄。每个电机在构造时通过 `Can::register_callback` 注册成员回调，在 ISR 中按协议与 ID 解码；宿主不另建电机反馈映射表或手工 HAL FIFO 分发入口。

### 宿主配置与资源归属

宿主通过 CubeMX 完成 HAL1 外设静态配置，包括时序、IRQ、FDCAN 接收资源与过滤器容量；根据电调协议选择 Classic CAN 或 FD+BRS。这里不提供固定外设实例、引脚、板卡配置或绕开 CubeMX 的 HAL 初始化示例。

宿主拥有 HAL 句柄、`Can`、过滤器索引和全局接收策略。同一外设不能同时绑定另一个 `Can` 或旧 CAN 驱动。FDCAN 的标准 / 扩展过滤器 element 下标与 bxCAN 的 filter bank 不能直接互换；bxCAN 双外设共享的 bank 分界也必须由宿主按 CANDriver2 契约统筹。MI 与 VESC 都接收扩展帧，可共享放行扩展帧的过滤器；具体条件见包 README。

**现有行为边界：** 各包的 `CAN_FilterInit` 会通过 `Can::configure_filter` 实际配置硬件过滤器，并非只做校验；`Can::start()` 是显式运行期启动操作。当前过滤器配置行为与“CubeMX 配置、库内只校验静态配置”的维护规则仍有差异，不能宣称现有实现已完整满足该规则，也不应据此绕开 CubeMX 手动重复初始化外设。若宿主的 CubeMX 配置无法被现有库校验，或需要调整上述边界，须先通过 Ask 确认。

### 初始化与寿命

1. 宿主完成 CubeMX 生成的 HAL 初始化，再构造并持有 `Can`；不要依赖跨文件全局对象的动态初始化顺序。
2. 把尚未启动的 `Can&` 借给所有电机，完成电机和控制器构造。控制器不拥有电机，电机不拥有总线，`Can` 不拥有 HAL 句柄。
3. 由宿主协调协议接收条件、过滤器资源及现有过滤器接口的使用，检查返回的 `bsp::can::Status`。bxCAN 双总线共享过滤器时，两路配置须在任一路启动前完成。
4. 检查 `Can::start()` 成功后，才使能节点或发送命令。独立控制器按 `enable()` → `standalone()` → `setRef()` 激活，每次生命周期调用均检查返回值；固定周期调用控制器更新，并执行协议额外要求的聚合发送或保活。

电机、`Can` 和 HAL 句柄必须存活到总线不再使用。已注册电机不可复制、移动或在总线仍使用时销毁；没有接收回调析构解绑接口，启动后不能追加电机。控制器和电机节点都必须在析构前成功失能，控制器必须先于借用的电机销毁；析构不再自动释放控制权或停机。

启动后注册或回调容量不足会触发 `Error_Handler()`。每个电机占用一个回调槽，所有协议与应用回调共享每条总线的 `CAN_DRIVER_MAX_CALLBACKS`，默认 `14`。同一总线内协议 ID 的唯一性由宿主保证；DJI 另会检查重复 `id1`，其发送槽位仅用于组包，不用于反馈分发。

### 周期调用与发送结果

控制周期、PID、分频、方向、减速比、协议量程和安全限幅由宿主按设备配置决定，不提供某消费项目的默认调参。控制器 `setRef()` 仅在 `Standalone` 接受；`setRefFrom(source, ...)` 仅在 `Controlled` 且显式来源等于当前父节点时接受。两个入口均返回 `bool`：`false` 不修改参考、限幅、前馈、分频或输出，`true` 只表示参考已接受，不表示发送受理或硬件执行。控制器使能时持有电机的关系由 Core 保证，不重复检查。

显式来源由调用方声明，不是强身份认证，持有父节点指针的代码仍可冒充。电机公开 `setCurrent/setInternalVelocity/setInternalPosition/setInternalMIT` 仍只按 `Standalone/Controlled` 门控，不验证来源；`getMotor()` 和原始电机接口仍可绕过控制器参考保护。本次只保护两个通用控制器的参考入口，不扩展为全设备写权限隔离。

`Protected` 拒绝新目标，但控制器 `update()` 仍运行保护参考：速度归零，位置保持保护瞬间的反馈角度。位置保持依赖有效反馈；零电流或零力矩不等于机械锁定。DJI 的 `setCurrent()` 以及保护/失能钩子只更新或清除缓存，不发送帧；用户层负责周期调用 `SendIqCommand`，组包时 `Protected/Disabled` 槽位置零，其他活动槽位仍发送各自缓存。

`bsp::can::SendResult` 的 `Submitted`、`Queued`、`ReplacedOldest` 表示受理，不保证物理发送或电机执行；软件队列还可能替换旧帧。普通业务接口保持尽力而为、不重试。MI、VESC 的保护/失能发送被拒时调用既有 `Error_Handler()`；DM 本轮保留原有乐观使能与发送结果语义。DJI 生命周期不发帧，用户必须继续发送电流组指令才能把零输出送到电调，并自行处理 `SendIqCommand` 返回值。宿主仍负责总线故障、排队旧指令和物理安全联锁，不能把 Core 状态当成硬件执行确认。

生命周期锁不保护参考值、PID、反馈或发包。宿主必须串行化生命周期、业务命令、控制周期及相关 ISR 操作；状态快照不是业务同步手段。钩子中不得启动总线、注册回调、阻塞等待反馈或重入生命周期。

## 生命周期迁移

本次改动仅限 MotorDrivers，外部底盘、轨迹和宿主调用方没有同步迁移，也不提供旧接口兼容层：

- 删除 `tryAcquireController()`、`releaseController()`、控制器 `enabled()`；改用 Core 的控制关系和 `isEnabled()` 等状态查询。`isEnabled()` 包含保护态，不能作为允许新目标的判断。
- 旧的“先 `setRef()`，再 `enable()`”与“`enable()` 后直接设目标”须改为成功进入 `Standalone` 后设置并检查 `bool` 返回值；作为父节点的子节点时由 Core 推进为 `Controlled`，父节点改用 `child.setRefFrom(*this, value)` 并检查结果，不能再调用直接入口。
- `disable()` 默认关闭本节点及活动上级，释放的下级保持 `Protected`；无控制者的根节点用 `disableTree()` 关闭活动子树。失能也可能失败，必须处理返回值。
- 同一候选树不能从两个分支重复引用同一电机；校准控制器与正常控制器需要作为独立根交接，不能同时挂入共同根的固定候选树。
- DM 反馈状态使用 `DriverState/driverState()`；DM/MI 的协议使能/失能已移入生命周期钩子。MI 原 `disable(clear_fault)` 的故障清除改用仅在 `Disabled` 可调用的 `clearFault()`，`disable(bool)` 的参数现在是 Core 的传播方向。
- 自定义 `IMotor` 派生类须将公开 `set*` 重写迁到 protected `apply*`，并实现 `selfProtect()/selfDisable()`；公开命令门控和控制权由基类提供。

## 阅读顺序

先读 [Core](core/README.md) 的抽象与单位，再读 [controllers](controllers/README.md) 选择控制路径，最后到实际电机包核对协议、设备模式、单位与资源限制。完整包入口见上表；具体 API 请沿包文档进入已有源码 Doxygen。
