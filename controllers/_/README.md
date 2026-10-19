# MotorDrivers::Controller

本目录是控制器聚合包，包名和依赖见 [cpkg.toml](cpkg.toml)。它依赖：

- [MotorController::VelController](../vel_controller/README.md)
- [MotorController::PosController](../pos_controller/README.md)

该包没有独立的控制器实现或另一套 API；宿主需要同时使用两种控制器时可依赖此入口，只需要一种时可直接依赖相应包。它不代替电机协议包，也不自动选择电机或配置外设。

控制路径、对象寿命和启停边界见 [controllers 综述](../README.md)，全仓包导航与 CAN 接入见 [根 README](../../README.md)。
