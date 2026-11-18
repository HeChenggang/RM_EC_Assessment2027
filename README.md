# Hello World 电控组 2027 考核项目

英雄机器人麦轮底盘和双轴云台控制。底盘与云台各用一块 RoboMaster C 板，底盘接收 DT7 / DR16 指令，两板通过 CAN1 双向通信。

全部考核项目已通过验收。底盘分离、跟随、小陀螺和双轴云台控制均已实现。2026-11-18 基于 `53a0bce` 验证，11 组 Cortex-M4 仿真测试和两板 Debug / Release 构建均通过。各项结果见[验收清单](docs/acceptance.md)。

| 模式 | 底盘 | 云台 |
| --- | --- | --- |
| 分离 | 按云台朝向平移，旋转目标为零 | Yaw、Pitch 采用关节角反馈 |
| 跟随 | 沿最短角差对齐云台，接近目标时提前减速 | Yaw 保持 IMU 世界航向，Pitch 采用关节角反馈 |
| 小陀螺 | 以 3.00 rad/s 为自转目标，平移使用剩余轮速余量 | Yaw 保持 IMU 世界航向，Pitch 采用关节角反馈 |

右摇杆可在各档调整 Yaw 和 Pitch。Yaw 支持跨 ±π 连续控制，Pitch 按实测反馈限位。两板执行急停、断链停机及回中重新使能。

| 文档 | 内容 |
| --- | --- |
| [验收清单](docs/acceptance.md) | 考核条款、实现及实机验收记录 |
| [配置说明](config/README.md) | 接线、实车参数、遥控器操作 |
| [双板固件](firmware/README.md) | 工具链、构建产物和板级入口 |
| [云台固件](firmware/gimbal/README.md) | 外设、控制流程和故障诊断 |
| [离线验证](docs/offline-control.md) | 测试命令、覆盖范围和固件接入 |
| [跟随模式](docs/follow-mode.md) / [小陀螺模式](docs/spin-mode.md) | 模式行为和实机核验 |
| [调试记录](docs/debug-log.md) | 历史上板、参数调整和当前复核 |
| [参考资料](docs/references/README.md) | 考核原文和本地设备手册 |

左拨杆上／中／下分别选择分离、跟随、小陀螺。右拨杆中位待机，下位急停。使能须先在中位让四轴摇杆回中，再拨到上位。
