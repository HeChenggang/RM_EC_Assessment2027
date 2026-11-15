# Hello World 电控组 2027 考核项目

机器人底盘、云台及遥控器控制开发项目。
已建立两板固件、驱动和无硬件控制核心，包括模式、安全状态机、闭环框架、姿态与板间协议。
离线测试和接入边界见 [无硬件控制软件与验证](docs/offline-control.md)。底盘已启用并烧录四轮低速遥控控制，运动效果尚待实机验证，当前状态见 [调试记录](docs/debug-log.md)。

## 目录

| 路径 | 用途 |
| --- | --- |
| `firmware/` | 确认开发板后导入 CubeMX / HAL / RTOS 工程 |
| `modules/chassis/` | 底盘运动学和分离、跟随、小陀螺模式 |
| `modules/gimbal/` | yaw / pitch 控制与关节限位 |
| `modules/motor/` | 电机协议、反馈及控制封装 |
| `modules/remote/` | DT7 / DR16 数据解析及拨杆映射 |
| `modules/imu/` | IMU 采集、零漂校准与姿态 |
| `modules/control/` | PID、坐标变换等通用算法 |
| `modules/safety/` | 急停、掉线处理和启动保护 |
| `config/` | 经确认的硬件映射与参数 |
| `docs/` | 需求、调试记录与验收清单 |
| `tests/` | 后续主机算法测试 |
| `tools/` | 后续构建及调试辅助工具 |

## 开发起点

1. 在 `config/README.md` 确认开发板、底盘类型、电机与通信映射。
2. 将对应固件工程放入 `firmware/`，记录工具链和构建方式。
3. 依次完成通信及安全保护、单电机、底盘、云台和模式联调。
4. 按 `docs/acceptance.md` 记录验证结果。

模块目录为组织约定；导入固件后仍需配置源码路径与头文件路径。
Git 默认分支为 `main`，提交前检查 `git diff`。
