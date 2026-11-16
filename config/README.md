# 硬件配置待确认

- 当前固件目标：RoboMaster C 板、STM32F407IGH6、Arm GNU/CMake/Ninja。云台配置与构建说明见 `firmware/gimbal/README.md`，接线尚需实机核验。
- 底盘类型：麦轮；左右轮距 0.45 m、前后轮距 0.55 m（用户于 2026-11-01 确认）。
- 按考核拓扑，上板负责云台，下板接遥控器，板间共用 CAN1；通信协议待定义。
- 电机型号、CAN 总线、ID、正方向和减速比：待确认。
- IMU 型号、安装方向及通信接口：待确认。
- 遥控器通过底盘 USART3/DMA 接收。已将实测通道及以下拨杆分配接入底盘固件。
- pitch 机械限位、yaw 零位及控制输出上限：待实测。

底盘解算采用轮中心间的完整轮距，旋转系数为 (0.45 + 0.55) / 2 = 0.50 m。
轮半径 0.07786 m、减速比 14:1 暂沿用考核文档；45° X 型麦轮布局与四轮电机正方向需实车核对。
参数集中在 `modules/chassis/mecanum.c` 的 `MECANUM_DEFAULT_CONFIG`，调用方可复制后修改。

## DT7 摇杆实测

| 摇杆轴 | DR16 通道 | 负满行程 −660 | 正满行程 +660 |
| --- | --- | --- | --- |
| 左杆纵向 | ch3 | 后 | 前 |
| 左杆横向 | ch2 | 左 | 右 |
| 右杆纵向 | ch1 | 后 | 前 |
| 右杆横向 | ch0 | 左 | 右 |

按底盘前进、左移为正的约定，`RemoteMap` 的前进轴使用 ch3、方向 `+1`，
左移轴使用 ch2、方向 `-1`。

| 实体拨杆 | DR16 字段 | 上 | 中 | 下 |
| --- | --- | --- | --- | --- |
| 左侧 | `remote_state.s1` | 1 | 3 | 2 |
| 右侧 | `remote_state.s2` | 1 | 3 | 2 |

左拨杆上、中、下分别选择分离、跟随、小陀螺。右拨杆上、中、下分别请求使能、待机、急停。
摇杆死区为 5%，右杆向左和向前分别映射为归一化 yaw、pitch 输入的正方向。
在 Ozone 观察 `remote_command`。其中 `arm` 是请求。

底盘已接入左杆前后与横移的速度闭环。云台每 20 ms 通过板间 CAN 的 0x310..0x312
回传关节角状态，底盘用其中的 Yaw 关节角进行平移坐标变换。小陀螺以 1.50 rad/s
旋转，保留平移并用 IMU 保持云台世界航向；累计时间和关节转角停机保护已关闭。见 [小陀螺模式](../docs/spin-mode.md)。

首次上车必须先标定云台零位：把云台摆到指向底盘正前方，读取
`yaw_feedback.pos * GIMBAL_YAW_DIRECTION`，写入 `config/gimbal_setup.h` 的
`GIMBAL_YAW_ZERO`，再把 `GIMBAL_YAW_ZERO_CONFIRMED` 置 1。未确认时状态帧
`calibrated=false`，底盘进入 FAULT。

云台联调暂用三轴陀螺零偏 0 °/s，由 BMI088 实测数据直接解算姿态。无磁力计，
实际零偏会使世界 yaw 随时间漂移。核对 `GIMBAL_IMU_SENSOR_TO_BODY` 的轴序和正负方向后，
通过 `GIMBAL_IMU_MOUNT_CONFIRMED` 允许世界航向反馈进入闭环。`GIMBAL_YAW_WORLD` 默认置 0，
分离模式使用关节反馈。跟随和小陀螺使用世界航向，要求 IMU 安装与 Yaw 零位已确认且姿态反馈新鲜。
姿态解算失败或世界角达到 30 ms 未更新时，使用世界航向的云台闭环会退出使能。见 [跟随模式](../docs/follow-mode.md)。
状态帧丢失超过 100 ms、CRC/序号错误，或 yaw 单周期跳变超过 0.3 rad 时，
底盘会退出使能，需要重新回中并使能。
在 `firmware/chassis/Core/Inc/chassis_setup.h` 填写实测轮向（左前、左后、右后、右前）、
转子最高转速、原始电流上限和速度环 Kp。目前启用四轮低速调试配置：
转子上限 1600 rpm、原始电流上限 ±2000、Kp=4、Ki=3、Kd=0，积分原始输出限幅 ±650。平移指令上限 0.60 m/s，起步加速度 1.20 m/s²，满杆目标约 0.5 秒达到上限，回中立即给出零平移目标。两种右后轮专用测试模式均关闭。
这些是初始调试参数，尚未完成四轮运动验证与负载调参。
四轮反馈都新鲜时，右拨杆先置中并让摇杆回中，再拨上使能。右拨杆置下、遥控掉线、
任一轮反馈过期或 CAN 发送失败会退出使能。Ozone 可观察 `chassis_configured`、
`chassis_wheel_fresh_mask`、`chassis_safety_state`、`chassis_output`、
`board_status_count` 和 `board_relative_yaw`；上板可观察 `status_sent_count`。
