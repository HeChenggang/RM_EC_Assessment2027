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
回传关节角状态，底盘用其中的 yaw 关节角作为云台相对底盘的偏角：分离与跟随按该角度
做坐标变换，小陀螺以 0.5 rad/s 旋转并保留平移（整机随转）。

小陀螺采用方案 A：整机绕自身中心旋转，云台 yaw 关节保持相对角、随底盘一起转，
所以线束不会在 yaw 轴上持续缠绕，代价是云台视角跟着转。旋转速率在
`firmware/chassis/Core/Inc/chassis_setup.h` 的 `CHASSIS_SPIN_RATE_RAD_S` 调整。
方案 B（云台世界系锁定、底盘在下面转）留待后续，需要限时/限圈或滑环；它与
`GIMBAL_YAW_WORLD` 绑定，开启前不要用方案 B 跑小陀螺，否则 yaw 关节会一直绕线。

首次上车必须先标定云台零位：把云台摆到指向底盘正前方，读取
`yaw_feedback.pos * GIMBAL_YAW_DIRECTION`，写入 `config/gimbal_setup.h` 的
`GIMBAL_YAW_ZERO`，再把 `GIMBAL_YAW_ZERO_CONFIRMED` 置 1。未确认时状态帧
`calibrated=false`，底盘在分离/跟随会进入 FAULT；小陀螺不依赖 yaw，可先单独验证。

跟随模式还要求云台 yaw 做世界系稳定，否则关节伺服会把相对角锁住，底盘会一直转到
旋转上限。P0-2 骨架已接入世界系 yaw 但默认关闭：把 `config/gimbal_setup.h` 的
`GIMBAL_YAW_WORLD` 置 1 后，yaw 轴改用 BMI088 姿态解算的世界 yaw 作反馈，底盘转向
时云台保持世界朝向，关节角随之减小，跟随环才可能收敛。上电后需静止等待陀螺零偏
标定完成；零偏未完成、姿态解算失败或世界角超过 30 ms 未更新时，云台会以
`GIMBAL_FEEDBACK` 退出使能。无磁力计，世界 yaw 会缓慢漂移，长时间运行需重新标定。
首次开启前必须核对 `GIMBAL_IMU_SENSOR_TO_BODY` 安装矩阵与实机安装方向一致。
跟随档还受 `CHASSIS_FOLLOW_IMU_READY=0` 限制，世界系调通后再置 1。
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
