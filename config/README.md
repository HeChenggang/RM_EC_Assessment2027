# 板级配置与遥控映射

## 硬件与当前参数

| 部件 | 当前配置 | 源码 |
| --- | --- | --- |
| 两块 RoboMaster C 板 | STM32F407IGH6，CAN1 互连，1 Mbit/s | [固件工程](../firmware/README.md) |
| 麦轮底盘 | 左右轮距 0.45 m，前后轮距 0.55 m，轮半径 0.07786 m，减速比 14:1，45° X 型模型 | [mecanum.c](../modules/chassis/mecanum.c) |
| M3508 / C620 | 下板 CAN2，左前／左后／右后／右前 ID 1／2／3／4，方向 +1／+1／−1／−1 | [chassis_setup.h](../firmware/chassis/Core/Inc/chassis_setup.h) |
| 底盘速度环 | 3000 rpm，原始电流上限 ±4000，Kp=4、Ki=3、Kd=0，积分贡献 ±650 | 同上 |
| 底盘指令 | 平移上限 1.80 m/s，加速度 1.20 m/s²，松杆立即给零平移目标。跟随 Kp=5.5，角加速度 4.00 rad/s²，按偏角提前减速。跟随转速上限和小陀螺设定均为 3.00 rad/s | 同上 |
| Yaw DM-J4310 | 上板 CAN1，TX 0x03、RX 0x13，方向 +1，零位 −0.697036878 rad，输出上限 7.00 N·m（峰值，额定 3.00 N·m），速度环 Kp=1.5、Ki=3.0，速度参考上限 2.00 rad/s | [gimbal_board.h](gimbal_board.h)、[gimbal_setup.h](gimbal_setup.h) |
| Pitch DM-J4340 | 上板 CAN2，TX 0x09、RX 0x19，方向 −1，原始零位 −0.184 rad。原始边界 0.074 和 −1.143 rad，对应 −0.258 到 +0.959 rad。速度参考上限 2.00 rad/s（与 Yaw 同级，`GIMBAL_PITCH_MAX_RATE`），静止保持刚度 150 N·m/rad（`GIMBAL_PITCH_HOLD_KP`），输出上限 28 N·m | 同上 |
| BMI088 | 上板 SPI1，安装变换为单位矩阵，固定零偏 0 °/s。代码记录俯视逆时针对应正 Yaw，静态漂移约 +0.19°/10.1 s | 同上 |

两种右后轮专用测试和 Pitch 单轴测试均关闭。双轴、Yaw 协议、Yaw 零位及 IMU 安装确认位均已置 1。更换装配后应复核轮向、Yaw 对底盘前向的零位及 IMU 方向。工作上限与现有确认位仅描述当前版本，实机验收状态见[验收清单](../docs/acceptance.md)。

## DT7 操作

| 控件 | 通道或字段 | 映射 |
| --- | --- | --- |
| 左杆纵向／横向 | ch3／ch2 | 前推为正前进，向左为正左移 |
| 右杆横向／纵向 | ch0／ch1 | 向左为正 Yaw，前推为正 Pitch |
| 左拨杆 | s1  上 1／中 3／下 2 | 分离／跟随／小陀螺 |
| 右拨杆 | s2  上 1／中 3／下 2 | 请求使能／待机／急停 |

DR16 摇杆原始范围为 ±660，死区 5%。右拨杆先置中，保持四轴回中后拨上。故障后需要再次执行该过程。下板 USART3 RX 接收 DBUS，详见[遥控器驱动](../modules/remote/README.md)。

底盘三个档都使用上板的有效相对 Yaw，板间状态有效期 100 ms，单周期回绕角差超过 0.3 rad 时拒绝。跟随和小陀螺还要求世界航向未达到 30 ms 的过期阈值。可观察 remote_command、chassis_safety_state、chassis_wheel_fresh_mask、board_status_count、board_relative_yaw 和上板 gimbal_drive.last_fault。
