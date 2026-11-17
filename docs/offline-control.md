# 无硬件控制软件与验证

更新日期为 2026-11-17，测试基线为 ebc26db。共享 C 模块由 robot_logic 静态库编译。底盘 main 接入 DR16、上下板 CAN 状态和四轮闭环。云台 main 接入 BMI088、双轴闭环和板间命令。实际控制与离线协调器的区别见下文。

## 已实现范围

| 考核内容 | 软件实现 | 离线验证 |
| --- | --- | --- |
| 遥控器指令与拨杆 | `remote/command`：通道/符号配置、连续死区、归一化、模式、急停 | 满量程、死区、非法输入、断链 |
| 启动保护与掉线停机 | `safety/safety_gate`：校准等待、解除使能、运行、故障 | 恢复后不自动运动、非中位不允许启动 |
| 云台系→底盘系 | `control/motion_math`：二维旋转 | ±90°、180°、正反变换 |
| 三种底盘模式 | `chassis/chassis_control`：分离、跟随比例环、小陀螺、加速度限制 | 模式切换、方向、旋转分量、速度斜坡 |
| 四轮速度闭环 | 麦轮解算→四路 PID→C620 原始电流限幅 | 单轮失联全部清零、积分复位、输出上限 |
| Yaw 连续旋转 | `AngleTracker`：回绕处理、连续圈数、转速/采样超时检查 | 正反跨界、多圈、时钟回绕、跳变拒绝 |
| Pitch 限位及云台控制 | `gimbal/gimbal_control`：每轴关节/世界系选择、角度环→速度 PID→力矩 | 跨界角误差、关节限位、世界系限位、失联清零 |
| IMU 姿态解算 | `imu/attitude`：安装矩阵、四元数积分、重力方向反馈、Euler 角/角速率 | 倾斜、旋转、归一化、加速干扰、异常采样 |
| 上下板协议 | `communication/board_link`：命令/状态、分片、CRC、序号、超时 | 位翻转、重复/旧包、乱序、丢片、重新同步 |
| 整体软件链路 | `control/robot_control`：安全门控后计算底盘/云台输出 | 遥控→CAN 编解码→使能→运动→掉线停止 |

模块不分配动态内存、不调用 HAL、不发送电机报文。每个实例应串行访问；
不要直接修改已初始化实例的配置/状态，重新配置通过 Init 完成。

## 坐标与控制约定

- 单位 rad、rad/s、m/s；BMI088 的 gyro_dps 必须乘 π/180。
- 底盘 x 前、y 左、z 上，正 yaw 为绕 +z 逆时针；相对 yaw 为云台相对底盘。
  `vx_c=cos(yaw)*vx_g-sin(yaw)*vy_g`，`vy_c=sin(yaw)*vx_g+cos(yaw)*vy_g`。
- 分离模式旋转目标为 0；跟随为限幅后的 `follow_kp*wrap(relative_yaw)`；小陀螺为非零
  `spin_rate`。摇杆 yaw 控制云台。模式切换受斜坡限制，反转时旋转速度可短暂经过 0。
- 平移按向量长度限幅，四轮超速按麦轮模块等比例缩放。`ChassisOutput.velocity` 是缩放前
  请求，`wheels` 是实际给速度环的目标。PID 输出是 C620 原始电流单位。
- Yaw 使用最短角误差，目标跨 ±π 不引入整圈跳变；AngleTracker 另行保留连续角。
  圈数解包要求相邻采样转角小于 π；超时/不合理跳变返回失败，控制层应停机。
- 云台角度外环输出限幅速度，内环 PID 输出力矩。Pitch 限位是实测关节值，不能使用协议量程
  代替。当前模型要求限位严格位于 (-π/2,π/2)。
- 世界系 Pitch 限位采用本周期世界 Pitch 增量等于关节 Pitch 增量的轴对齐模型；底座倾斜或
  关节耦合时必须增加运动学适配。接近边界时仅禁止向外力矩，不保证实际制动距离。
- 姿态输出为右手 ZYX Euler 角。x 前、z 上时，数学正 Pitch 使 x 朝 -z 倾斜；若机械定义
  抬枪为正，需要统一适配角度、速率和力矩符号。身体轴 gyro Y/Z 不等于任意姿态下的 Euler 速率。

## 姿态解算

独立实现四元数运动学积分加重力方向比例修正。方法背景参考
[x-io IMU/AHRS 资料](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)，未复制其源码。
没有加入磁力计或新的零偏积分估计，复用已有 BMI088 标定模块。

安装矩阵必须正交且行列式为 +1。首次有效静止重力样本建立 roll/pitch，yaw=0；
采用测量零偏时，调用方先完成标定。当前上板联调使用固定零偏 0 °/s。加速度模长偏离配置重力值时，已初始化实例用陀螺仪传播，
未初始化则拒绝。NaN/Inf、超量程、非法 dt、接近 Euler Pitch 奇异点均使姿态失效。
重新对齐后需重新使能，不能继续旧控制输出。六轴 yaw 没有绝对航向约束，会漂移。

## 固件接入与安全门控

下板每 10 ms 更新控制，要求 DR16 有效、板间状态与相对 Yaw 有效、四轮反馈新鲜。上板约 200 Hz 发送闭环，跟随和小陀螺额外要求新鲜世界 Yaw。两板均要求解除使能时四轴回中，再在回中状态重新使能。故障或急停绕过运动斜坡，并重置控制输出和 PID 状态。

Robot_Step 是用于测试的整机参考协调器。板上运行路径由两块板各自的 main 和 GimbalDrive 执行。仿真可检查软件输出，电机端实际停止、CAN 断开后失能和两板 IWDG 复位须在实机记录。当前程序未将上电零偏标定作为全电机使能门槛，见[验收清单](acceptance.md)。

## 一键验证

```powershell
python tests/run_motor_tests.py --all
cmake -S firmware/chassis --preset Debug
cmake --build firmware/chassis/build/Debug
cmake -S firmware/chassis --preset Release
cmake --build firmware/chassis/build/Release
cmake -S firmware/gimbal --preset Debug
cmake --build firmware/gimbal/build/Debug
cmake -S firmware/gimbal --preset Release
cmake --build firmware/gimbal/build/Release
```

Arm GNU 14.3.1 编译真实源码，由 Python Unicorn 2.1.4 的 Cortex-M4 仿真器运行。省略参数时仅运行 M3508 驱动测试，完整回归用 --all。PASS 表示 C 测试程序实际返回 0。工具链可提示 newlib 文件系统调用桩未实现，测试不调用这些接口。

本轮 11 组通过，覆盖 M3508、DR16、PID、BMI088、麦轮、Pitch、Yaw、云台驱动、整机逻辑、板间协议和姿态解算。两板 Debug 与 Release 均重新构建通过。各模式与硬件验证状态见[验收清单](acceptance.md)。
