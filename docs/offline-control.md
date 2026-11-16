# 无硬件控制软件与验证

更新日期：2026-11-16。

已完成无需设备输入的控制核心、协议编解码和仿真测试。源码统一由
`modules/CMakeLists.txt` 的 `robot_logic` 静态库编译，两套固件都能引用。
两板 main 已接入板间 CAN 收发与底盘速度闭环：上板每 20 ms 回传云台关节角，
下板据此获得相对 yaw 并接入分离/跟随。构建与离线测试通过，但实机总线收发、
云台零位标定和运动仍待验收；未调用的函数可能被链接器移除。

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
  代替。当前模型要求限位严格位于 (-π/2,π/2)，不是通用机械臂逆运动学。
- 世界系 Pitch 限位采用本周期世界 Pitch 增量等于关节 Pitch 增量的轴对齐模型；底座倾斜或
  关节耦合时必须增加运动学适配。接近边界时仅禁止向外力矩，不保证实际制动距离。
- 姿态输出为右手 ZYX Euler 角。x 前、z 上时，数学正 Pitch 使 x 朝 -z 倾斜；若机械定义
  抬枪为正，需要统一适配角度、速率和力矩符号。身体轴 gyro Y/Z 不等于任意姿态下的 Euler 速率。

## 姿态解算

独立实现四元数运动学积分加重力方向比例修正。方法背景参考
[x-io IMU/AHRS 资料](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/)，未复制其源码。
没有加入磁力计或新的零偏积分估计，复用已有 BMI088 标定模块。

安装矩阵必须正交且行列式为 +1。首次有效静止重力样本建立 roll/pitch，yaw=0；
调用方须先完成零偏标定。加速度模长偏离配置重力值时，已初始化实例用陀螺仪传播，
未初始化则拒绝。NaN/Inf、超量程、非法 dt、接近 Euler Pitch 奇异点均使姿态失效。
重新对齐后需重新使能，不能继续旧控制输出。六轴 yaw 没有绝对航向约束，会漂移。

## 安全状态机与实机接入边界

Safety_Step 要求有效遥控、链路/传感器新鲜、电机健康、硬件参数确认、校准成功。
任何停止/故障/重新校准后，先收到**解除使能且四轴回中**，再收到**使能且仍回中**，
才允许运行。急停/掉线立即清零软件输出并复位 PID，不经过速度斜坡。

Robot_Step 是离线整机参考协调器。实际上下板各自拥有不同传感器和电机，本地函数返回停止
不代表另一块板收到停止；双方均需实现本地超时、驱动发送状态检查和停止命令重试。

后续任务接入顺序：

1. DR16_GetState 快照→RemoteInput→Remote_Map；通道和拨杆配置由实机核验。
2. 完整消息编解码→LinkReceiver；Link_GetCommand 在过期时输出无效急停命令。
3. BMI088_StartGyroCalibration/Service→单位/安装轴转换→Attitude_Update。
4. 反馈时间戳、电机状态、标定状态→SafetyInputs；真实参数未确认时
   `hardware_configured` 保持 false。测试内的示例参数只用于仿真。
5. 固定 dt 调用控制器；失败清零软件输出，再由驱动适配层发送零电流/失能并检查状态和重试。
6. 接入 CAN1 收发调度和合法命令解包处喂狗。具体协议见
   [板间协议](../modules/communication/README.md)。反馈、重复/错误包不作为喂狗事件。

上板原有约 802 ms 看门狗复位仍保留。板间命令帧、状态帧与 CAN1 滤波/中断已在两板
main 中开通；`GIMBAL_YAW_ZERO` 与 `GIMBAL_YAW_ZERO_CONFIRMED` 必须由实机测得后填写，
未确认时下板分离/跟随会保持停机。

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

Arm GNU 编译真实源码后在 Unicorn Cortex-M4 执行。新增 --logic、--link、--attitude；
原有六组驱动/算法测试保留。libm 错误处理依赖通过 libnosys 链接，工具链可能提示文件
读写系统调用未实现；测试不使用这些系统调用。PASS 必须由主函数实际返回 0，超出预算会失败。

当前十一组测试和两板 Debug/Release 构建通过。机械参数、安装方向、实际总线收发、
云台零位标定、PID 调参、断链停机、看门狗复位与功能演示仍需实机验收。
