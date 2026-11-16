# 云台 C 板外设固件

根据本地考核说明第 3 页电气拓扑建立上板工程。MCU 为 STM32F407IGH6，
沿用底盘 C 板的 12 MHz HSE、168 MHz SYSCLK、42 MHz APB1、84 MHz APB2。
CubeMX 配置版本为 6.18.1，HAL/CMSIS 复用仓库 `firmware/chassis/Drivers` 中的
STM32CubeF4 V1.28.3；云台 Core、启动文件、链接脚本与构建目录独立。

## 外设与接线

| 外设 | 引脚/配置 | 用途 |
| --- | --- | --- |
| CAN1 | PD0 RX / PD1 TX；1 Mbit/s；RX FIFO0 IRQ | Yaw DM-J4310、拨弹电机及上下板通信 |
| CAN2 | PB5 RX / PB6 TX；1 Mbit/s；RX FIFO1 IRQ | Pitch DM-J4340 与摩擦轮 |
| USART1 | PA9 TX / PB7 RX；256000、8N1；TX DMA2 Stream7；全局 IRQ | 预留 NX 视觉接口，波特率沿用已有工程，待对端确认 |
| SPI1 | PB3 SCK / PB4 MISO / PA7 MOSI；Mode 3；5.25 MHz | BMI088 |
| GPIO/EXTI | PA4 加速度 CS、PB0 陀螺 CS；PC4/PC5 数据就绪 | 复用 BMI088 驱动 |
| TIM6 | PSC=83、ARR=999，1 kHz | 控制节拍预留，尚未启动 |
| TIM10 CH1 | PF6；PSC=167、ARR=999，1 kHz | IMU 加热 PWM 预留，尚未启动 |
| IWDG | LSI；Prescaler=64、Reload=400 | 标称约 802 ms 超时复位，实际受 LSI 误差影响 |
| SWD | PA13 / PA14 | ST-Link 调试下载 |

CAN 位时序：42 MHz / (3 × (1 + 6 + 7)) = 1 Mbit/s。
电机映射集中在 `config/gimbal_board.h`：Yaw TX_ID=3、RX_ID=0x13，
Pitch TX_ID=9、RX_ID=0x19，ID 命名遵循考核文档。Pitch 组件暂选 MIT 模式，发送控制前必须核对实机模式。
遥控器在底盘板 USART3，上板不直接初始化 DR16。模板保留 USART3/USART6 的
生成配置与 IRQ，主程序不初始化或启动这两个串口。TIM4 同样只初始化、不输出 PWM。

## 构建与下载

需要 PATH 中有 CMake、Ninja 和 Arm GNU 工具链。在仓库根目录运行：

```powershell
cmake -S firmware/gimbal --preset Debug
cmake --build firmware/gimbal/build/Debug
cmake -S firmware/gimbal --preset Release
cmake --build firmware/gimbal/build/Release
```

输出位于 `firmware/gimbal/build/Debug` 或 `Release`，名称为
`gimbal.elf`、`gimbal.hex`、`gimbal.bin`。VS Code 可将 CMake sourceDirectory
切换到 `${workspaceFolder}/firmware/gimbal`。默认仍保留底盘工程入口。
确认连接的是上板后，可通过 ST-Link/SWD 用 STM32CubeProgrammer 下载 HEX，
或将 BIN 写入 0x08000000。本次未执行烧录。

## 当前运行行为与后续工作

主循环初始化并采集 BMI088，同时通过 Pitch 组件启动 CAN2 的 FIFO1 反馈接收。
调试器可观察 `imu_init_status`、`imu_online`、`imu_sample`、`pitch_init_status`、
`pitch_feedback` 和 `pitch_online`（100 ms 超时）。CAN1 FIFO0 接收 Yaw 的标准反馈帧
`0x13`，可观察 `yaw_init_status`、`yaw_online`、`yaw_rx_count` 和 `yaw_position_raw`。
Yaw 量程和机械限位未确认时保持运动保护，详见下文“遥控双轴控制”。
主循环发送失能重试；参数确认且遥控回中使能后才进入闭环。不启动加热。
DM 电机可能需要请求才返回反馈，未收到反馈并不单独证明接线错误。
没有沿用底盘的 M3508 零电流广播，以免错误作用于上板摩擦轮。

已接入底盘 CAN1 命令发送和云台 CAN1 接收：每 10 ms 发送
0x300/0x301/0x302 三帧，校验版本、序号、CRC 和字段。组包超时 20 ms，
命令超时 100 ms。启动或超时重连必须先收到未使能命令，再重新使能。
主循环只在收到新的完整命令且 remote_valid=true 后喂狗。坏帧、重复帧、
电机反馈均不喂狗；无有效命令时仍约 802 ms 复位。失联时 board_command
清零并置 estop。电机使能、关节闭环和失能重试已接入，实机参数仍待确认。
世界系姿态控制已接上骨架：BMI088 陀螺零偏标定后由 `Attitude_Update` 解算世界
yaw，`GIMBAL_YAW_WORLD=1` 时经 `GimbalDrive_SetWorld` 作为 yaw 轴反馈；该开关默认
关闭，IMU 安装矩阵与实机一致性、零偏标定时间、世界角漂移均待实测。Pitch 实测
标定仍待完成。
视觉协议和 RX 接收方式也需随后实现。不得将任意 CAN 帧当成有效命令喂狗。

重新由 CubeMX 生成后需复核：HAL/CMSIS 共用路径、自定义顶层 CMake、
USART3/USART6 未启动的约定、共享模块链接及 USER CODE。CubeMX 生成结果
不能替代硬件接线与总线实测。

## 验证记录（2026-11-07）

Arm GNU 14.3.1 下 Debug 与 Release 均编译链接成功，生成 ELF/HEX/BIN；
`python tests/run_motor_tests.py --imu` 通过。尚未进行实机烧录、总线收发、
IMU 采样和看门狗复位验证。

## Pitch 组件（2026-11-15）

已加入基于用户提供 DM4340 驱动适配的 C/HAL 组件，详见
[Pitch 组件说明](../../modules/motor/pitch_motor.md)。控制 API 现由 GimbalDrive 调用；
电机状态反馈不会刷新 IWDG。Debug/Release 构建及 `--pitch` 主机测试通过，未实机验证。

## 板间命令验证（2026-11-15）

两板 Debug/Release 构建、板间协议测试和控制逻辑测试通过，未烧录或实机验证。
观察底盘 board_link_tx_status，云台 board_command_count、board_link_online、board_command。
两板 CAN1 均为 1 Mbit/s；确认接线、共地、终端电阻及 ID 无冲突。
发送端邮箱忙时中止旧批次，下周期发送新序号。CAN 自动 bus-off 恢复未启用，
bus-off 后需复位。首次连接和重连先关闭遥控使能拨杆。
实机应检查摇杆命令、断链 100 ms 失效、随后看门狗复位和重连行为。

## Yaw 驱动（2026-11-15）

新增 modules/motor/yaw_motor.h/.c，CAN1 命令 ID 0x03、反馈 ID 0x13。
复用主程序 FIFO0 分发，不重新启动 CAN、不覆盖板间过滤器。
未确认参数时主程序以 NULL 量程初始化，仅采集原始反馈并重试失能。
观察 yaw_feedback：received、rx_count、state、position_raw、velocity_raw、
torque_raw、mos_temperature、coil_temperature；scaled=false 时物理量字段不可用。
确认实际电机 MIT 模式和 PMAX/VMAX/TMAX 后，在初始化处传入 YawMotor_Ranges，
才启用物理量换算和 MIT 发送接口。不得套用 Pitch 量程。
驱动提供 Enable/Disable/SendMIT；上层 GimbalDrive 完成遥控使能和关节闭环。
调用方负责工作扭矩限幅、故障检查、急停重试及共享总线发送调度。
在线只表示反馈新鲜，不表示已使能或无故障。未实机验证。

## 遥控双轴控制（2026-11-15）

已接通底盘 DR16 → CAN1 0x300/0x301/0x302 → 云台 GimbalDrive →
Yaw CAN1 0x03 / Pitch CAN2 0x09。使用原有右拨杆使能，底盘也会同时使能。
源码与离线测试完成，尚未烧录本次版本或验证两电机实际运动。

当前 `config/gimbal_setup.h` 中 `GIMBAL_HARDWARE_CONFIRMED=0`，双轴模式仍受保护；
用户已要求 Pitch 单轴试车，`GIMBAL_PITCH_ONLY_TEST=1` 单独允许 Pitch 遥控使能。
此模式不发送 Yaw 使能/运动指令，也不等待 Yaw 反馈，仍在停止阶段发送 Yaw 失能。
关闭单轴测试开关后，必须完成下面全部实机参数确认才能使用双轴模式。
Pitch 已按用户实测填写：原始零位 -0.184 rad，上限 -1.143 rad，下限 0.043 rad，
方向 -1（抬头为正），换算后关节范围 [-0.227, 0.959] rad。
还需填写 Yaw 实机 PMAX/VMAX/TMAX 并核对 Yaw 方向，
确认两电机 MIT 模式、Pitch 量程与 dm4340_drv.h 一致，再置 1 并构建下载。
Pitch 关节角 = (原始反馈角 - pitch_zero) × pitch_direction，单位 rad，
端点附近 0.001 rad 内的测量误差贴合限位处理，目标限位不放宽，禁止向外力矩。
限位需留制动余量，严格位于 (-π/2, π/2)。上电不会把当前位置擅自设为机械零点。
还须核对电机自身通信超时保护；总线断开或 MCU 停机时软件不能保证失能报文送达。

配置完成后的操作：所有摇杆回中，右拨杆中位，再拨上位；右摇杆水平控制 Yaw、
竖直控制 Pitch；右拨杆中位失能，下位急停。首次联机及恢复后必须重新执行中→上。
初始关节速度上限为 Yaw 0.3 rad/s、Pitch 0.2 rad/s，当前源码力矩上限分别 0.3、2.0 N·m，
这些是待调试的低输出设置，不保证足够支撑云台负载。当前为关节角控制，无 IMU 世界系稳定。
Pitch 速度环为 Kp=2、Ki=2，积分贡献上限 1.8 N·m、总输出上限 2 N·m；
增加积分以避免纯比例控制在静止负载下只能输出约 0.4 N·m。停止时清除积分。
2 N·m 是在实测 1 N·m 无法抬头、用户确认手动顺畅但较重后准备的下一档参数，
仅完成代码和离线验证，2026-11-16 未连接板卡、未烧录，不代表已经足够支撑负载。
参数集中在 gimbal_setup.h 的 GIMBAL_PITCH_TORQUE_LIMIT / SPEED_KP / SPEED_KI /
INTEGRAL_LIMIT；不使用协议 TMAX 代替工作力矩限幅。

状态流程：STOPPED 持续失能 → PREPARING 预装零力矩 → ENABLING 发送使能并用零力矩
请求反馈 → 两轴均收到新的 state=1 反馈后 ACTIVE。使能阶段须持续回中，300 ms 内
未确认成功就停机。使能指令入邮箱不等于使能成功。闭环 200 Hz，初始目标取当前反馈角。
命令或电机反馈过期 100 ms、错误状态、超限、控制周期超过 20 ms、发送失败均停止两轴。
停止时取消待发旧命令，随后每 20 ms 重试失能；电机反馈不喂 MCU 看门狗。

调试观察：`gimbal_configured`、`gimbal_drive.state/fault/output/tx`、
`board_link_online`、`board_command`、`yaw_feedback`、`pitch_feedback`。
fault：0 正常，1 参数，2 链路/急停/使能阶段偏杆，3 反馈，4 限位/控制输入，
5 发送，6 使能超时，7 周期超时。tx 是最后一次发送结果，不代表电机执行确认。
新增 fault=8：Pitch 持续高负载无进展。输出达到工作上限 90%、目标与测量差至少
0.05 rad，且连续 1000 ms 未产生 0.005 rad 位移时，停止、清积分并重试失能；
达到位移门槛则重新计时，低负载或小误差不累计。该故障也可能意味着工作力矩不足，
并不能单独证明机械卡死。last_fault 在回中失能后保留，下一次使能尝试清除，
便于事后检查。故障后保持上位不会自动恢复，仍要求回中、中位→上位。
新增诊断字段会改变内存布局，调试必须使用匹配 ELF 的符号地址，不复用历史脚本地址。

验证：`python tests/run_motor_tests.py --all` 共 11 组通过；云台 Debug/Release 构建通过。
新增 `--gimbal` 覆盖回中使能、反馈确认、双轴输出、掉线、故障、超限、发送失败、
失能重试、禁止自动恢复、使能超时、方向适配与时钟回绕。

## C 板读取 Yaw 参数（2026-11-16）

保持遥控器开启、板间命令有效且使能拨杆关闭。主循环自动通过 CAN1 向
0x7FF 发送只读参数查询，目标电机 3，预期回复 CAN ID 0x13。
顺序读取 CTRL_MODE(10)、PMAX(21)、VMAX(22)、TMAX(23)，每项最多三次，
每次等待 100 ms。读取结束后不再发送，复位可重新读取。
Watch 添加 yaw_params.control_mode、pmax、vmax、tmax、valid_mask、finished、
timeout_count、tx_error_count。valid_mask=15 才表示四项全部有效；finished
仅表示查询结束，不能单独判定成功。MIT 模式值为 1。
参数回复独立分发，不更新 yaw_feedback 的在线时间或普通反馈计数。
结果只供核对，不自动写入电机或开启 Yaw，也不自动覆盖 gimbal_setup.h。
未收到回复时检查电机 ID、反馈 ID、供电、总线及固件协议兼容性。
协议参考 https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py 。

## Yaw 实测量程接入（2026-11-16）

用户 C 板读取结果：CTRL_MODE=1，PMAX=3.14159298 rad、VMAX=21 rad/s、
TMAX=7.5 N·m，valid_mask=15、timeout_count=0。已写入 gimbal_setup.h。
协议确认与整机运动确认分离：GIMBAL_YAW_PROTOCOL_CONFIRMED=1 使反馈正常换算，
GIMBAL_HARDWARE_CONFIRMED=0、GIMBAL_PITCH_ONLY_TEST=1 保持原有 Pitch 调试行为。
上述配置取代此前 NULL 量程接入说明，Yaw 不自动使能。

参数查询结束且遥控有效、未使能时，每 100 ms 用 0x7FF/0xCC 请求一次 Yaw
状态，不发送运动目标。观察 yaw_query_status、yaw_feedback.received/scaled、
pos、vel、torque、state、rx_count 和 yaw_online。请求协议尚待本机实测；
HAL_OK 仅代表入邮箱。正常参数回复不会计入电机运动反馈。
Yaw 发送接口已有 MIT 编码、有限数检查、量程拒绝、特殊帧冲突拒绝、邮箱忙处理、
显式使能/失能。量程不是工作限幅；现有双轴控制配置的 Yaw 输出限制为 0.3 N·m。
实机方向、转动空间/线束及失联停机验证完成后，才切换双轴配置。

## Yaw 低扭矩测试配置（2026-11-16）

用户确认逆时针转动时 pos 增大、vel 为正，并请求开始测试。
已设 GIMBAL_HARDWARE_CONFIRMED=1、GIMBAL_PITCH_ONLY_TEST=0，双轴参与控制。
Yaw 方向 +1，扭矩限幅 0.3 N·m，目标角速度/摇杆速度上限 0.3 rad/s。
这些开关表示允许本次调试，不表示已完成实机验收。Pitch 保持已有 2 N·m 配置。
先使能关闭、所有摇杆回中，再使能；首次目标跟随当前位置。
确认 gimbal_drive.state=3、fault=0、两电机反馈 state=1 后，小幅短时操作
Yaw 摇杆，再回中观察保持。关闭使能检查 gimbal_drive.state=0 和电机失能。
该固件会使能两个云台轴；底盘仍可能响应同一遥控器，测试时需固定底盘并留出
云台转动空间。Yaw 没有线束角度限位，不能持续单方向转动。
异常时立即关闭使能；记录 gimbal_drive.last_fault、tx[0]、yaw_feedback，
不要直接增大扭矩。实际烧录及运动结果待现场确认。
