# 云台 C 板外设固件

根据本地考核说明第 3 页电气拓扑建立上板工程。MCU 为 STM32F407IGH6，
沿用底盘 C 板的 12 MHz HSE、168 MHz SYSCLK、42 MHz APB1、84 MHz APB2。
CubeMX 配置版本为 6.18.1，HAL/CMSIS 复用仓库 `firmware/chassis/Drivers` 中的
STM32CubeF4 V1.28.3；云台 Core、启动文件、链接脚本与构建目录独立。

## 外设与接线

| 外设 | 引脚/配置 | 用途 |
| --- | --- | --- |
| CAN1 | PD0 RX / PD1 TX；1 Mbit/s；RX FIFO0 IRQ | Yaw DM-J4310 及上下板通信 |
| CAN2 | PB5 RX / PB6 TX；1 Mbit/s；RX FIFO1 IRQ | Pitch DM-J4340 |
| USART1 | PA9 TX / PB7 RX；256000、8N1；TX DMA2 Stream7；全局 IRQ | 预留 NX 视觉接口，波特率沿用已有工程，待对端确认 |
| SPI1 | PB3 SCK / PB4 MISO / PA7 MOSI；Mode 3；5.25 MHz | BMI088 |
| GPIO/EXTI | PA4 加速度 CS、PB0 陀螺 CS；PC4/PC5 数据就绪 | 复用 BMI088 驱动 |
| TIM6 | PSC=83、ARR=999，1 kHz | 控制节拍预留，尚未启动 |
| TIM10 CH1 | PF6；PSC=167、ARR=999，1 kHz | IMU 加热 PWM 预留，尚未启动 |
| IWDG | LSI；Prescaler=64、Reload=400 | 标称约 802 ms 超时复位，实际受 LSI 误差影响 |
| SWD | PA13 / PA14 | ST-Link 调试下载 |

CAN 位时序：42 MHz / (3 × (1 + 6 + 7)) = 1 Mbit/s。
电机映射集中在 `config/gimbal_board.h`：Yaw TX_ID=3、RX_ID=0x13，
Pitch TX_ID=9、RX_ID=0x19，ID 命名遵循考核文档。两轴闭环均发送 MIT 控制报文。
遥控器在底盘板 USART3，上板不直接初始化 DR16。模板保留 USART3/USART6 的
生成配置与 IRQ，主程序不初始化或启动这两个串口。TIM4 同样只初始化、不输出 PWM。


## 构建与运行

构建命令和产物见[双板固件工程](../README.md)，当前机械参数和拨杆映射见[配置说明](../../config/README.md)。CubeMX 重新生成后复核 HAL/CMSIS 路径、顶层 CMake、共享模块和 USER CODE。

上板通过 CAN1 接收下板的遥控命令，并按 20 ms 周期调度关节状态回传。命令组包限时 20 ms，100 ms 后过期。只有完整有效命令且 remote_valid 为真才在解包后登记看门狗事件，由主循环在消费新鲜命令时刷新 IWDG。状态、电机反馈及坏包不刷新。详见[板间协议](../../modules/communication/README.md)。

双轴控制约 200 Hz，STOPPED 重试失能，PREPARING 预装零力矩，ENABLING 发送使能并请求反馈。两轴收到新的 state=1 反馈后进入 ACTIVE，目标从当前角度开始，使能阶段须保持摇杆回中。失联、异常反馈、超限、无法发送和控制周期异常均走停机路径。停止时取消待发旧命令，按 20 ms 间隔重试失能。HAL_OK 表示进入邮箱，断链时能否停机仍取决于电机端通信超时保护。

分离档使用关节 Yaw 和 Pitch。跟随、小陀螺使用世界 Yaw + 关节 Pitch。BMI088 数据经安装矩阵和姿态解算后提供世界航向及角速度。联调使用固定零偏 0 °/s。世界姿态失效或达到 30 ms 未更新时，使用世界航向的档位退出使能。加热 PWM 未启动。

分离档的 Yaw 角度目标使用电机编码器。IMU 新鲜时，速度环用 IMU 航向角速度减去估算的底盘角速度，抑制电机速度反馈中的快速波动。底盘角速度由两路速度之差低通估计，时间常数 40 ms。IMU 过期时使用电机速度反馈。

Yaw 收杆时，如果实测速率仍高于减小后的杆量上限，速度环按杆量变化比例减小同向驱动积分，提前开始减速。回中时捕获当前角度作为保持目标，将速度环积分限制在 ±0.35 N·m，减小外力撤去后回正的超调。

Pitch 目标由编码器反馈的限位约束，端点禁止向外力矩。当前软件输出上限为 28 N·m，关节在限位范围内卡住时可能持续输出力矩。详见[Pitch 组件](../../modules/motor/pitch_motor.md)和[验收清单](../../docs/acceptance.md)。

## 调试观测

| 变量 | 用途 |
| --- | --- |
| board_command、board_command_count、board_link_online | 命令、累计完整消息及链路有效性 |
| watchdog_generation、watchdog_consumed | 解包事件及主循环消费代次 |
| imu_online、attitude_ready、attitude_world_yaw、attitude_world_rate | IMU 采样与世界航向反馈 |
| yaw_feedback、pitch_feedback、status_sent_count | 电机反馈和已入邮箱的状态分片数 |
| gimbal_drive.state / fault / last_fault / output / tx | 使能阶段、故障、最近故障和输出 |

故障码 0 正常，1 参数，2 链路／急停／使能阶段偏杆，3 反馈，4 限位／控制输入，5 发送，6 使能超时，7 周期超时，9 小陀螺累计时间触限，10 小陀螺转角触限。编号 8 保留给旧版固件记录。当前 9、10 对应的限制已关闭。last_fault 保留至下一次使能尝试。读取这些变量时使用与板上固件匹配的 ELF。

### Yaw 只读参数

遥控有效且未使能时，主循环通过 CAN1 以 0x7FF 查询目标电机 3 的 CTRL_MODE(10)、PMAX(21)、VMAX(22)、TMAX(23)，每项最多三次，每次等待 100 ms。回复 ID 0x13，yaw_params.valid_mask=15 表示四项齐备，finished 表示查询流程已结束。2026-11-16 记录为 MIT=1、PMAX=3.14159298 rad、VMAX=21 rad/s、TMAX=7.5 N·m。结果供核对，配置由 gimbal_setup.h 提供。参数回复不更新普通反馈计数或在线时间。

读取参数结束、链路有效且未使能时，约每 100 ms 发一次 0x7FF/0xCC 请求 Yaw 状态。观察 yaw_query_status 和 yaw_feedback。Yaw/Pitch 反馈在线仅表示数据新鲜。
