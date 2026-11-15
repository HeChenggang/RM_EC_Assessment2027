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
Yaw 原始字段尚未按实机量程换算。固件不发电机指令、
不使能电机，也不启动加热。DM 电机可能需要请求才返回反馈，未收到反馈并不单独证明接线错误。
没有沿用底盘的 M3508 零电流广播，以免错误作用于上板摩擦轮。

已接入底盘 CAN1 命令发送和云台 CAN1 接收：每 10 ms 发送
0x300/0x301/0x302 三帧，校验版本、序号、CRC 和字段。组包超时 20 ms，
命令超时 100 ms。启动或超时重连必须先收到未使能命令，再重新使能。
主循环只在收到新的完整命令且 remote_valid=true 后喂狗。坏帧、重复帧、
电机反馈均不喂狗；无有效命令时仍约 802 ms 复位。失联时 board_command
清零并置 estop。电机使能、闭环和实际停机发送状态机尚未接入。
姿态解算、Yaw 驱动和 Pitch 标定仍待完成。
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
[Pitch 组件说明](../../modules/motor/pitch_motor.md)。控制 API 已提供，但主程序不调用；
电机状态反馈不会刷新 IWDG。Debug/Release 构建及 `--pitch` 主机测试通过，未实机验证。

## 板间命令验证（2026-11-15）

两板 Debug/Release 构建、板间协议测试和控制逻辑测试通过，未烧录或实机验证。
观察底盘 board_link_tx_status，云台 board_command_count、board_link_online、board_command。
两板 CAN1 均为 1 Mbit/s；确认接线、共地、终端电阻及 ID 无冲突。
发送端邮箱忙时中止旧批次，下周期发送新序号。CAN 自动 bus-off 恢复未启用，
bus-off 后需复位。首次连接和重连先关闭遥控使能拨杆。
实机应检查摇杆命令、断链 100 ms 失效、随后看门狗复位和重连行为。
