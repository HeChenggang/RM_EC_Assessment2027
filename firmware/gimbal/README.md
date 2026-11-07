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
Pitch TX_ID=9、RX_ID=0x19，ID 命名遵循考核文档；尚未绑定具体 DM 控制模式。
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

主循环只初始化并采集 BMI088，可在调试器观察 `imu_init_status`、`imu_online`、
`imu_sample`。不启动 CAN、不发电机指令、不使能电机，也不启动加热。
没有沿用底盘的 M3508 零电流广播，以免错误作用于上板摩擦轮。

**当前未实现板间协议，因此没有合法指令解包点，也不喂 IWDG；固件会约每 802 ms
复位。** 这是外设配置阶段的固件，不是可运行的云台控制程序。
IMU 零偏标定和姿态解算尚未接入；目前的复位周期也不足以完成默认 1000 样本标定。
接入控制前需定义并实现板间命令帧、完整性检查、有效帧处喂狗、超时与急停处理，
以及 DM 驱动、Yaw 连续角度、Pitch 实测限位和零位、输出限幅及启动标定状态机。
视觉协议和 RX 接收方式也需随后实现。不得将任意 CAN 帧当成有效命令喂狗。

重新由 CubeMX 生成后需复核：HAL/CMSIS 共用路径、自定义顶层 CMake、
USART3/USART6 未启动的约定、共享模块链接及 USER CODE。CubeMX 生成结果
不能替代硬件接线与总线实测。

## 验证记录（2026-11-07）

Arm GNU 14.3.1 下 Debug 与 Release 均编译链接成功，生成 ELF/HEX/BIN；
`python tests/run_motor_tests.py --imu` 通过。尚未进行实机烧录、总线收发、
IMU 采样和看门狗复位验证。
