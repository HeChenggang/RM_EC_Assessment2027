# C 板 BMI088 外设与驱动

标准 RoboMaster C 板的 BMI088 六轴 IMU。MCU 外设配置（CubeMX/HAL）与寄存器级
SPI 驱动都已就位，并配有可在无硬件条件下运行的主机测试。
**尚未在真实硬件上验证**，也还没有姿态解算。

## CubeMX 参数

| 功能 | 引脚/外设 | 设置 |
| --- | --- | --- |
| SPI 时钟 | PB3 / SPI1_SCK | Full Duplex Master |
| SPI 输入 | PB4 / SPI1_MISO | AF5，无上下拉 |
| SPI 输出 | PA7 / SPI1_MOSI | AF5，无上下拉 |
| 加速度计片选 | PA4 / IMU_ACCEL_CS | 推挽输出，初始高电平 |
| 陀螺仪片选 | PB0 / IMU_GYRO_CS | 推挽输出，初始高电平 |
| 加速度计数据就绪 | PC4 / IMU_ACCEL_INT | EXTI4，上升沿，无上下拉 |
| 陀螺仪数据就绪 | PC5 / IMU_GYRO_INT | EXTI5，上升沿，无上下拉 |
| 加热输出 | PF6 / TIM10_CH1 | PWM1，高有效，Pulse=0 |

SPI1：8 bit、MSB First、CPOL=High、CPHA=2 Edge（Mode 3）、Software NSS，
Prescaler=16。当前 PCLK2=84 MHz，SPI 时钟为 5.25 MHz，低于器件 10 MHz 上限。
没有开启 SPI DMA：单帧最长 8 字节，轮询读取的耗时远小于 1 kHz 控制周期。
SYS 保持 Serial Wire 调试，避免 JTAG 占用 PB3/PB4。

EXTI4、EXTI9_5 中断抢占优先级均为 3，低于遥控器、CAN 和 TIM6。
EXTI9_5 为共享向量，若以后启用其他 EXTI6..9，需在 `EXTI9_5_IRQHandler` 里增加分发。

TIM10 时钟为 168 MHz（APB2 分频后定时器时钟倍频），PSC=167、ARR=999：

```text
PWM frequency = 168000000 / (168 * 1000) = 1000 Hz
```

主程序尚未启动 TIM10 PWM，初始比较值为零。启动加热前必须先做完温度读数校验、
限幅和超温/失联关断。

## 驱动文件

- `modules/imu/bmi088.h`：对外接口与数据结构
- `modules/imu/bmi088.c`：SPI 帧、初始化序列、数据读取、零偏标定
- `tests/test_bmi088.c`：用寄存器模型模拟两颗 die 的主机测试

BMI088 是“一颗封装、两颗 die”：加速度计和陀螺仪各有独立片选、独立寄存器空间，
而且 **SPI 读帧格式不同**——加速度计在地址字节后多一个 dummy 字节，陀螺仪没有。
驱动用 `BMI088_Target.dummy_bytes` 区分，两处写序列也因此不能共用。

## 初始化序列

`BMI088_Init(&hspi1)` 依次写入下列寄存器，任何一步 SPI 失败或芯片 ID 不符都返回
`HAL_ERROR`：

| die | 寄存器 | 值 | 含义 |
| --- | --- | --- | --- |
| 加速度计 | 0x7F 读 | — | 上电为 I2C 模式，一次带片选的读操作才切到 SPI |
| 加速度计 | 0x7E | 0xB6 | 软复位，等 2 ms（手册 ≥1 ms） |
| 加速度计 | 0x00 | 读回 0x1E | 芯片 ID |
| 加速度计 | 0x7D | 0x04 | 加速度计上电 |
| 加速度计 | 0x7C | 0x00 | 退出 suspend |
| 加速度计 | 0x40 | 0xA8 | 800 Hz、normal 带宽（bit7 为手册要求必须置位） |
| 加速度计 | 0x41 | 0x01 | ±6 g |
| 加速度计 | 0x53/0x54 | 0x0A | INT1/INT2：输出使能、推挽、高有效 |
| 加速度计 | 0x58 | 0x04 | 数据就绪映射到 INT1 |
| 陀螺仪 | 0x14 | 0xB6 | 软复位，等 30 ms |
| 陀螺仪 | 0x00 | 读回 0x0F | 芯片 ID |
| 陀螺仪 | 0x0F | 0x00 | ±2000 dps |
| 陀螺仪 | 0x10 | 0x01 | 1000 Hz 输出率 / 230 Hz 滤波 |
| 陀螺仪 | 0x11 | 0x00 | normal 模式（非 suspend/LPM） |
| 陀螺仪 | 0x15 | 0x80 | 使能数据就绪中断 |
| 陀螺仪 | 0x16 | 0x00 | INT3 推挽、高有效 |
| 陀螺仪 | 0x18 | 0x01 | 数据就绪映射到 INT3 |

整个初始化约 35 ms 的阻塞延时，必须在主循环之前调用；IWDG 约 802 ms，余量充足。

## 数据通路

```text
EXTI4/EXTI9_5 → HAL_GPIO_EXTI_Callback → BMI088_ExtiCallback（只置位标志）
主循环 → BMI088_Service（按标志读 SPI、换算、更新快照）→ BMI088_GetSample
```

中断里不做 SPI 传输、不做浮点运算，避免和数据就绪中断抢总线，
也让 1 kHz 控制周期可以对 SPI 访问排他。`BMI088_Service` 的返回值只在**收到新的
陀螺仪样本**时为 true，可作为控制周期的触发条件；只更新了加速度计时返回 false。

单位换算：

```text
accel [m/s^2] = raw * (6 g * 9.80665 / 32768)
gyro  [dps]   = raw * (2000 / 32768)
temp  [degC]  = raw11 * 0.125 + 23
```

## 陀螺仪零偏标定

驱动支持 BMI088_StartGyroCalibration 启动非阻塞标定，默认窗口 1000 个新陀螺仪样本。采样期间须保持静止。通过 BMI088_GetGyroCalibration 读取状态。

- 标定成功时，用窗口的三轴均值作为零偏，后续样本自动扣除。失败保留旧零偏。
- drift_dps 为前后半窗均值差的三轴最大值，上限 1 °/s。该指标能发现窗口内的角速度变化。整窗恒定转动会进入估计的零偏。
- spread_dps 为三轴最大峰峰值，上限 20 °/s。window_ms 供检查采样耗时及主循环是否漏采。
- 历史静止记录出现 4.46 °/s 的峰峰值。此前 2 和 4 °/s 的峰峰值门槛都曾拒绝静止窗口，详见[调试记录](../../docs/debug-log.md)。

当前双板主程序均未启动该标定 API。上板世界航向联调使用固定零偏 0 °/s，分离档使用关节反馈。考核要求启动阶段仅发送零指令或保持失能，现有启动门控缺口见[验收清单](../../docs/acceptance.md)。

## 接口速查

| 函数 | 用途 |
| --- | --- |
| `HAL_StatusTypeDef BMI088_Init(SPI_HandleTypeDef *hspi)` | 校验是 SPI1、跑完初始化序列；可重复调用重试 |
| `void BMI088_ExtiCallback(uint16_t gpio_pin)` | 由 `HAL_GPIO_EXTI_Callback` 转发 |
| `bool BMI088_Service(void)` | 主循环里读数据，有新陀螺仪样本时为 true |
| `bool BMI088_GetSample(BMI088_Sample *out)` | 快照；返回 false 表示数据过期或未初始化 |
| `bool BMI088_IsOnline(uint32_t timeout_ms)` | 数据就绪是否在超时内 |
| `bool BMI088_StartGyroCalibration(uint16_t samples)` | 启动零偏窗口，不阻塞 |
| `void BMI088_GetGyroCalibration(BMI088_GyroCalibration *out)` | 标定状态快照 |

## 测试

```bash
python tests/run_motor_tests.py --imu
```

测试在 Unicorn Cortex-M4 上跑真实驱动源码，用寄存器模型模拟两颗 die：按片选决定
哪颗芯片应答、加速度计读操作额外吞掉一个 dummy 字节、寄存器自动递增。覆盖：

- 参数与状态拒绝（空指针、非 SPI1、未初始化就取数）
- 芯片 ID 错时必须初始化失败，且失败后可以重试
- 上表所有初始化写序列的字面值
- 加速度、角速度、温度的换算（16384 LSB = 3 g、3276 LSB ≈ 200 dps、0x0150 → 24.25 ℃）
- 标定：静止窗口接受并扣除、运动窗口拒绝且保留旧零偏
- SPI 错误计数、样本过期判定

## 当前固件与硬件记录

上板用 Attitude_Update 计算姿态，采用单位安装矩阵，俯视逆时针已记录为正 Yaw。代码记录静态航向约 +0.19°/10.1 s。六轴融合没有绝对航向参考，运行中仍会漂移。TIM10 加热 PWM 未启动，温度换算尚未做实物比对。

参考：[RoboMaster C 板用户手册](https://rm-static.djicdn.com/tem/35228/RoboMaster%20%20%E5%BC%80%E5%8F%91%E6%9D%BF%20C%20%E5%9E%8B%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf)、
[Bosch BMI088 数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)。
