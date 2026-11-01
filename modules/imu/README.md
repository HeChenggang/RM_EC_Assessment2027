# C 板 BMI088 外设配置

本次配置的是标准 RoboMaster C 板的 BMI088 六轴 IMU，已同步修改
`firmware/chassis/chassis.ioc` 及对应 HAL 初始化代码。
这是 MCU 外设配置，尚不包含 BMI088 寄存器驱动、数据采集、标定或姿态解算。

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
没有开启 SPI DMA；后续先完成短报文轮询读取，再按采样频率需要增加 DMA。
SYS 保持 Serial Wire 调试，避免 JTAG 占用 PB3/PB4。

EXTI4、EXTI9_5 中断抢占优先级均为 3，低于遥控器、CAN 和 TIM6。
中断入口已调用 `HAL_GPIO_EXTI_IRQHandler()` 清除挂起标志；当前使用 HAL
默认空回调。后续通过 `HAL_GPIO_EXTI_Callback()` 设置采样标志，主循环或任务
读取 SPI，不在 EXTI 中执行延时、阻塞传输或姿态计算。EXTI9_5 为共享向量，
若以后启用其他 EXTI6..9，需在该入口增加对应分发。

TIM10 时钟为 168 MHz（APB2 分频后定时器时钟倍频），PSC=167、ARR=999：

```text
PWM frequency = 168000000 / (168 * 1000) = 1000 Hz
```

主程序尚未启动 TIM10 PWM，初始比较值为零。后续只有在温度读数可信、
温控限幅和超温/失联关闭逻辑齐备后才启动加热；不能直接输出固定满占空比。

## 后续驱动需要完成的工作

1. 分别操作两个片选，初始化 BMI088 的加速度计和陀螺仪并核对芯片 ID。
2. 按数据手册处理上电、软复位等待及加速度计 SPI 接口切换和读操作 dummy byte。
3. 设置量程、输出数据率、滤波器，配置传感器 INT 为高有效推挽，映射数据就绪。
   仅配置 STM32 EXTI 不会让传感器自动输出中断。
4. 读取加速度、角速度和温度，转换物理单位，检查通信错误与数据新鲜度。
5. 按实际板子安装方向设置轴映射；静止条件下标定陀螺仪零偏，再进行姿态融合。

BMI088 本身是六轴器件，不能提供绝对航向，纯六轴解算的 yaw 会漂移。
本次没有配置 C 板另一个 IST8310 磁力计。
现有遥控器合法帧喂狗保持不变，后续 IMU 初始化不应绕过这项保护。

## 验证状态

Debug 固件编译通过；未连接硬件验证芯片 ID、SPI 波形、中断频率或温度读数。
`.ioc` 为文本同步修改，未通过 CubeMX GUI 重新生成验证；打开后可按上表复核。

参考：[RoboMaster C 板用户手册](https://rm-static.djicdn.com/tem/35228/RoboMaster%20%20%E5%BC%80%E5%8F%91%E6%9D%BF%20C%20%E5%9E%8B%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf)、
[Bosch BMI088 数据手册](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)。
