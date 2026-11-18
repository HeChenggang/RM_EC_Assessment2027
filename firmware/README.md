# 双板固件工程

底盘和云台均使用 RoboMaster C 板（STM32F407IGH6），各有 CubeMX .ioc、启动文件和链接脚本。两板共享 modules 下的控制代码及底盘工程所带 HAL/CMSIS。上板外设和诊断见[云台固件](gimbal/README.md)。

| 板卡 | 板间通信 | 本地接口 |
| --- | --- | --- |
| 底盘 | CAN1，PD0 RX / PD1 TX，1 Mbit/s | CAN2，PB5 RX / PB6 TX，四个 C620。USART3 RX PC11，DR16 |
| 云台 | CAN1 同时连接 Yaw 和下板 | CAN2 接 Pitch。SPI1 接 BMI088，数据就绪走 EXTI |

## 构建

将 CMake、Ninja、Arm GNU 工具链加入 PATH。在仓库根目录执行：

```powershell
cmake -S firmware/chassis --preset Debug
cmake --build firmware/chassis/build/Debug
cmake -S firmware/gimbal --preset Debug
cmake --build firmware/gimbal/build/Debug
```

将命令中的 Debug 改为 Release 可构建发布配置。2026-11-18 用 Arm GNU 14.3.1 对 `53a0bce` 的两板 Debug / Release 配置重新构建，四个组合均通过。

- 底盘的 project.hex / project.bin 在对应配置构建目录。project.elf 由当前 CMake 固定写入 firmware/chassis/build，两种配置共用路径，后一次链接会覆盖前一次。需要保留匹配的调试符号时，在切配置前保存对应 ELF。
- 云台的 gimbal.elf / gimbal.hex / gimbal.bin 都在对应配置构建目录。

下载时核对板卡、构建配置和对应产物。历史调试用 SWD，下载 BIN 的 Flash 起址为 0x08000000。功能与安全验收结果见[验收清单](../docs/acceptance.md)。
