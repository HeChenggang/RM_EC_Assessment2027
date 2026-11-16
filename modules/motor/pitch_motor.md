# DM4340 Pitch 电机组件

基于用户于 2026-11-15 提供的 `dm4340_drv.hpp`、`dm4340_drv.cpp` 适配，
不依赖原工程的 `system_user.hpp`、`HW_can.hpp`。原文件未修改。
本仓库采用 C 实现，并提供 `extern "C"` 供 C++ 调用。

- `dm4340_drv.h/.c`：MIT、位置速度、速度报文，显式使能/失能，反馈解包。
- `pitch_motor.h/.c`：绑定 CAN2、TX_ID=0x09、RX_ID=0x19，接收过滤、快照和超时。
- `config/gimbal_board.h`：板级 ID 映射。

反馈包括状态码、位置、速度、扭矩、MOS/线圈温度及接收时间和计数。
反馈必须是 0x19 的标准数据帧、长度 8，载荷中的低四位电机 ID 必须为 9。
故障状态仍保留供诊断；`PitchMotor_IsOnline()` 只说明反馈新鲜，不代表电机健康或已使能。

## 接入与调用约定

`PitchMotor_Init(&hcan2, DM4340_MIT_MODE)` 在 `MX_CAN2_Init()` 后调用，
配置 filter bank 14、16 位列表过滤、FIFO1、共享过滤器分界 14，并启动 CAN2。
FIFO1 回调转发到 `PitchMotor_RxFifo1Callback()`，主循环读取快照。
初始化失败返回 HAL 状态，不发送任何帧；初始化成功后拒绝重复初始化。
此组件独占 CAN2 的启动及 FIFO1 消费，之后接入摩擦轮需统一过滤和接收分发，
不能直接再调用底盘 `M3508_CAN_Init()`。

主程序已完成上述接入，MIT 控制由 GimbalDrive 管理；
只有 gimbal_setup.h 的实机参数确认且遥控安全条件满足后才发送使能/运动命令。
控制层可显式调用 `PitchMotor_Enable()`、`PitchMotor_Disable()`，以及匹配初始化
模式的 `PitchMotor_SendMIT()`、`PitchMotor_SendPosition()`、`PitchMotor_SendSpeed()`。
模式选择仅决定报文地址，不更改电机参数。现场模式尚未验证。

发送仅用于主循环/控制任务，调用方需串行化同一总线的发送。邮箱未全部空闲返回
`HAL_BUSY`，避免继续堆积旧指令；HAL 发送错误原样返回。`HAL_OK` 只表示进入发送邮箱，
不表示电机确认或执行。急停失能不能只发一次后忽略状态，控制层需处理发送失败、
待发旧指令与重试。本组件不提供完整急停状态机。

## 单位和限制

沿用参考文件：位置 ±3.141593 rad、速度 ±10 rad/s、扭矩 ±28 N·m，
Kp 0..500、Kd 0..5。它们是报文映射范围，必须与电机 PMAX/VMAX/TMAX 一致，
并非实测机械安全范围。位置速度模式的速度上限只接受 0..10。
原驱动的超范围截断已改为返回 `HAL_ERROR`；NaN/Inf 同样拒绝，且不发送报文。
MIT 报文若与特殊操作帧冲突也拒绝，防止控制输入意外触发使能/失能/置零。
位置速度与速度模式沿用参考文件的 STM32 小端 IEEE754 float 字节格式。

GimbalDrive 已接入输出限幅、遥控器急停、板间失联和重新使能保护；
Pitch 零位、机械限位及实机模式/量程仍需填写确认。目前看门狗仍约 802 ms 复位，电机反馈不喂狗。
尚未进行电机实测，也未发送置零或修改电机配置的命令。

## 验证

```powershell
python tests/run_motor_tests.py --pitch
cmake --build firmware/gimbal/build/Debug
cmake --build firmware/gimbal/build/Release
```

测试以 HAL 替身在 Cortex-M4 仿真中验证真实源码：初始化不发送、过滤器、
控制报文字节、参数拒绝、模式不匹配、邮箱忙、HAL 错误、反馈字段、坏帧拒绝、
时钟回绕超时及中断屏蔽状态恢复。不能替代实机 CAN 收发与运动验证。
