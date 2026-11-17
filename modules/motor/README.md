# M3508 / C620 底盘 CAN 通信

当前实现使用下板 CAN2，适配已有 CubeMX 配置：PB5/RX、PB6/TX，
1 Mbit/s，FIFO1 接收中断。四轮 ID 顺序为左前 1、左后 2、右后 3、右前 4。
本电机模块不包含 PID、运动学和解锁状态机；遥控器解包由 `modules/remote` 提供。

## 协议

- 接收标准数据帧 `0x201..0x204`，DLC 必须为 8。
- 字节 0..1：转子编码器值 0..8191；2..3：有符号转子转速 rpm；
  4..5：有符号反馈电流原始值；6：温度 °C；7：保留。
- 发送标准数据帧 `0x200`，四个有符号电流指令按 ID 1..4 排列，每项高字节在前。
- 指令自动限制在协议范围 ±16384（对应 ±20 A）。这是协议上限，
  **不是调试建议值**；控制层应设置更小的实测电流限制。
- 反馈为转子数据，不在通信层写死减速比或安装方向。

来源：[DJI C620 使用说明](https://cdn-hz.robomaster.com/robomasters/public/document/RoboMaster%20C620%20Brushless%20DC%20Motor%20Speed%20Controller%20V1.01.pdf)。

## 已有接入

`main.c` 的 USER CODE 区调用 `M3508_CAN_Init(&hcan2)`，并在
`HAL_CAN_RxFifo1MsgPendingCallback()` 中转发接收事件；主循环约每 10 ms 按安全门控发送一次电流。运行时使用四轮闭环输出，停机时发零电流。调试器可观察 `motor_can_tx_status`，
以及模块内部 `feedback` 数组；业务代码请用快照接口读取反馈。

```c
M3508_Feedback left_front;
if (M3508_CAN_GetFeedback(1, &left_front) &&
    M3508_CAN_IsOnline(1, 100)) {
    // left_front.speed_rpm 为电机转子转速。
}

// 电流顺序：左前、左后、右后、右前；此例保持零输出。
const int16_t currents[M3508_COUNT] = {0, 0, 0, 0};
HAL_StatusTypeDef status = M3508_CAN_SendCurrents(currents);
// HAL_BUSY：上一帧尚未发送完成，下个周期重试最新目标。
// HAL_ERROR：检查 HAL_CAN_GetError(&hcan2) 和总线状态。
```

发送接口仅由一个主循环/控制任务调用，不要同时从中断和主循环发送。
HAL_OK 只表示入邮箱，不保证对端收到。仅允许一个未完成发送，避免堆积
旧控制指令；未收到 ACK 时自动重发可能使邮箱长期占用，零指令也可能返回
HAL_BUSY。控制层必须监测持续失败，重试停止指令，并依赖电调端失联保护；
总线断线时 MCU 无法保证送达停止帧。

过滤器使用 CAN2 bank 14、16 位列表模式，精确接收四个反馈 ID。
CAN1/CAN2 共享过滤器边界固定为 14；后续 CAN1 过滤器用 bank 0..13，
所有过滤器配置的 SlaveStartFilterBank 都保持 14。CAN1 未由本模块启动。

当前 `.ioc` 的 CAN 位段为 BS1=6、BS2=7，波特率为 1 Mbit/s，采样点 50%。
若现场需要调整，可在 CubeMX 改为 BS1=11、BS2=2（采样点约 85.7%），
保持 Prescaler=3 与 PCLK1=42 MHz；需重新生成并实测。
当前 AutoBusOff 关闭，bus-off 后不会自动恢复，需后续明确恢复策略。

## 看门狗和上电行为

现有工程初始化了 IWDG（64/400，按 32 kHz 约 802 ms），遥控器模块已接入
合法帧喂狗。没有遥控器有效帧时仍会周期性复位；没有以电机反馈或
主循环无条件喂狗替代遥控器保护。解锁和急停已接入安全门控。上电待机、急停和故障时发零电流，底盘仍可能因惯性滑行。

## 编译与验证

在 `firmware/chassis` 中执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

在仓库根目录可运行无硬件测试，需要 Python 的 `unicorn==2.1.4` 和 PATH
中的 Arm GNU 工具链：

```powershell
python tests/run_motor_tests.py --all
```

测试将真实驱动和 HAL 替身编译为 Cortex-M4 指令并执行，覆盖 ID/帧类型/
长度检查、负数解析、电流字节序与限幅、邮箱忙、离线超时及计时回绕。
四轮反馈曾在线，当前基线的实机闭环与停止行为见[验收清单](../../docs/acceptance.md)。

## 云台 Pitch

DM4340 驱动与 CAN2 Pitch 组件见 [pitch_motor.md](pitch_motor.md)。
