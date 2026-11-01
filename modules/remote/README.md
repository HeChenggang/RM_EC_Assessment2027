# DT7 / DR16 遥控器驱动

使用 C 板 DBUS 接口对应的 USART3 RX（PC11）：100000 bit/s、9 位含偶校验、
1 停止位，DMA1 Stream1 / Channel4，循环模式、字节对齐。C 板 DBUS 接口已有
反相电路。驱动接入现有配置，不需要重新生成 CubeMX 工程。

参考：[RoboMaster C 型开发板嵌入式软件教程](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/attachment/pre/v1/202312/05/212942k01113z230s4x80x.attach/RoboMaster%E5%BC%80%E5%8F%91%E6%9D%BFC%E5%9E%8B%E5%B5%8C%E5%85%A5%E5%BC%8F%E8%BD%AF%E4%BB%B6%E6%95%99%E7%A8%8B%E6%96%87%E6%A1%A3.pdf)。

## 接口和字段

`DR16_Init(&huart3)` 初始化一次；主循环调用 `DR16_Service()`，处理错误后的
接收重启。`main.c` 的 USER CODE 区已经接入上述接口和 HAL 接收/错误回调。

```c
DR16_State rc;
if (DR16_GetState(&rc)) {
    // channel[0] 右摇杆水平，channel[1] 右摇杆竖直
    // channel[2] 左摇杆水平，channel[3] 左摇杆竖直，channel[4] 拨轮
    // 原始中心化数值 -660..660；通信层不加入死区、不改变方向。
    float right_horizontal = rc.channel[0] / 660.0f;
    (void)right_horizontal;
} else {
    // 尚无有效帧、异常帧、UART 错误或超过 100 ms 未更新。
    // rc 的所有控制字段已清零；此分支必须禁止电机输出。
}
```

拨杆枚举：UP=1、DOWN=2、MIDDLE=3、INVALID=0。
`s1` 对应第 5 字节 bits 7:6，`s2` 对应 bits 5:4；上板后单独拨动两拨杆，
确认实际左右对应，再配置模式/急停映射。驱动不自行定义解锁或急停拨杆。
`mouse_x/y/z` 为有符号 16 位值；`mouse_left/right` 为 0/1。
`keyboard` 从 bit0 到 bit15 依次是 W、S、A、D、Shift、Ctrl、Q、E、R、F、G、Z、X、C、V、B。

`last_rx_ms`、`valid_frames`、`invalid_frames`、`uart_errors` 用于诊断，
离线时仍保留；获取函数用短临界区保证同一帧快照。

## DMA 与帧校验

64 字节循环 DMA 缓冲区保留 HT/TC 中断；HAL 回调的 `Size` 是缓冲区写入位置。
驱动累积两次位置之间的新数据，跨半缓冲区或末尾时继续拼接。
仅在 IDLE 事件且累计长度恰好 18 字节时解码；TC 后同位置的 IDLE 不重复读取。
重复 IDLE 不重复计数、不重复喂狗。UART 与 RX DMA 必须保持相同抢占优先级
（当前均为 0），不要在回调里延时、打印或阻塞，避免 DMA 覆盖未处理的数据。

所有五个通道（含拨轮）原始值必须在 364..1684，拨杆必须为 1/2/3，
鼠标按钮必须为 0/1。短帧、超长/粘连帧及非法字段被丢弃，并立即使输入无效；
下个完整 IDLE 帧可恢复。DBUS 没有帧头或 CRC，这些检查只做合理性验证，
不能识别所有恰好落在合法范围内的错误数据。旧协议若不提供拨轮、末两字节
一直为零，会被当前 DT7 配置拒绝，需先确认接收机协议，不能直接取消全部校验。

UART/DMA 错误立即使输入无效，主循环中 abort 接收并重新启动；恢复失败
返回 HAL_BUSY/HAL_ERROR/HAL_TIMEOUT，后续主循环重试，直到恢复或 IWDG 复位。

## 喂狗和当前上电行为

只有 `DR16_RxEventCallback()` 返回 true 时，主程序才调用
`HAL_IWDG_Refresh(&hiwdg)`。HT/TC、错误/短帧和重复回调均不喂狗。
正常接收合法帧后可持续运行；关闭遥控器后，100 ms 起输入无效，随后由
现有 IWDG（64/400，按 32 kHz 约 802 ms）触发复位。实际超时受 LSI 误差影响。
若开机没有遥控器，仍会反复复位，这符合当前的遥控器喂狗方式。

调试器观察 `remote_online`、`remote_state` 和 `remote_rx_status`。
当前主程序依然每 10 ms 发送四轮零电流；本次没有加入摇杆到电流的直通控制。
后续做底盘闭环时，必须将在线检查、明确的急停拨杆、启动/掉线后重新解锁
纳入控制状态机。合法遥控帧恢复不应直接重新启动车辆。

## 验证

固件：在 `firmware/chassis` 下运行 `cmake --build --preset Debug`。
模拟测试：安装 `unicorn==2.1.4`，Arm GNU 工具链加入 PATH，在仓库根目录运行：

```powershell
python tests/run_motor_tests.py --remote
python tests/run_motor_tests.py
```

测试覆盖解码边界、负数、按键、错误长度/字段、循环 DMA 的 HT/TC/IDLE、
跨界拼接、重复事件、100 ms 超时/计时回绕、错误恢复及临界区状态保留。
这不代替实物验证：先架空底盘观察各通道/拨杆，再关闭遥控器确认离线及复位。
