# 调试记录

每次记录日期、硬件配置、提交版本、测试步骤、预期结果、实际结果及待解决问题。

当前状态：底盘已烧录四轮低速遥控控制固件，四轮反馈在线；遥控当前离线，尚未验证新固件下四轮实际运动。

## 2026-11-15：离线控制核心

- 新增遥控映射、安全门控、坐标变换、底盘模式/四轮闭环、云台限位/闭环、姿态和板间协议。
- `python tests/run_motor_tests.py --all`：9 组通过，含失联停止、故障恢复与分片报文破坏测试。
- 底盘和云台 Debug/Release 均构建通过，未烧录、未启用运动控制。
- 硬件参数、传感器安装方向与控制器增益仍待实测；详见 `offline-control.md`。

## 2026-11-15：解除右后轮测试模式对四轮控制的阻断

- 只读比对确认原板上固件与右后轮点动测试构建一致，四路输出为零，遥控离线；四轮反馈计数持续增长，CAN ESR 为零。
- 关闭 `CHASSIS_RR_DIRECT_TEST` 和 `CHASSIS_RR_BENCH`，启用 100 rpm、±410 原始电流、Kp=5 的四轮初始参数；保留回中使能、急停及失联保护。
- Debug 构建通过，`python tests/run_motor_tests.py --logic` 通过，新增实际配置的四轮非零输出、限流限速及反馈丢失归零检查。
- 烧录前备份前 64 KiB 到 `tmp/before-four-wheel-enable.bin`；烧录后 53868 字节逐字节校验通过，见 `tmp/install-four-wheel-enable.log`。
- 启动回读：`chassis_configured=1`、反馈掩码 `0x0F`、CAN 发送状态 `HAL_OK`。因遥控离线，安全状态为 FAULT，输出保持零；未强制写入使能或驱动电流。
- 实机操作：遥控上线，左拨杆上，所有摇杆回中，右拨杆先中后上，再推左摇杆。右拨杆下为急停。原有看门狗在无有效遥控帧时仍会复位。

## 2026-11-15：落地起动限流调整

- 用户反馈四轮只轻微转动，整车几乎无位移，具体表现为转一下就停。
- 连续读取记录 `tmp/capture-drive-stall.log`：采样期间门控保持 ACTIVE、遥控在线、反馈掩码 0x0F、发送状态 HAL_OK；首组读取到目标 ±100 rpm、指令电流 ±410、四轮转速为零。分次内存读取不保证同一控制周期，随后摇杆回中，不能将全部停转都归因于限流。
- 原台架限流和纯比例低速输出不足是首要怀疑，尚未排除机械阻力和轮向问题。
- 初步落地参数调整为转子上限 400 rpm、原始电流上限 ±2000、Kp=10；平移指令上限仍为 0.20 m/s，保持急停、失联、反馈过期和回中使能保护。
- Debug 构建与控制逻辑测试通过；备份 `tmp/before-floor-tuning.bin` 后烧录，53868 字节校验通过，记录为 `tmp/install-floor-tuning.log`。
- 烧录后初始化成功，遥控在线、四轮反馈在线，处于 DISARMED、输出零；需要重新回中使能。新参数下的落地移动效果待用户验证。

## 2026-11-15：Pitch 实机被动反馈检查

- J-Link/SWD 连接云台板；回读 Flash 前 64 KiB，其中 38932 字节与当前云台 Debug 二进制完全一致，未烧录或发送电机控制命令。
- Pitch 离线测试通过。实机 pitch_init_status=HAL_OK，CAN2 ESR=0，FIFO1 无待取帧。
- 多次读取 pitch_feedback 全零，rx_count=0、received=false、pitch_online=false；延长观察超过 3 秒后仍未收到有效反馈，系统 tick 持续递增。
- 结论：当前仅接收状态下未观察到 CAN2 / 0x19 / 载荷 ID 9 的有效 Pitch 反馈，不能确认电机反馈正常，也不能据此认定电机损坏。尚未验证主动请求后的应答，应进一步核实电机供电、接线、ID、波特率与反馈触发方式。
- 原始记录：tmp/pitch-identify.log、tmp/pitch-feedback.log、tmp/pitch-feedback-long.log、tmp/pitch-feedback-final.log。

## 2026-11-15：遥控 Pitch / Yaw 控制接入

- 保留工作区现有 Yaw 驱动改动，新增 GimbalDrive 并接入云台主循环，沿用底盘板间命令和右拨杆使能映射。
- 两轴零力矩预装后发送使能，300 ms 内要求两轴新的已使能反馈；200 Hz 关节角/速度闭环。首次目标取当前反馈，Pitch 使用显式标定零位与机械限位。
- 急停、100 ms 断链/反馈过期、异常状态、超限或发送失败均停止两轴，取消待发旧命令，每 20 ms 重试失能；故障后必须重新回中、中位→上位使能。
- 配置位于 config/gimbal_setup.h。Yaw 实机量程、两电机 MIT 模式、Pitch 零位/限位和方向仍待确认，GIMBAL_HARDWARE_CONFIRMED 保持 0。当前构建不能遥控使能，未烧录或进行运动验证。
- 11 组离线测试通过，新增使能应答、旧反馈拒绝、单轴应答不足、发送错误、失能重试、断链、限位、禁止自动恢复和时钟回绕检查。最终云台 Debug/Release 均构建通过。
- 实机操作与参数说明见 firmware/gimbal/README.md 的“遥控双轴控制”。电机自身通信超时须实测，CAN 断线时不能以软件失能重试代替电机侧保护。

## 2026-11-15：录入 Pitch 实测标定

- 用户提供 pitch_feedback.motor.pos：零位 -0.184、上限 -1.143、下限 0.043 rad。
- gimbal_setup.h 中采用抬头为正，方向 -1，关节角 = -0.184 - 原始反馈角；关节限位为 [-0.227, 0.959] rad。位置、速度和力矩沿用同一方向适配。
- 实际标定的零位方向、上下限外向输出抑制和越界停止测试通过；云台 Debug/Release 构建通过。
- Yaw 量程、方向及两电机 MIT 模式/协议量程仍待确认，运动保护保持开启，本次未烧录。

## 2026-11-15：用户要求 Pitch 单轴遥控试车

- 新增 GIMBAL_PITCH_ONLY_TEST=1，Pitch 使用现有 MIT 驱动和用户实测标定；Yaw 不使能、不发送运动帧，Yaw 缺失反馈不阻断 Pitch。回中重新使能、反馈确认、急停、断链与力矩限幅继续有效。
- 原板完整 1 MiB Flash 备份至 tmp/before-pitch-remote.bin。首次单轴固件 51148 字节烧录及逐字节校验成功，gimbal_configured=1，遥控急停，Pitch 保持失能；反馈计数 263→273，CAN2 ESR=0。
- 观察到 Pitch 位于下限约 0.043 rad；添加 0.001 rad 端点测量容差，保持目标限位及向外输出禁止。单轴隔离、下限容差、越界停机测试与 Debug/Release 构建通过。
- 最终 51264 字节版本重烧时 J-Link/DAP 通信失败，未完成烧录准备及校验；重新连接同样失败，无法确认当前运行状态。需恢复 J-Link 后重新烧录与回读，不能视为最终版本已部署。
- 记录：tmp/install-pitch-remote.log、tmp/pitch-reconnect.log。恢复连接后，右拨杆中→上使能，右摇杆向前抬头，下拨急停；该使能拨杆也影响底盘，应保持左摇杆回中。
- 用户恢复 J-Link 后，以 SWD 1 MHz 重新下载检查：Flash 已与最终版本一致，51264 字节逐字节校验成功并复位运行。gimbal_configured=1，遥控有效且处于急停，Pitch state=0（失能），反馈计数 342→353，CAN2 ESR=0。等待用户回中、中→上使能后测试抬头，尚未确认实际抬头效果。最终记录：tmp/install-pitch-remote-final.log、tmp/pitch-remote-final-state.log。

## 2026-11-15：Pitch 无法抬头排查与速度环积分

- 初读遥控 pitch=1、arm=true，但状态 STOPPED、ready=false、电机失能。请用户回中、中位保持→上位保持后，采样进入 ACTIVE、故障码 0，电机 state=1。
- tmp/pitch-capture-arm.log 共 150 次采样，其中 110 次 ACTIVE 且抬头；原始位置约 0.04367～0.04377 rad 几乎不变，反馈力矩约 -0.09～-0.42 N·m，稳态命令约 -0.40 N·m。遥控和使能链路已正常，尚不能排除机械阻力。
- 发现纯比例速度环 Kp=2、速度上限 0.2 rad/s，使静止时输出最多约 0.4 N·m，不能用到 1 N·m 限幅。加入 Ki=2、积分贡献上限 0.8 N·m，保留总输出 1 N·m、原速度/限位/失联保护。
- 新增持续负载下输出能达到既有限幅的测试；云台测试及 Debug/Release 构建通过。先通过 configured=false 触发正常失能并回读 state=0，再烧录；51280 字节校验成功，重新运行后反馈持续增长、ESR=0，等待用户重新回中使能复测。
- 记录 tmp/pitch-stop-for-tuning.log、tmp/install-pitch-pi.log、tmp/pitch-pi-state.log。实际抬头效果仍待复测，不将控制参数修正等同于已排除全部机械问题。

## 2026-11-15：积分版仍不动，确认达到力矩上限

- 用户再试后采集 tmp/pitch-pi-capture.log：200 次采样均为 ACTIVE、fault=0，其中 170 次有效饱和样本，命令力矩幅值 0.99977～1.0 N·m，电机反馈 -1.005～-1.019 N·m，原始位置固定约 0.0428083 rad，速度近零。
- 已排除本次未使能、遥控未到达和输出达不到设定上限；负载需求超过当前输出或机械阻挡仍待区分，不能仅凭反馈认定机械故障或贸然提高力矩。
- 通过运行时 configured=false 触发原停机路径，回读 STOPPED、Pitch state=0，停止持续堵转输出；未改变 Flash 参数。记录 tmp/pitch-stall-stop.log。重启会恢复原配置，当前需先确认机械状态再继续试车。
- 已请用户右拨杆下位、托住云台检查手动抬升是否顺畅，等待结果后决定力矩调整或机械排障。

## 2026-11-16：Pitch 负载参数与持续无位移保护（仅代码）

- 用户确认可以顺畅手动抬起但较重，并明确要求先完成代码、不上板测试。本次未调用下载器、未连接板卡或修改板上固件。
- 将下一档 Pitch 工作力矩设为 2 N·m，积分贡献上限 1.8 N·m，Kp/Ki 仍为 2/2，速度上限仍为 0.2 rad/s；参数集中在 gimbal_setup.h。只是待实测参数，不推断其必然足够抬头。
- 新增 GIMBAL_STALL=8：目标误差至少 0.05 rad、输出达到工作力矩上限 90%，且 1000 ms 内位移不足 0.005 rad 时停止并清积分；位移进展刷新计时，小误差保持负载不误报。保护触发后需重新回中使能。
- last_fault 保留最近一次停机事件的起因，解除使能不会覆盖，下一次使能尝试清除。保留原急停、断链、反馈过期、限位和 Yaw 隔离。
- 新增/扩展离线测试覆盖 2 N·m 限幅、积分建立、持续无位移停机、带负载移动不误报、到位保持不误报、积分复位、故障保留、禁止自动恢复、计时回绕和无效保护参数。11 组测试全部通过，云台 Debug/Release 构建通过，git diff --check 通过。
- 结构体布局已改变；后续实机验证须以新 ELF 查询符号地址，不沿用 tmp 下历史固定地址脚本。尚未进行 2 N·m 版本实机验证。

## 2026-11-16 J-Link 云台运行检查

J-Link CE 69405108，SWD 1 MHz，VTref 约 3.29 V。读取 Flash 与最新 Debug
gimbal.bin 比较，52880 字节全部一致，未重复擦写。发现 main 入口硬件断点，
清除该断点后恢复运行。非停机 RAM 快照：board_command_count、watchdog_generation、
watchdog_consumed 均为 2363；remote_valid=1、arm=0。Yaw 参数 mode=1、
PMAX=3.14159298、VMAX=21、TMAX=7.5，valid_mask=15、无超时。
Yaw received=1、scaled=1、state=0、rx_count=1416，pos=-0.632345 rad、
vel=-0.015385 rad/s、torque=-0.005495 N·m。云台 STOPPED、fault=0、
configured/ready/stop_sent 均为 1，两轴发送状态 HAL_OK，控制输出为零。
程序保持运行；尚未操作遥控器使能，也未验证实际运动和运动后的停机。

用户打开使能后反馈平稳保持。J-Link 非停机快照确认 ACTIVE(3)、fault=0、last_fault=0、两轴 TX=HAL_OK，遥控四轴回中且 arm=1；Yaw state=1、pos=-0.632441 rad，目标=-0.632345 rad，位置差约 0.000096 rad。控制输出 Yaw 约 0.00513 N·m；板间/喂狗计数均为17165。静态保持通过，摇杆运动与失能停机仍待验证。

2026-11-16：用户重新放置 Pitch 允许最低位置；J-Link 读取原始 pos=0.0503823757 rad、motor.state=0。更新 GIMBAL_PITCH_LOWER_RAW=0.0503824f，关节下限=-0.2343824 rad（原 -0.227 rad），上限不变。读取时遥控 arm=1，已提示先关闭使能并回中；仍保留重新使能保护。

下限更新后的云台控制测试及 Debug/Release 构建通过；通过 J-Link 烧录 Debug HEX，并以 BIN 回读校验 52880 字节成功，复位恢复运行。

2026-11-16 Yaw 启动吃力：运行快照 ACTIVE、无故障，摇杆回中，目标 -0.481754 rad、实际 -0.549509 rad，目标速度 0.203264 rad/s，输出 0.208392 N·m、反馈 0.203297 N·m，速度接近零。准备将速度比例 kp 从 1 增到 1.2，扭矩上限仍为 0.3 N·m，待失能后下载验证；该静态快照不足以排除机械摩擦或线束影响。

用户关闭使能后，J-Link 确认两电机 state=0、arm=0；烧录 kp=1.2 的 Debug 固件并回读校验 52884 字节成功，复位运行。

## 2026-11-16 Yaw「转到约半圈卡住、须重新使能」的代码复核（未上板）

- 现象：Yaw 朝一个方向转约半圈后停住，反向拨杆无响应，只能遥控重新使能后恢复。
- 板上固件不是当前工作区代码：`firmware/gimbal/build/Debug/gimbal.bin` 为 53300 字节（10:05:30），而最近一次烧录/回读校验为 52884 字节，差值即未提交的 yaw_input_direction 改动（GimbalControl 220→224 字节）。即板上仍是「目标无界积分」的旧逻辑。
- 根因（旧逻辑）：`modules/gimbal/gimbal_control.c:55` 只做 `s->target[0] = Motion_Wrap(s->target[0] + cmd->yaw * c->yaw_command_rate * dt)`，松杆不重置目标、误差不钳制，且 Yaw 无任何行程限位（GimbalConfig 仅有 pitch_min/pitch_max）。被线束或机械挡住后目标仍以 0.3 rad/s 累加；松杆后误差保持，速度环持续输出 0.3 N·m 顶住；反向拨杆必须先把 windup 退回去（π 需满杆 10.5 s）才见动作，故表现为“卡住、只有重新使能才能动”。使能时 `Gimbal_Reset` 把 target 清零、tracking 重新取反馈角，因此总能立即恢复——与现象完全一致。
- 次生现象：windup 超过 π 后 `Motion_Wrap` 令误差由 +π 跳到 −π，指令速度整幅反向，可能表现为猛然反向或来回顶死。
- 易感因素：Yaw 速度环 output_limit 仅 0.3 N·m、无堵转保护（Pitch 有 GIMBAL_STALL=8）；`README.md` 已注明“Yaw 没有线束角度限位，不能持续单方向转动”，半圈处很可能就是线束极限，0.3 N·m 顶不过去。
- 未排除的替代分支：Pitch 下限 −0.2343824 rad 恰等于自重静止位（raw 0.0503824），越限 0.001 rad 即触发 GIMBAL_LIMIT(4) 整机停机；Yaw 线束拉扯致 D 帧丢失则触发 GIMBAL_FEEDBACK(3)。两者同样表现为“整机失能、需重新使能”。
- 判别方法（卡住当下读快照，不要立刻重新使能）：ACTIVE(3)、fault=0、输出 0.3 N·m、Yaw 速度≈0 且 target 领先 pos ≥0.1 rad → 确认目标 windup；STOPPED(0) 且 fault=4 或 3 → 走限位/反馈分支。
- 工作区未提交的改动（目标钳制到 max_error=max_rate/angle_kp=0.1 rad、松杆与反向时从反馈角重建目标并复位 PID）正是针对该根因，尚未烧录验证。

## 2026-11-16 云台状态帧回传与底盘相对 yaw 接入（P0-1，未上板）

- 目的：解决分离/跟随缺少数值来源。此前底盘 `Chassis_Step` 的 yaw 恒为 0，
  且 `sensors_fresh` 只在分离模式为真，跟随/小陀螺被安全门直接禁止。
- 上板新增 `ServiceStatusFrames`：每 20 ms 组成一帧 `LinkStatus`（0x310..0x312），
  每个主循环只发 1 帧且要求 CAN1 至少剩 2 个空邮箱，给 Yaw MIT 力矩帧留 1 个；
  100 ms 发不完则整批丢弃，下批换新序号。字段为 yaw 关节角（已按
  `GIMBAL_YAW_DIRECTION` 和 `GIMBAL_YAW_ZERO` 换算并回绕）、pitch 关节角（夹到限位）、
  calibrated/motors_healthy/sensors_fresh/fault。状态帧不喂看门狗。
- 上板 CAN1 发送策略相应放宽：`YawMotor` 的 `send_id` 只在 0 个空邮箱时返回
  `HAL_BUSY`，`GimbalDrive` 的 `empty()` 对 CAN1 只要求 ≥1 个空邮箱；CAN2 仍要求全空。
  若维持“必须 3 个全空”，状态帧一占邮箱，Yaw 力矩帧就会报 `HAL_BUSY` 并被误判为
  `GIMBAL_TX`。
- 下板新增 CAN1 bank0 16 位 IDLIST 滤波器（0x310/0x311/0x312）、FIFO0 中断、
  `Link_Init(LINK_STATUS, 50, 100)`；中断中只解标准帧/DLC8，CRC/序号由
  `Link_Receive` 校验，成功后仅更新计数与最后一帧，不喂 IWDG。
- 下板 `BoardYaw` 要求状态新鲜（<100 ms）、`calibrated`、无 `fault`、数值有限；
  相对 yaw 单周期跳变超过 `CHASSIS_YAW_JUMP_RAD=0.3` 时判为无效并清基线，
  安全门进入 FAULT，需重新回中并使能。退出 ACTIVE 时清基线，避免重新使能时误判。
- 安全门：任何模式都要求状态帧新鲜；分离/跟随还要求 yaw 有效；跟随另受
  `CHASSIS_FOLLOW_IMU_READY=0` 限制（云台 yaw 尚未世界系稳定，否则底盘会一直转到
  `max_rotation`）。`Chassis_Step` 现在接收真实相对 yaw。
- 零位：新增 `GIMBAL_YAW_ZERO` 与 `GIMBAL_YAW_ZERO_CONFIRMED`，默认未确认，
  此时上板状态 `calibrated=false`，下板分离/跟随保持停机。上车第一步必须实测零位。
- 验证：`python tests/run_motor_tests.py` 的十一组用例全部 PASS（本机 DSH 沙箱下
  `tempfile` 目录不可写，用 `tmp/run_tests_sandbox.py` 包装后运行）；
  两板 Debug/Release 均以 MinGW Makefiles 构建通过，无 `-Werror` 告警。
- 待实机：两板 CAN1 实际收发与状态帧计数、`GIMBAL_YAW_ZERO` 标定、分离/小陀螺低速
  运动、状态帧拔线/坏 CRC/跳变时的停机、跟随需等云台世界系稳定后再打开。
