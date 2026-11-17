# 调试记录

每次记录日期、硬件配置、提交版本、测试步骤、预期结果、实际结果及待解决问题。

## 2026-11-17：Yaw 松杆超调，两次加阻尼尝试均失败，保持低 Kp

- 现象：Yaw 在提到 2.00 rad/s 后有超调。松杆或反向时控制层用反馈角重建目标并清零速度环积分（`gimbal_control.c:90-95`），制动只靠比例项：Kp=1.2 时 2 rad/s 仅有约 2.4 N·m 制动力，停下来会冲过目标。
- 尝试 1：`GIMBAL_YAW_SPEED_KP` 1.2 → 2.5，制动约 5 N·m（上限 7 N·m）。实机剧烈抖动——Yaw 轴轻、5 ms 控制周期加 CAN 延迟，高 Kp 把速率反馈噪声/延迟放大成极限环。回退到 1.5（制动约 3 N·m，`GIMBAL_YAW_INTEGRAL_LIMIT` 自动为 7 − 1.5×2 = 4.0 N·m）。
- 尝试 2：新增 `GIMBAL_YAW_COAST_KP`（3.0 N·m/(rad/s)），仅摇杆回中时叠加 `-Kp*rate*|rate|/(|rate|+0.4)`，静止附近淡出；实机仍剧烈抖动，已整体撤销（宏、配置字段、前馈项、回归断言均已删除）。
- 尝试 3：撤掉 coast 后 Kp 仍为 1.5，用户反馈「推杆转动、经过某个角度时抖」。未使能手转该角度顺畅，排除机械卡点；代码在 ±π 处对目标/误差都做了 `Motion_Wrap`，不存在角度相关的软件跳变。判断是该角度负载/线束刚度把速率环推到稳定边界，遂把 `GIMBAL_YAW_SPEED_KP` 1.5 → 1.2（长期使用的稳定值；1.2 时 2 rad/s 制动力约 2.4 N·m）。
- 结论：该轴稳定边界低，速率型阻尼和较高 Kp 都不能用；松杆超调（约 2°）只能靠降低 `GIMBAL_YAW_MAX_RATE`（超调距离 ∝ ω/Kp）或接受。
- 尝试 4：为减小松杆滑行，`GIMBAL_YAW_MAX_RATE` 2.00 → 1.70 → 1.50 rad/s（滑行距离与满杆转速成正比，约 2° → 1.5°），速度环保持 Kp=1.2；`GIMBAL_YAW_INTEGRAL_LIMIT` 表达式自动变为 7 − 1.2×1.5 = 5.2 N·m。
- 尝试 5：松杆瞬间锁存实测 yaw 速度 v0，在 12 ms 内叠加开环制动力矩 `-K·v0`（K=3.5 N·m/(rad/s)，1.5 rad/s 时约 5.2 N·m）。实测出现抖动，判定不可用并整体撤销（宏、配置字段、状态机、测试断言均已删除），Yaw 回到 1.5 rad/s + Kp 1.2 的无制动版本。
- 测试：`python tests/run_motor_tests.py --all` 11 组通过；云台 Debug/Release 重新构建通过。未烧录、未实机验证。

## 2026-11-17：Pitch 速度环阻尼加倍，抑制保持刚度带来的过冲

- 现象：`GIMBAL_PITCH_HOLD_KP` 回调到 150 N·m/rad 后实机仍有轻微过冲。刚度约 156 N·m/rad，而阻尼只有 `angle_kp*Kp = 3*2 = 6` N·m/(rad/s)，属于欠阻尼，继续降刚度会重新变软。
- 修改 `GIMBAL_PITCH_SPEED_KP` 2.0 → 4.0，阻尼提到约 12 N·m/(rad/s)，静态刚度基本不变（`angle_kp*Kp` 只增加 6 N·m/rad）。速度环 Kp 再高会把编码器差分噪声放大成力矩纹波。
- 测试：`python tests/run_motor_tests.py --all` 11 组通过；云台 Debug/Release 重新构建通过。未烧录、未实机验证。

## 2026-11-17：Pitch 静止保持刚度定为 150 N·m/rad

- 现象：用户反馈 Pitch「力矩过低/太软」。实机表现为静止或被手推时太软；软件力矩上限已是 `DM4340_T_MAX`（28 N·m，电机峰值 27 N·m），没有可再抬的余量，因此改调位置保持刚度。20 → 100 → 150 → 200 后实机出现超调，回调到 150。
- 修改 `GIMBAL_PITCH_HOLD_KP` 20 → 150 N·m/rad。小偏差静态刚度约 `150 + angle_kp*Kp = 156` N·m/rad：0.1 rad 偏差立即约 15 N·m（原 20 时约 2 N·m），约 0.18 rad 达到 28 N·m 上限。
- 副作用：手推偏差超过约 0.16 rad 且 1 s 内位移不足 0.005 rad 会触发既有堵转停机。200（约 206 N·m/rad）实机过冲，150 是当前稳定上限；以后再调应优先加阻尼（速度环 Kp）而不是继续加刚度。
- 测试：一步满杆力矩上界 2 N·m，0.1 rad 外力恢复力矩断言 14 N·m，并锁定 `pitch_hold_kp==150`；`python tests/run_motor_tests.py --all` 11 组通过；云台 Debug/Release 重新构建通过（Flash 67136 / 41080 字节）。未烧录、未实机验证。

## 2026-11-17：Pitch 速度上限提高到 2.00 rad/s

- 用户反馈 Pitch 操纵过慢。新增 `GIMBAL_PITCH_MAX_RATE 2.00f`，替换 `pitch_command_rate` 和外环 `max_rate[1]` 的 0.25 字面量，与 Yaw 同级；关节限位、向外力矩禁止、带载堵转和 28 N·m 力矩上限保持不变。
- 外环 `angle_kp=3` 下维持 2.00 rad/s 需要约 0.67 rad 跟随误差，松杆后关节会滑向冻结目标约 0.2 rad（≈12.7°）。实机若觉得停位发飘，可提高 `angle_kp[1]`，或像 Yaw 一样在松杆时从反馈重建 Pitch 目标。
- 离线测试 `python tests/run_motor_tests.py --all` 11 组通过。生产配置的冻结轴满杆用例原先假定 6 s 内保持 ACTIVE，现改为断言带载堵转在约 1～2 s 内停机，峰值力矩达到 DM4340 MIT 上限 28 N·m。
- 云台 Debug / Release 重新构建通过（Flash 67116 / 41064 字节）。未烧录、未实机验证。

## 2026-11-17：Pitch 下止点导致整机无法使能

- 下板遥控有效、四轮反馈齐全，收到上板故障后保持零输出。重新使能复现；上板 `gimbal_drive.last_fault=4`，Pitch 实际机械下止点原始读数约 0.07426 rad，超出原 0.060 rad 下端配置。
- 将下止点配置为 0.074 rad（关节下限 −0.258 rad）。原有 0.001 rad 端点容差覆盖实测值，端点向外力矩和超容差停机均通过离线测试。11 组离线测试和上板 Debug/Release 构建通过。
- 备份上板 Flash 后烧录 Debug 固件，67120 字节逐字节校验通过。在实测下止点重新使能，读回上板 ACTIVE、故障码 0。用户在分离档确认底盘恢复移动。

## 2026-11-17：考核复核

- 基线 ebc26db。11 组 Cortex-M4 仿真测试、两板 Debug/Release 重新构建通过。GitHub main 与该基线一致。
- 复核时的配置为底盘 3000 rpm、1.80 m/s、旋转 3.00 rad/s，Yaw 2.00 rad/s 和 0.50 N·m，Pitch 0.25 rad/s，关节下限 −0.244 rad。
- 分离档已经接入真实相对 Yaw，跟随和小陀螺以 IMU 世界 Yaw 闭环。上电静置门控缺失，Pitch 高负载保护门槛约 25.2 N·m。详见[验收清单](acceptance.md)。
- 本次未连板、烧录或复验实车。下列记录对应各次历史硬件与软件状态，当前板上版本尚未查验。

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

- 后续状态：用户已确认 Yaw 360° 无缝旋转问题解决，见[验收清单](acceptance.md)。以下为问题发生时的排查记录。

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

## 2026-11-16：云台 yaw 世界系稳定骨架（P0-2，未上板）

- 目的：跟随需要“底盘转、云台保持世界朝向”，关节角才会随底盘旋转减小；此前
  yaw 轴只用关节角反馈，底盘转到哪云台跟到哪，跟随环不收敛。P0-2 先打通
  BMI088 → 姿态解算 → 世界 yaw 反馈 → `Gimbal_Step` 的链路。
- 上板主循环新增 IMU 段：`BMI088_Service` 返回新陀螺样本时，用
  `HAL_GetTick()` 差算 dt，把 dps 乘 π/180 后调用
  `Attitude_Update(accel, gyro, dt, &attitude_out)`；解算成功才置
  `attitude_ready`，并在 `GIMBAL_YAW_WORLD` 时
  `GimbalDrive_SetWorld(yaw, yaw_rate, HAL_GetTick())`。未完成陀螺零偏标定时
  每 1 s 重试 `BMI088_StartGyroCalibration`，且只在未使能时重试。
- `GimbalDrive` 支持 yaw 轴 `GIMBAL_WORLD`：Init 只对 pitch 强制 JOINT；世界
  样本超过 30 ms 未更新时按 `GIMBAL_FEEDBACK` 停机；反馈同时填
  `world/world_rate/world_fresh`。`GIMBAL_YAW_WORLD=0` 时行为与之前完全一致。
- 修复既有隐患：`GimbalDrive_Step` 用 `elapsed*0.001f` 算 dt，20 ms 浮点舍入后
  可能略大于 `control.max_dt=0.02f`，导致误报 `GIMBAL_LIMIT`。现把 dt 显式钳到
  `max_dt`，新增 WORLD 过期用例后 `--gimbal`、`--logic` 均通过。
- 安全：世界 yaw 未就绪、姿态解算失败、样本过期、IMU 未标定都会退出使能而不是
  继续用旧角度；`GIMBAL_YAW_WORLD` 默认 0，实机验证前不改变现有行为。
- 待实机：BMI088 安装矩阵 `GIMBAL_IMU_SENSOR_TO_BODY`、陀螺零偏标定时间、
  世界 yaw 漂移速度、云台锁定世界朝向时的手感、底盘 follow 闭环整定（先把
  `CHASSIS_FOLLOW_IMU_READY` 置 1，低速小 `follow_kp` 起步）。
## 2026-11-16：小陀螺方案 A 定稿（整机随转）

- 决策：小陀螺先用方案 A——底盘以固定 `spin_rate` 整机旋转，云台 yaw 保持关节角随
  底盘一起转。理由是现有关节控制下它已经能工作，线束不会在 yaw 轴上持续缠绕，满足
  “保留非零旋转速度并支持平移”的验收字面要求；代价是云台视角跟着转。
- 方案 B（云台世界系锁定、底盘在下面转）留到世界系验证之后，需要限时/限圈或滑环，
  且与 `GIMBAL_YAW_WORLD` 绑定；开启前不能用 B 跑小陀螺，否则 yaw 关节会一直绕线。
- 参数：`CHASSIS_SPIN_RATE_RAD_S=0.50`、`CHASSIS_MAX_ROTATION_RAD_S=1.0` 收到
  `chassis_setup.h`，`InitChassisControl` 改用它，实机调参不必改 main。
- 行为：spin 的 wz 不随相对 yaw 变化；平移仍按相对 yaw 旋转（摇杆前推时整机边走边
  转，轨迹是圆弧）。spin 不要求 yaw 有效，但要求状态帧新鲜；`GIMBAL_YAW_ZERO` 未
  标定时 spin 仍可动，只是平移按底盘系解释。
- 测试：`test_robot_logic.c` 的 spin 用例改为相对 yaw=3 rad、连跑 60 步，断言
  `wz=spin_rate` 且 `vx/vy` 等于按 3 rad 旋转后的平移，覆盖“旋转与平移解耦”。
- 待实机：转向正负（正 wz 应为俯视逆时针）、0.5 rad/s 下的平移手感、轮速是否饱和、
  急停/断链时旋转立即归零。

## 2026-11-16：Pitch 解除工作力矩上限（未上板）

- 现象：2 N·m 工作上限、积分 1.8 N·m 下 Pitch 实机仍无法抬头；用户确认手动可顺畅
  抬起但较重。更早的 1 N·m 档命令已饱和、反馈约 -1.0 N·m、位置固定 0.0428 rad。
- 修改 `config/gimbal_setup.h`：`GIMBAL_PITCH_TORQUE_LIMIT` 由 `2.0f` 改为
  `DM4340_T_MAX`（28 N·m），`GIMBAL_PITCH_INTEGRAL_LIMIT` 由 `1.8f` 改为
  `0.9f * DM4340_T_MAX`（25.2 N·m）；Kp/Ki 仍 2/2。只抬总输出而保留 1.8 积分上限时，
  零误差稳态保持仍被积分项限制在约 1.8～2.2 N·m，等于没有解除，故同步放大积分。
- 保留：关节限位 [-0.2343824, 0.959] rad、端点向外力矩禁止、堵转停机、
  命令/反馈/急停/周期看门狗。`GimbalDrive_Init` 仍校验
  `output_limit <= DM4340_T_MAX`，`dm4340_mit_ctrl` 对超量程返回 HAL_ERROR，
  因此 28 N·m 是软件可达上限，不是机械或热保证。
- 取舍：堵转判据为 `|torque| >= 0.9*output_limit` 且目标误差 >= 0.05 rad 且
  1000 ms 位移 < 0.005 rad；上限改成 28 N·m 后触发点约 25.2 N·m，普通“负载偏大”
  不再快速停机，带载抬头时可能长时间大电流堵转。首次带载需随时急停并观察电机温度。
- 测试：`tests/test_gimbal_drive.c` 增加生产配置断言（`output_limit==DM4340_T_MAX`、
  `integral_limit==0.9*DM4340_T_MAX`）和冻结关节、`cmd.pitch=1` 持续 6 s 的用例，
  断言力矩超过 2 N·m（约 2.7 N·m）且不超过 DM4340 量程；原 Pitch-only 场景显式
  改回 2 N·m/1.8 N·m，保留原有 1000 步积分建立与堵转窗口时序。
  `python tmp/run_tests_sandbox.py --gimbal` 通过。本次 ninja 在沙箱内启动编译器会挂起，
  改用 `tmp/build_gimbal_manual.py` 逐条执行 ninja 列出的命令，Debug/Release 均构建通过
  （Debug FLASH 65.7 KB，Release 40.4 KB）。
- 待实机：烧录后先空载确认方向与限位，再小幅带载抬头；若 28 N·m 仍不足，问题更可能
  在机械（配重、线束、减速器）而非软件限幅，不应继续抬上限。

## 2026-11-16：常规小陀螺（替代方案 A）

- 用户确认采用底盘旋转、云台保持朝向。MODE_SPIN 强制世界系 Yaw，模式切换捕获
  当前角度；摇杆仍能调整航向，平移按实测相对 yaw 转换。
- 上板新增安装/零位确认门控、4 s 累计时间与 ±2 rad 关节转角保护；模式切换不
  清空预算。IMU 失败立即废弃样本，30 ms 无更新停机；故障状态传播到下板。
- 下板小陀螺不再允许未标定 yaw 回退。确认位保留 0，未烧录或实机验证。
- 11 组离线测试及两板 Debug/Release 构建通过。详见 [小陀螺模式](spin-mode.md)。

## 2026-11-16：IMU 零偏标定反复失败（阈值压在噪声峰峰值上）

- 现象：`imu_calibration` 的 `running`/`failed` 约每 2 s 翻转一次，`collected` 涨到
  1000 后归 0，`calibrated` 始终为 0。`main.c:310` 因此在未标定时强制
  `attitude_ready=false`、`world_received=false`，世界系 yaw 整条链路不可用。
- 机制：`finish_calibration` 唯一的拒绝路径是 `spread_dps > CALIBRATION_MAX_SPREAD_DPS`；
  `failed` 只保留到下一次 `BMI088_StartGyroCalibration`（它清 `failed`/`collected`），
  所以表现为两个标志来回跳。
- 数值依据：BMI088 陀螺噪声密度约 0.014 dps/√Hz，0x10=0x01 为 230 Hz 滤波
  （噪声带宽 ≈1.57×230≈361 Hz），σ≈0.27 dps；N=1000 个样本的峰峰值期望
  `2σ√(2lnN)`≈2.0 dps，三轴取最大约 2.1 dps。原阈值 2.0 dps 正好等于静止噪声的
  峰峰值，采满一窗却被判"被移动"是必然的。
- 改动：`BMI088_CALIBRATION_MAX_SPREAD_DPS` 2.0→4.0（保留约 2 倍余量）；
  `BMI088_GyroCalibration` 新增 `window_ms`，`StartGyroCalibration` 记录起点、
  `finish_calibration` 结算实际耗时，用来区分"噪声"与"主循环慢到丢样本"。
  README 同步写入阈值依据，以及想压得更紧时应改 0x10 带宽档而非收紧阈值。
- 未改变的部分：均值估计本来就精确（1000 样本均值的标准误 ≈0.008 dps），峰峰值只是
  粗糙的"动没动"探测器，且对匀速转动本来无感，所以放宽到 4 dps 不损失真正的运动检测
  （手碰/被推会产生几十 dps 的尖峰）。
- 测试：`python tmp/run_tests_sandbox.py --imu`、`--attitude` 通过。直接跑
  `tests/run_motor_tests.py` 在 DSH 沙箱内会因 `tempfile.mkdtemp` 的 0700 ACL 无法写入
  而失败，必须走 `tmp/run_tests_sandbox.py`。
- 待实机：烧录后读 `imu_calibration`——预期 `calibrated==1`、`spread_dps` 约 1.5–2.5、
  `window_ms` 约 1000（明显偏大说明主循环比数据就绪慢、样本被丢）。若仍失败且
  `spread_dps` 达几十 dps，则不是噪声问题，要查线束拉扯、云台缓慢漂移或单帧 SPI 位错。

## 2026-11-16（续）：峰峰值判据换成半窗均值差

- 实测：4 dps 版本编译烧录后仍是 `calibrated=0`、`failed=1`、`collected=1000/1000`，
  而 `spread_dps = 4.4556641`。也就是说云台静止不动，一窗的峰峰值就有 4.46 dps
  （≈73 个 LSB，1 LSB = 0.061 dps），是白噪声期望 2.0 dps 的两倍多：峰峰值被单个最坏
  样本支配，尾部比按 σ 推的宽。
- 判断：继续抬峰峰值阈值只能猜数字——4.46 之上该写 5 还是 6 没有物理依据，换块板噪声
  不同又会失败，而且这个判据对匀速转动本来就无感。
- 改动：动作判据换成**半窗均值差**，峰峰值降级为数据损坏保险。
  - `modules/imu/bmi088.h`：新增 `BMI088_CALIBRATION_MAX_DRIFT_DPS 1.0f`；
    `BMI088_CALIBRATION_MAX_SPREAD_DPS` 4.0→20.0（约静止实测值的 4.5 倍、噪声期望的
    10 倍，只有总线/位错风暴才会顶穿）；`BMI088_GyroCalibration` 新增 `drift_dps`。
  - `modules/imu/bmi088.c`：累积到 `target/2` 时快照 `calibration_half_sum`；
    `finish_calibration` 里 `drift = 三轴最大 |后半均值 − 前半均值|`，与 spread 一起进
    拒绝条件。
- 数值依据：静止时半窗均值差约 0.02 dps（σ/√500×√2），单个坏样本只贡献 G/500
  （8 dps 尖峰 → 0.016 dps），而窗口中角速度发生阶跃 r dps 且与半窗边界对齐时会贡献 r。整窗恒定角速度会进入所估计的零偏，1 dps 阈值也无法约束这种误差。
- 测试：`tests/test_bmi088.c` 新增两个用例——孤立 18 dps 尖峰必须通过（drift 远小于 1），
  0→4.95 dps 的缓慢爬升必须被拒（spread 仅 4.95 低于 20，但 drift 2.5 超限）。
  `python tmp/run_tests_sandbox.py --imu`、`--attitude` 通过。
- 待实机：烧录后同时读 `drift_dps` 与 `spread_dps`。若 `drift_dps` 也超过 1 dps，说明窗口
  前后半段的平均角速度发生变化，可结合实机观察区分机械运动与采样异常。
