# PID 控制器

`pid.c/pid.h` 提供无 HAL 依赖的 C11 位置式 PID，可从 C++ 调用。
每个轮子使用一个独立 `PID_Controller`，同一实例只能由一个控制任务更新。

计算方式：

```text
e = target - measurement
P = kp * e
I_candidate = clamp(I + ki * e * dt, ±integral_limit)
raw_derivative = -(measurement - previous_measurement) / dt
alpha = dt / (derivative_tau + dt)
filtered_derivative += alpha * (raw_derivative - filtered_derivative)
D = kd * filtered_derivative
output = clamp(P + I + D, ±output_limit)
```

首次更新的 D 为零。微分作用于反馈，目标阶跃不会直接产生微分冲击。
`derivative_tau=0` 表示不滤波；`kd=0` 禁用微分。
候选输出饱和且积分增量继续推向该饱和方向时，拒绝本次积分增量；
积分项另有独立限幅。`integral_limit=0` 禁用积分贡献。

## 接口

- `PID_Init()`：校验并复制参数，清空历史；失败后控制器禁用。
- `PID_Update()`：输入目标、反馈、实际周期（秒），经输出指针返回控制量。
- `PID_Reset()`：保留参数，清空积分、微分历史和输出。

增益必须有限且非负，输出限幅必须大于零，积分限幅/滤波时间常数不能为负。
错误输入、NaN/Inf、非正周期、周期大于 `max_dt` 或计算溢出时，Update
返回 false，输出零，并清空历史。控制端应检查返回值；掉线/急停时也要 Reset。
参数调整需重新 Init，不要直接写入控制器内部字段；输出指针使用单独的 float
变量，不要指向控制器的积分或配置字段。

## 轮速环调用示例

以下参数仅展示字段和单位；增益全零，尚未完成实车整定。

```c
#include "pid.h"

static PID_Controller wheel_pid[4];
const PID_Config config = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .output_limit = 1000.0f,  // 示例电流指令上限，仍需按实车设置
    .integral_limit = 200.0f, // I 项的输出上限，不是误差累积值上限
    .derivative_tau = 0.01f,  // 秒
    .max_dt = 0.02f          // 示例：超出 20 ms 的调度间隔拒绝计算
};

// 初始化时：逐个检查 PID_Init(&wheel_pid[i], &config) 的返回值。
// 进入控制周期后，在已解锁、遥控器在线、电机反馈新鲜的前提下：
float current_raw = 0.0f;
bool ok = PID_Update(&wheel_pid[0], target_rotor_rpm,
                     measured_rotor_rpm, actual_dt_seconds, &current_raw);
// ok 为 false 时保持零指令；状态机决定是否重新解锁。
```

M3508 反馈转速是转子 rpm，目标也必须用同样单位。若输出直接交给
`M3508_CAN_SendCurrents()`，输出单位就是电流协议原始值，最后再转换为
`int16_t`，且 `output_limit` 不应超过 16384；这是协议上限，不是调试推荐值。
`ki` 和 `kd` 采用含真实 dt 的连续时间定义，不可直接照搬省略 dt 的 PID 参数。
主循环当前仍每 10 ms 发零电流；这里只把模块加入 CMake，尚未接入轮速闭环。

先完成方向核对、遥控器急停和重新解锁逻辑，再在架空底盘上小电流调参。
通常先调 P，再按静差需要加 I；速度反馈噪声较大时先保持 D=0。
当 CAN 发送失败、控制禁用或反馈过期时，调用层应停止输出并清空对应 PID。
算法只考虑自身 output_limit；若后级另有更小的功率/电流限制，需要在控制层
同步限制或增加针对执行器饱和的反馈处理。

## 验证

在仓库根目录运行（依赖 Arm GNU 工具链和 Python `unicorn==2.1.4`）：

```powershell
python tests/run_motor_tests.py --pid
```

测试包含 P/I/D 数值、实际 dt 缩放、双向限幅、积分抗饱和、目标阶跃、
微分滤波、独立实例、复位、非法输入/溢出，以及一阶模拟对象闭环收敛。
模拟对象不代表实际底盘，实车增益和周期仍需整定。
