# 麦轮底盘解算

左右轮中心距离 W=0.45 m，前后轮中心距离 L=0.55 m。
两者都是完整轮距，半宽为 0.225 m，半长为 0.275 m：

```text
k = L/2 + W/2 = 0.50 m
```

默认轮半径 r=0.07786 m、减速比 G=14（转子转速/轮转速）沿用考核文档。
配置位于 `MECANUM_DEFAULT_CONFIG`，可复制成可修改的 `Mecanum_Config`。
本模块无 HAL 依赖，已加入固件 CMake，但尚未接入运动控制主循环。

## 坐标与轮序

底盘中心为原点，x 向前、y 向左、z 向上；wz 为俯视逆时针正方向。
vx/vy 单位 m/s，wz 单位 rad/s，输入必须已经转换到底盘坐标系。
本实现假定标准 45° X 型麦轮安装，不适用于任意滚子角度或其他装配布局。

| 数组索引 | 位置 | 电调 ID | 轮中心 (x,y)，m |
| --- | --- | --- | --- |
| 0 / MECANUM_FL | 左前 | 1 | (+0.275,+0.225) |
| 1 / MECANUM_RL | 左后 | 2 | (-0.275,+0.225) |
| 2 / MECANUM_RR | 右后 | 3 | (-0.275,-0.225) |
| 3 / MECANUM_FR | 右前 | 4 | (+0.275,-0.225) |

## 公式

设各轮几何正方向均为推动底盘向前的转动方向，轮缘速度为：

```text
u_FL = vx - vy - 0.50*wz
u_RL = vx + vy - 0.50*wz
u_RR = vx - vy + 0.50*wz
u_FR = vx + vy + 0.50*wz

motor_rpm[i] = direction[i] * u[i] * 60 * G / (2*pi*r)
```

符号约定与 [WPILib 麦轮运动学](https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/mecanum-drive-kinematics.html)
一致；本项目使用的数组顺序是左前、左后、右后、右前。

`motor_direction[i]` 只允许 +1 或 -1：若电机反馈的正 rpm 对应此轮向前滚动，
填 +1，否则填 -1。默认全 +1 仅为数学约定，不能视作真实安装方向。
上板架空后逐轮核对，不要凭左右位置直接猜符号。逆解算会转换成电机反馈的
符号坐标，因此速度 PID 比较的是目标转子 rpm 和原始反馈 rpm，勿再重复反向。

正解算先令 `u[i] = direction[i]*motor_rpm[i]*2*pi*r/(60*G)`：

```text
vx = ( u_FL + u_RL + u_RR + u_FR)/4
vy = (-u_FL + u_RL - u_RR + u_FR)/4
wz = (-u_FL - u_RL + u_RR + u_FR)/(4*k)
```

正解算返回无滑移模型下的速度估计，不是融合里程计；麦轮打滑会造成误差。

## 使用示例

```c
#include "mecanum.h"

Mecanum_Config config = MECANUM_DEFAULT_CONFIG;
config.max_motor_rpm = 1000.0f; // 示例限速，实际值需按底盘整定
// config.motor_direction[0..3] 必须根据实车测定。
Mecanum_Velocity command = {.vx = 0.2f, .vy = 0.0f, .wz = 0.0f};
Mecanum_Wheels target;
if (Mecanum_Inverse(&config, &command, &target)) {
    // target.motor_rpm[i] -> 第 i 个速度 PID 的目标
    // 电机反馈 speed_rpm -> PID 测量值
    // PID 输出电流 -> M3508_CAN_SendCurrents()
}
```

`max_motor_rpm=0` 表示不做速度限幅；正值启用统一比例缩放：
`scale=min(1,max_motor_rpm/max(abs(rpm[i])))`。输出 `scale` 表明实际缩放比例。
不要逐轮独立截断目标，否则会改变平移与旋转的组合。
正解算始终使用实际输入 rpm，不因目标限速配置裁剪反馈。

非法参数、空指针、NaN/Inf 或中间计算溢出时返回 false，非空输出清零。
此模块不做急停/解锁判断、遥控器映射、云台坐标变换或 PID 控制；主循环的
零电流行为未改变。后续接入时仍需检查遥控器在线、电机反馈新鲜度和解锁状态。

## 测试

```powershell
python tests/run_motor_tests.py --chassis
```

覆盖零速、前进/横移/旋转方向、轮速到转子 rpm 换算、斜向运动、符号映射、
正逆解算一致性、统一比例限速及异常输入。Debug 编译与该模拟测试通过；
轮布局、轮径、减速比和电机方向仍需硬件核对。
