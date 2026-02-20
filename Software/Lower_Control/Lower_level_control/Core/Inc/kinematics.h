#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <stdint.h>

// ==========================================
// 1. 物理参数配置 (拿尺子量准一点！)
// ==========================================
// 轮距 (Track Width): 两个驱动轮接触地点之间的距离
// 单位: 米
#define TRACK_WIDTH     0.180f 

// 轮径 (Wheel Diameter): 驱动轮的直径
// 单位: 米
#define WHEEL_DIAMETER  0.085f 
#define WHEEL_RADIUS    (WHEEL_DIAMETER / 2.0f)
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * 3.1415926f)

// 减速比和编码器线数 (JGB37-520)
// 11线霍尔 * 90减速比 * 4倍频 = 3960
#define ENCODER_RESOLUTION 3960.0f 

// ==========================================
// 2. 数据结构定义
// ==========================================

// 机器人整体速度 (Robot Frame)
typedef struct {
    float linear_x;  // 线速度 m/s (前进后退)
    float angular_z; // 角速度 rad/s (左转右转)
} Kinematics_Twist_t;

// 左右轮速度 (Motor Frame)
typedef struct {
    float left_mps;  // 左轮线速度 m/s
    float right_mps; // 右轮线速度 m/s
} Kinematics_Motor_Speed_t;

// 左右轮目标脉冲 (用于PID)
typedef struct {
    float left_ticks;  // 左轮目标脉冲数/dt
    float right_ticks; // 右轮目标脉冲数/dt
} Kinematics_Motor_Pulse_t;

// ==========================================
// 3. 函数接口
// ==========================================

// 逆运动学: (v, w) -> (v_L, v_R)
Kinematics_Motor_Speed_t Kinematics_Inverse(float linear_x, float angular_z);

// 速度转脉冲: (m/s) -> (Ticks/dt)
Kinematics_Motor_Pulse_t Kinematics_SpeedToPulse(Kinematics_Motor_Speed_t speed_mps, float dt);

#endif /* __KINEMATICS_H__ */
