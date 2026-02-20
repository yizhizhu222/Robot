#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include "main.h" // 包含 HAL 库定义

// ==========================================
// 1. 物理参数配置 (根据你的实际车体修改)
// ==========================================
#define WHEEL_DIAMETER      0.085f   // 轮子直径 (米)
#define WHEEL_RADIUS        (WHEEL_DIAMETER / 2.0f)
#define TRACK_WIDTH         0.180f   // 轮距 (米) - 左右轮中心距离

// JGB37-520 电机参数 (11线霍尔, 减速比90, 4倍频)
// 11 * 90 * 4 = 3960
#define ENCODER_TOTAL_RESOLUTION 3960.0f 

#define CONTROL_PERIOD      0.05f    // 控制周期 50ms

// ==========================================
// 2. 对外函数接口
// ==========================================

// 初始化底盘 (PID, 定时器等)
void Chassis_Init(void);

// 设置目标速度 (供 protocol.c 调用)
void Chassis_Set_Velocity(float linear_x, float angular_z);

// 底盘控制任务实体 (供 FreeRTOS 任务调用)
void Chassis_Task_Loop(void *argument);

#endif /* __CHASSIS_H__ */
