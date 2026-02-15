#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

// ================= 引脚配置区 (请根据你的实际连线修改) =================

// 电机1 (PB6/PB7那边) 的控制引脚 (TB6612 AIN1/AIN2)
#define MOTOR1_IN1_PIN   GPIO_PIN_4   // 举例：PA4
#define MOTOR1_IN1_PORT  GPIOA
#define MOTOR1_IN2_PIN   GPIO_PIN_5   // 举例：PA5
#define MOTOR1_IN2_PORT  GPIOA

// 电机2 (PA6/PA7那边) 的控制引脚 (TB6612 BIN1/BIN2)
#define MOTOR2_IN1_PIN   GPIO_PIN_2   // 举例：PA2
#define MOTOR2_IN2_PIN   GPIO_PIN_3   // 举例：PA3
#define MOTOR2_IN2_PORT  GPIOA
#define MOTOR2_IN1_PORT  GPIOA

// ===================================================================

// 函数声明
void Motor_Init(void);               // 电机初始化（开启PWM等）
void Motor1_SetSpeed(int pwm_val);   // 设置电机1速度
void Motor2_SetSpeed(int pwm_val);   // 设置电机2速度

#endif
