#include "motor.h"
#include "stm32f1xx_hal.h"

// 假设你用 TIM1 产生 PWM，需要根据实际情况修改
extern TIM_HandleTypeDef htim1; 

// 设置电机1 (PB6/7那边的电机) 的PWM和方向
// pwm_val 范围: -Output_Limit 到 +Output_Limit
void Motor1_SetSpeed(int pwm_val) {
    // 1. 处理方向
    if (pwm_val >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);   // AIN1 (示例)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // AIN2 (示例)
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        pwm_val = -pwm_val; // 取绝对值作为占空比
    }
    // 2. 输出PWM
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val); 
}

// 设置电机2 (PA6/7那边的电机)
void Motor2_SetSpeed(int pwm_val) {
    if (pwm_val >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // BIN1 (示例)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // BIN2 (示例)
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        pwm_val = -pwm_val;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_val);
}
void Motor_Init(void) {
    // 1. 开启 PWM 输出
    // 注意：如果你用的是 TIM1 的通道1和通道2，代码如下：
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    
    // 2. 初始化方向引脚 (可选，因为GPIO Init已经在main里做过了)
    // 这里为了保险，可以先把所有方向引脚拉低（停止状态）
    HAL_GPIO_WritePin(MOTOR1_IN1_PORT, MOTOR1_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR1_IN2_PORT, MOTOR1_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_IN1_PORT, MOTOR2_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_IN2_PORT, MOTOR2_IN2_PIN, GPIO_PIN_RESET);
}
