#include "motor.h"
#include "stm32f1xx_hal.h"
#include "math.h"

extern TIM_HandleTypeDef htim2; 

// 设置电机1 (PB6/7那边的电机) 的PWM和方向
// pwm_val 范围: -Output_Limit 到 +Output_Limit
void Motor1_SetSpeed(int pwm_val) {
    // 1. 处理方向
    if (pwm_val >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // AIN1 (示例)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // AIN2 (示例)
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        pwm_val = -pwm_val; // 取绝对值作为占空比
    }
    // 2. 输出PWM
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_val); 
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
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_val);
}
void Motor_Init(void) {
    // 1. 开启 PWM 输出
    // 注意：如果你用的是 TIM2 的通道1和通道2，代码如下：
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    
    // 2. 初始化方向引脚 (可选，因为GPIO Init已经在main里做过了)
    // 这里为了保险，可以先把所有方向引脚拉低（停止状态）
    HAL_GPIO_WritePin(MOTOR1_IN1_PORT, MOTOR1_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR1_IN2_PORT, MOTOR1_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_IN1_PORT, MOTOR2_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR2_IN2_PORT, MOTOR2_IN2_PIN, GPIO_PIN_RESET);
}
// 参数：motor_id(1=左, 2=右), pid_output(PID算出来的结果)
void Motor_Set_Output(int motor_id, float pid_output) 
{
    // 1. 获取 PWM 的绝对值 (负数变正数，因为占空比不能是负的)
    int pwm_val = (int)fabs(pid_output);

    // 2. 这里的引脚请根据你实际接线修改！！！
    // 假设：IN1/IN2 是左轮，IN3/IN4 是右轮
    if (pid_output >= 0) {
        // --- 正转逻辑 ---
        if(motor_id == 1) { // 左轮正转
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // 改你的引脚
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); 
        } else {            // 右轮正转
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // 改你的引脚
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
    } else {
        // --- 反转逻辑 (PID算出来是负数) ---
        if(motor_id == 1) { // 左轮反转
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // 改你的引脚
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        } else {            // 右轮反转
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // 改你的引脚
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        }
    }

    // 3. 发送 PWM 给定时器 (假设电机用 TIM1 的 CH1 和 CH4)
    if(motor_id == 1) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_val);
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_val);
    }
}
