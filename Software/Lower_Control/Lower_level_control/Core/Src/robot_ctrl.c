#include "robot_ctrl.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"

// --- 私有变量 (外部看不见) ---
static PID_TypeDef PID_Motor1, PID_Motor2;
static float Target_L = 0, Target_R = 0;
extern TIM_HandleTypeDef htim2; // 引用 main.c 里的定时器句柄

// --- 初始化函数 ---
void Robot_Init(void) {
    Encoder_Init();
    Motor_Init();
    // 初始化PID
    PID_Init(&PID_Motor1, 10.0f, 0.5f, 0.0f, 7200, 2000);
    PID_Init(&PID_Motor2, 10.0f, 0.5f, 0.0f, 7200, 2000);
    // 开启中断
    HAL_TIM_Base_Start_IT(&htim2);
}

// --- 所有的脏活累活都在这里 (中断回调) ---
// 只要把这个函数放在这里，HAL库会自动找到它并执行
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        int speed1 = Read_Encoder_Motor1();
        int speed2 = Read_Encoder_Motor2();
        
        float out1 = PID_Compute(&PID_Motor1, Target_L, speed1);
        float out2 = PID_Compute(&PID_Motor2, Target_R, speed2);
        
        Motor1_SetSpeed((int)out1);
        Motor2_SetSpeed((int)out2);
    }
}

// --- 简单的控制逻辑 ---
void Robot_SetVelocity(float left, float right) {
    Target_L = left;
    Target_R = right;
}

// --- 测试循环 ---
void Robot_Test_Loop(void) {
    Robot_SetVelocity(50, 50); // 前进
    HAL_Delay(2000);
    
    Robot_SetVelocity(0, 0);   // 停止
    HAL_Delay(1000);
    
    Robot_SetVelocity(-50, 50); // 左转
    HAL_Delay(2000);
}
