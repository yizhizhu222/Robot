#include "stm32f1xx_hal.h" // 根据你的芯片型号调整

// 外部引用的定时器句柄（在main.c或stm32f1xx_hal_msp.c中定义）
extern TIM_HandleTypeDef htim3; // 对应 PA6, PA7
extern TIM_HandleTypeDef htim4; // 对应 PB6, PB7

// 读取速度：读取计数器值 -> 清零 -> 返回
// 这是一个必须周期性调用（例如每10ms）的函数
int Read_Encoder_Motor1(void) {
    int speed = (short)__HAL_TIM_GET_COUNTER(&htim4); // 强制转换为short处理负数
    __HAL_TIM_SET_COUNTER(&htim4, 0); // 清零，准备下一次计数
    return speed;
}

int Read_Encoder_Motor2(void) {
    int speed = (short)__HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return speed;
}

void Encoder_Init(void) {
    // 开启定时器的编码器模式
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}
