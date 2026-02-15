#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h" // 一定要包含这个，因为需要用到TIM_HandleTypeDef

// 函数声明
void Encoder_Init(void);         // 初始化编码器
int Read_Encoder_Motor1(void);   // 读取电机1速度 (PB6/PB7)
int Read_Encoder_Motor2(void);   // 读取电机2速度 (PA6/PA7)

#endif
