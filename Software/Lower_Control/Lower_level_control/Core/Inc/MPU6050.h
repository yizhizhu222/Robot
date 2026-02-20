#ifndef __MPU6050_H
#define __MPU6050_H

/* ============================================= */
/* 这里必须包含 HAL 库，否则报错！        */
/* ============================================= */
#include "stm32f1xx_hal.h" 
// 如果你是 F4 系列，改成 #include "stm32f4xx_hal.h"

#include "main.h"

// MPU6050 I2C地址 (ADO引脚接地时)
#define MPU6050_ADDR 0xD0 

// 结构体：存储陀螺仪数据
typedef struct {
    float Gyro_Z;        // 当前 Z 轴角速度 (度/秒)
    float Gyro_Z_Offset; // 零点漂移值
} MPU_Data_t;

// 供外部调用的变量
extern MPU_Data_t MPU_Data;

// 函数声明
void MPU_Init(void);         // 初始化
void MPU_Calibrate(void);    // 零点校准 (需静止)
void MPU_Read_Gyro_Z(void);  // 读取数据

#endif
