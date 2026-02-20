#include "MPU6050.h" // 此时这里面已经包含了 HAL 库，所以下面都安全了

// 引用外部的 I2C 句柄
// 只要上面的 MPU6050.h 包含了 HAL 库，这一行就不会再报错 #20
extern I2C_HandleTypeDef hi2c2;

MPU_Data_t MPU_Data = {0};

/**
  * @brief  初始化 MPU6050
  */
void MPU_Init(void) {
    uint8_t check = 0;
    uint8_t data = 0;

    // 1. 检查设备 ID (WHO_AM_I)
    // 此时 HAL_I2C_Mem_Read 应该能被正常识别
    if(HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x75, 1, &check, 1, 100) != HAL_OK) {
        // 如果读取失败，可以在这里加个断点或者错误处理
        return; 
    }

    if (check == 0x68) { 
        // 2. 唤醒传感器 (PWR_MGMT_1 写 0)
        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x6B, 1, &data, 1, 100);

        // 3. 配置陀螺仪量程 (GYRO_CONFIG 0x1B)
        data = 0x18; // +/- 2000 dps
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, 0x1B, 1, &data, 1, 100);
    }
}

/**
  * @brief  读取 Z 轴角速度并转换为度/秒
  */
#define ALPHA 0.2f 

void MPU_Read_Gyro_Z(void) {
    uint8_t raw_data[2];
    int16_t raw_val;

    // 1. 读取原始数据
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x47, 1, raw_data, 2, 50);
    raw_val = (int16_t)(raw_data[0] << 8 | raw_data[1]);

    // 2. 转换物理量并去皮
    float new_val = (float)raw_val / 16.4f;
    new_val -= MPU_Data.Gyro_Z_Offset;

    // 3. 死区处理 (屏蔽小噪声)
    if (new_val > -0.5f && new_val < 0.5f) {
        new_val = 0.0f;
    }

    // 4. 一阶低通滤波 (让数据更柔顺)
    // 公式：输出 = 上次输出 * (1-系数) + 本次输入 * 系数
    MPU_Data.Gyro_Z = MPU_Data.Gyro_Z * (1.0f - ALPHA) + new_val * ALPHA;
}
/**
  * @brief  零点校准 (开机时必须静止)
  */
void MPU_Calibrate(void) {
    float sum = 0;
    // 循环读取 100 次
    for(int i=0; i<100; i++) {
        uint8_t raw_data[2];
        int16_t raw_val;
        HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, 0x47, 1, raw_data, 2, 50);
        raw_val = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        
        sum += (float)raw_val;
        HAL_Delay(5); 
    }
    // 计算平均值
    MPU_Data.Gyro_Z_Offset = (sum / 100.0f) / 16.4f;
}
