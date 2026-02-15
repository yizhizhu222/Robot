#ifndef __PID_H
#define __PID_H

typedef struct {
    float Kp, Ki, Kd;       // PID参数
    float Target;           // 目标速度
    float Actual;           // 实际速度
    float Error;            // 当前误差
    float Last_Error;       // 上次误差
    float Sum_Error;        // 累计误差（用于积分项，需限幅）
    float Output;           // 输出PWM值
    float Output_Limit;     // 输出限幅（例如PWM最大值）
    float Integral_Limit;   // 积分限幅
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out, float max_int);
float PID_Compute(PID_TypeDef *pid, float target, float actual);

#endif
