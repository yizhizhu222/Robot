#include "pid.h"

// 初始化PID参数
void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out, float max_int) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->Output_Limit = max_out;
    pid->Integral_Limit = max_int;
    pid->Sum_Error = 0.0f;
    pid->Last_Error = 0.0f;
}

// 计算PID输出
float PID_Compute(PID_TypeDef *pid, float target, float actual) {
    pid->Target = target;
    pid->Actual = actual;
    pid->Error = pid->Target - pid->Actual;

    // 积分项（带限幅，防止积分饱和）
    pid->Sum_Error += pid->Error;
    if (pid->Sum_Error > pid->Integral_Limit) pid->Sum_Error = pid->Integral_Limit;
    if (pid->Sum_Error < -pid->Integral_Limit) pid->Sum_Error = -pid->Integral_Limit;

    // PID公式: Kp*Error + Ki*Sum + Kd*(Error - Last_Error)
    pid->Output = pid->Kp * pid->Error + 
                  pid->Ki * pid->Sum_Error + 
                  pid->Kd * (pid->Error - pid->Last_Error);

    // 更新上次误差
    pid->Last_Error = pid->Error;

    // 输出限幅（例如PWM不能超过7200或100）
    if (pid->Output > pid->Output_Limit) pid->Output = pid->Output_Limit;
    if (pid->Output < -pid->Output_Limit) pid->Output = -pid->Output_Limit;

    return pid->Output;
}
