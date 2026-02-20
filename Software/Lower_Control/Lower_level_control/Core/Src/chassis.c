#include "chassis.h"
#include "pid.h"
#include "motor.h"
#include "encoder.h"
#include "cmsis_os.h" // FreeRTOS API
#include <math.h>

// --- 私有变量 ---
static PID_TypeDef PID_MotorL;
static PID_TypeDef PID_MotorR;
static volatile uint32_t Last_Cmd_Tick = 0;

// 全局目标变量 (volatile 防止被优化)
static volatile float Target_Linear_X = 0.0f;
static volatile float Target_Angular_Z = 0.0f;

// 引入外部句柄 (如果 motor.c 或 encoder.c 里需要用到)
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

// ==========================================
// 函数实现
// ==========================================

/**
 * @brief  初始化底盘所需的 PID 和 硬件
 */
void Chassis_Init(void)
{
    // 1. 确保底层硬件开启 (虽然 main.c 可能开过了，这里再保险一次)
    Motor_Init();    // 开启 PWM
    Encoder_Init();  // 开启编码器计数

    // 2. 初始化 PID 参数
    // Kp=5.0, Ki=0.1, Kd=0.0 是经验值，需根据实际响应调整
    // Output_Limit = 7199 (定时器ARR), Integral_Limit = 3000
    PID_Init(&PID_MotorL, 30.0f, 0.5f, 0.0f, 7199.0f, 3000.0f);
	PID_Init(&PID_MotorR, 30.0f, 0.5f, 0.0f, 7199.0f, 3000.0f);
}

/**
 * @brief  设置底盘目标速度 (线程安全)
 * @param  linear_x:  线速度 m/s
 * @param  angular_z: 角速度 rad/s
 */
void Chassis_Set_Velocity(float linear_x, float angular_z)
{
    // 简单赋值，50ms周期内这算是原子操作
    Target_Linear_X = linear_x;
    Target_Angular_Z = angular_z;
	
	Last_Cmd_Tick = HAL_GetTick();
}

static void Safety_Watchdog_Check(void)
{
    // 获取当前时间
    uint32_t current_tick = HAL_GetTick();

    // 计算距离上次收到指令过了多久
    // 注意：HAL_GetTick 是 uint32，溢出回绕也不影响减法结果
    if ((current_tick - Last_Cmd_Tick) > 500) // 阈值：500ms
    {
        // ！！超时了！！强制刹车！！
        Target_Linear_X = 0.0f;
        Target_Angular_Z = 0.0f;
        
        // 可选：在这里让蜂鸣器叫两声报警，或者亮个红灯
        // HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); 
    }
}

/**
 * @brief  底盘控制主循环 (这就是你的 FreeRTOS 任务函数)
 */
void Chassis_Task_Loop(void *argument)
{
    // 任务开始前先初始化
    Chassis_Init();

    // 临时变量
    float v_left_mps, v_right_mps;
    float target_pulse_L, target_pulse_R;
    float current_pulse_L, current_pulse_R;
    float pwm_L, pwm_R;

    const float PI = 3.1415926f;
    const float meters_per_round = WHEEL_DIAMETER * PI; // 轮子周长

    for(;;)
    {
		Safety_Watchdog_Check();
        // --- 1. 运动学逆解算 (Inverse Kinematics) ---
        // 将 整车速度 分解为 左右轮速度
        // v_L = v - w * (L/2)
        // v_R = v + w * (L/2)
        v_left_mps  = Target_Linear_X - (Target_Angular_Z * TRACK_WIDTH / 2.0f);
        v_right_mps = Target_Linear_X + (Target_Angular_Z * TRACK_WIDTH / 2.0f);

        // --- 2. 物理量转脉冲 (Unit Conversion) ---
        // 目标脉冲 = (目标速度m/s * 时间s * 总分辨率) / 轮周长m
        target_pulse_L = (v_left_mps * CONTROL_PERIOD * ENCODER_TOTAL_RESOLUTION) / meters_per_round;
        target_pulse_R = (v_right_mps * CONTROL_PERIOD * ENCODER_TOTAL_RESOLUTION) / meters_per_round;

        // --- 3. 获取反馈 (Feedback) ---
        current_pulse_L = (float)Read_Encoder_Motor1();
        current_pulse_R = (float)Read_Encoder_Motor2();

        // --- 4. PID 计算 (Control Loop) ---
        pwm_L = PID_Compute(&PID_MotorL, target_pulse_L, current_pulse_L);
        pwm_R = PID_Compute(&PID_MotorR, target_pulse_R, current_pulse_R);

        // --- 5. 执行输出 (Actuation) ---
        // 注意：Motor_Set_Output 需要你在 motor.c 里实现，或者直接调用 Motor_SetSpeed
        // 这里假设 motor.c 里有 Motor1_SetSpeed 和 Motor2_SetSpeed
        Motor1_SetSpeed((int)pwm_L);
        Motor2_SetSpeed((int)pwm_R);

        // --- 6. 周期性延时 ---
        osDelay(50); // 50ms = 20Hz
    }
}/*
#define FILTER_N 10  // 采样最近 10 次数据求平均

// 右轮专用的滤波缓存
static int filter_buf_R[FILTER_N] = {0};
static int filter_idx_R = 0;

static int Moving_Average_Filter(int new_val) {
    int sum = 0;
    
    // 1. 存入新数据
    filter_buf_R[filter_idx_R] = new_val;
    filter_idx_R++;
    if (filter_idx_R >= FILTER_N) filter_idx_R = 0; // 循环存
    
    // 2. 算平均
    for(int i=0; i<FILTER_N; i++) {
        sum += filter_buf_R[i];
    }
    return sum / FILTER_N;
}
*/

