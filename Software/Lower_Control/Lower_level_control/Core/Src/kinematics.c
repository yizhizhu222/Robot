#include "kinematics.h"

/**
  * @brief  逆运动学解算 (Inverse Kinematics)
  * 将 整体期望速度 分解为 左右轮期望速度
  * @param  linear_x:  目标线速度 (m/s)
  * @param  angular_z: 目标角速度 (rad/s)
  * @return 包含左右轮速度的结构体
  */
Kinematics_Motor_Speed_t Kinematics_Inverse(float linear_x, float angular_z)
{
    Kinematics_Motor_Speed_t output;

    // 核心公式：差速驱动模型
    // v_left  = v - (w * L / 2)
    // v_right = v + (w * L / 2)
    // L 是轮距 (TRACK_WIDTH)
    
    float tangent_vel = angular_z * (TRACK_WIDTH / 2.0f);

    output.left_mps  = linear_x - tangent_vel;
    output.right_mps = linear_x + tangent_vel;

    return output;
}

/**
  * @brief  物理速度转编码器脉冲 (Unit Conversion)
  * 将 m/s 转换为 一个控制周期内的脉冲数
  * @param  speed_mps: 左右轮线速度结构体
  * @param  dt:        控制周期 (秒)，例如 0.05
  * @return 包含左右轮脉冲数的结构体
  */
Kinematics_Motor_Pulse_t Kinematics_SpeedToPulse(Kinematics_Motor_Speed_t speed_mps, float dt)
{
    Kinematics_Motor_Pulse_t output;

    // 逻辑：
    // 1. 这一段时间(dt)内应该跑多少米?  dist = speed * dt
    // 2. 跑这些米需要转多少圈?        rounds = dist / 周长
    // 3. 转这些圈需要多少脉冲?        ticks = rounds * 一圈总脉冲

    // 合并公式: ticks = (speed * dt * total_res) / circumference
    
    float coef = (dt * ENCODER_RESOLUTION) / WHEEL_CIRCUMFERENCE;

    output.left_ticks  = speed_mps.left_mps  * coef;
    output.right_ticks = speed_mps.right_mps * coef;

    return output;
}
