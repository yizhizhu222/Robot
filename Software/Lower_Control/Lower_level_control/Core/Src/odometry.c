#include "odometry.h"
#include "kinematics.h" // 需要用到轮径和轮距参数
#include <math.h>       // 需要 cos, sin

// --- 私有全局变量 ---
static Odometry_Pose_t Global_Pose; // 机器人在世界坐标系下的位置

/**
 * @brief 初始化里程计
 */
void Odometry_Init(void) {
    Global_Pose.x = 0.0f;
    Global_Pose.y = 0.0f;
    Global_Pose.theta = 0.0f;
}

/**
 * @brief 更新里程计 (Dead Reckoning)
 */
void Odometry_Update(float delta_pulse_L, float delta_pulse_R, float dt) {
    // 1. 将 脉冲增量 转换为 距离增量 (米)
    // 公式: 距离 = (脉冲 / 总分辨率) * 轮周长
    float dist_L = (delta_pulse_L / ENCODER_RESOLUTION) * WHEEL_CIRCUMFERENCE;
    float dist_R = (delta_pulse_R / ENCODER_RESOLUTION) * WHEEL_CIRCUMFERENCE;

    // 2. 计算中心移动距离 (Center Delta Distance)
    // 车体中心走的距离是左右轮的平均值
    float delta_dist = (dist_L + dist_R) / 2.0f;

    // 3. 计算角度变化量 (Delta Theta)
    // 差速模型: 角度变化 = (右轮距离 - 左轮距离) / 轮距
    // 注意：这里假设 逆时针为正 (符合右手定则)
    // 如果你的车发现转弯方向反了，把 (dist_R - dist_L) 改成 (dist_L - dist_R)
    float delta_theta = (dist_R - dist_L) / TRACK_WIDTH;

    // 4. 更新全局角度 (积分)
    Global_Pose.theta += delta_theta;

    // --- 角度归一化 (保持在 -PI 到 PI 之间) ---
    // 这一步对于长期运行很重要，防止角度无限增大溢出
    if (Global_Pose.theta > 3.1415926f) {
        Global_Pose.theta -= 2 * 3.1415926f;
    } else if (Global_Pose.theta < -3.1415926f) {
        Global_Pose.theta += 2 * 3.1415926f;
    }

    // 5. 更新全局坐标 (分解到 X, Y 轴)
    // dx = d * cos(theta)
    // dy = d * sin(theta)
    // 注意：这里用的是更新后的 theta，也可以用 (old_theta + new_theta)/2 提高精度，但简单用即可
    Global_Pose.x += delta_dist * cosf(Global_Pose.theta);
    Global_Pose.y += delta_dist * sinf(Global_Pose.theta);
}

/**
 * @brief 获取当前坐标
 */
Odometry_Pose_t Odometry_Get_Pose(void) {
    return Global_Pose;
}
