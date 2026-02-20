#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

// ==========================================
// 数据结构定义
// ==========================================

typedef struct {
    float x;      // 全局 X 坐标 (米)
    float y;      // 全局 Y 坐标 (米)
    float theta;  // 全局朝向角度 (弧度 Radian), 范围 -PI ~ PI
} Odometry_Pose_t;

// ==========================================
// 函数接口
// ==========================================

// 初始化里程计 (归零)
void Odometry_Init(void);

/**
 * @brief  里程计更新 (核心数学逻辑)
 * @param  delta_pulse_L: 这一瞬间左轮走了多少脉冲 (Read_Encoder读出来的原值)
 * @param  delta_pulse_R: 这一瞬间右轮走了多少脉冲
 * @param  dt:            时间间隔 (秒)，通常是 0.05
 */
void Odometry_Update(float delta_pulse_L, float delta_pulse_R, float dt);

// 获取当前的全局坐标 (供 Protocol 发送用)
Odometry_Pose_t Odometry_Get_Pose(void);

#endif /* __ODOMETRY_H__ */
