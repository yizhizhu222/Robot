/**文件名：protocol.h
*描述：STM32与上位机Orangepi的通信协议定义文件
*作者：伊之猪
*日期：2026-1-15
*/
#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__
#include <stdint.h>

// ==========================
// 1. 基础常量定义 (Magic Number)
// ==========================
#define FRAME_HEAD1    0xAA
#define FRAME_HEAD2    0x55
#define MAX_PAYLOAD    32      // 最大允许数据长度

// ==========================
// 2. 功能码列表 (Command ID)
// ==========================
typedef enum {
    CMD_HEARTBEAT    = 0x00,   // 心跳
    CMD_SET_VEL      = 0x10,   // 设置速度
    CMD_SET_MODE     = 0x20,   // 设置模式
    CMD_REPORT_ODOM  = 0x11    // 里程计回传
} CommandID_t;

// ==========================
// 3. 数据载荷结构体 (Payloads)
// ==========================
// 注意：必须使用 pack(1) 强制 1 字节对齐，严禁编译器补 0

#pragma pack(1)

// [ID: 0x10] 速度控制数据包
typedef struct {
    float linear_x;    // 线速度 m/s
    float angular_z;   // 角速度 rad/s
} Payload_Velocity_t;

// [ID: 0x20] 视觉模式数据包
typedef struct {
    uint8_t mode;      // 0:Idle, 1:Track, 2:Scan
    uint8_t strategy;  // 策略参数
} Payload_Mode_t;

// [ID: 0x11] 里程计回传数据包
typedef struct {
    float x;           // X坐标
    float y;           // Y坐标
} Payload_Odom_t;

#pragma pack() // 恢复默认对齐
// 发送相关函数
void Protocol_Send_Heartbeat(void);
void Protocol_Send_Odom(float x, float y);

// 接收相关函数
void Protocol_Decode(uint8_t byte);

// 回调函数 (可以在 main.c 里重写)
void On_Receive_Velocity(float x, float z);
void On_Receive_Mode(uint8_t mode);
#endif // __PROTOCOL_H__
