#include "protocol.h"
#include "main.h"  // 必须包含这个，因为要用到 huart1 (或其他串口句柄)
#include <string.h> // 用于 memcpy

// ============================================================
// 0. 配置区域 (根据你的实际硬件修改)
// ============================================================
// 假设你使用串口1跟香橙派通信
extern UART_HandleTypeDef huart1;
#define PROTOCOL_UART &huart1

// ============================================================
// 1. 内部辅助函数 (发送底层)
// ============================================================
/**
 * @brief  组包并发送一帧数据
 * @param  cmd_id: 功能码
 * @param  pData:  数据载荷指针
 * @param  len:    数据载荷长度
 */
static void Protocol_Send_Frame(uint8_t cmd_id, uint8_t *pData, uint8_t len)
{
    // 缓冲区大小 = 头(2) + ID(1) + Len(1) + Payload(MAX) + Sum(1)
    static uint8_t tx_buf[5 + MAX_PAYLOAD]; 
    
    // 1. 填充帧头
    tx_buf[0] = FRAME_HEAD1;
    tx_buf[1] = FRAME_HEAD2;
    tx_buf[2] = cmd_id;
    tx_buf[3] = len;

    // 2. 填充数据载荷
    if (len > 0 && pData != NULL) {
        memcpy(&tx_buf[4], pData, len);
    }

    // 3. 计算校验和 (ID + Len + Payload)
    uint8_t checksum = cmd_id + len;
    for (int i = 0; i < len; i++) {
        checksum += pData[i];
    }
    tx_buf[4 + len] = checksum;

    // 4. 发送 (阻塞式发送，简单可靠)
    // 发送长度 = 4字节头信息 + 数据长度 + 1字节校验
    HAL_UART_Transmit(PROTOCOL_UART, tx_buf, 5 + len, 100);
}

// ============================================================
// 2. 发送接口实现 (给 main.c 调用的)
// ============================================================

// 发送心跳包
void Protocol_Send_Heartbeat(void)
{
    // 心跳包没有载荷，长度为0
    Protocol_Send_Frame(CMD_HEARTBEAT, NULL, 0);
}

// 发送里程计数据 (STM32 -> 上位机)
void Protocol_Send_Odom(float x, float y)
{
    Payload_Odom_t odom;
    odom.x = x;
    odom.y = y;
    
    // 利用 sizeof 自动计算长度，避免写死数字
    Protocol_Send_Frame(CMD_REPORT_ODOM, (uint8_t*)&odom, sizeof(Payload_Odom_t));
}

// ============================================================
// 3. 接收处理逻辑 (核心状态机)
// ============================================================

// 预留的弱函数，用户可以在 main.c 里重写这些函数来实现具体逻辑
__weak void On_Receive_Velocity(float x, float z) {}
__weak void On_Receive_Mode(uint8_t mode) {}

// 内部函数：处理校验通过的完整数据包
static void Protocol_Handle_Packet(uint8_t cmd_id, uint8_t *data, uint8_t len)
{
    switch (cmd_id)
    {
        case CMD_SET_VEL:
            if (len == sizeof(Payload_Velocity_t)) {
                Payload_Velocity_t *vel = (Payload_Velocity_t*)data;
                // 调用回调函数执行动作
                On_Receive_Velocity(vel->linear_x, vel->angular_z);
            }
            break;

        case CMD_SET_MODE:
            if (len == sizeof(Payload_Mode_t)) {
                Payload_Mode_t *mode_pkt = (Payload_Mode_t*)data;
                On_Receive_Mode(mode_pkt->mode);
            }
            break;
            
        // 可以在这里增加更多指令处理...
        default:
            break;
    }
}

// 公共函数：放入串口中断中调用，逐字节解析
void Protocol_Decode(uint8_t byte)
{
    static uint8_t state = 0;
    static uint8_t cmd_id = 0;
    static uint8_t len = 0;
    static uint8_t count = 0;
    static uint8_t rx_buf[MAX_PAYLOAD];
    static uint8_t checksum = 0;

    switch (state)
    {
        case 0: // 等待帧头1 (0xAA)
            if (byte == FRAME_HEAD1) state = 1;
            break;

        case 1: // 等待帧头2 (0x55)
            if (byte == FRAME_HEAD2) state = 2;
            else state = 0; // 错误，重置
            break;

        case 2: // 读取功能ID
            cmd_id = byte;
            checksum = byte; // 开始计算校验
            state = 3;
            break;

        case 3: // 读取数据长度
            len = byte;
            checksum += byte;
            if (len > MAX_PAYLOAD) { // 长度异常保护
                state = 0; 
            } else if (len == 0) {
                state = 5; // 无数据，直接跳去校验
            } else {
                count = 0;
                state = 4;
            }
            break;

        case 4: // 读取数据载荷
            rx_buf[count++] = byte;
            checksum += byte;
            if (count >= len) state = 5;
            break;

        case 5: // 校验位对比
            if (byte == checksum) {
                // 校验成功，执行处理函数
                Protocol_Handle_Packet(cmd_id, rx_buf, len);
            }
            // 无论成功失败，都回到初始状态
            state = 0;
            break;
            
        default:
            state = 0;
            break;
    }
}
