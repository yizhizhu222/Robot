#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h> 
#include "yolo.h"

// ================= 核心参数调优区域 (新手最常改这里) =================
#define CENTER_X        160.0f    // 画面中心点 (320宽度的一半)
#define KP_TURN         0.01f     // 【转向灵敏度】数值越大，转弯越猛
#define MAX_TURN_SPEED  1.0f      // 【转速上限】防止机器人转得太快把自己甩飞
#define FORWARD_SPEED   0.2f      // 【前进速度】单位通常是 m/s
#define CONFIDENCE_MIN  0.45f     // 【置信度门槛】低于这个分数的物体会被忽略

// ================= 串口通信 (下位机协议) =================
/**
 * @brief 打开并配置串口 (与单片机通信)
 */
int open_serial(const char *device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) { perror("错误：无法打开串口"); return -1; }
    
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200); // 波特率 115200
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD); // 忽略控制线，开启读取
    options.c_cflag &= ~PARENB;          // 无校验
    options.c_cflag &= ~CSTOPB;          // 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;              // 8位数据
    
    // 原始模式 (Raw Mode)，不进行回显和换行处理
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

// 定义发送给机器人的数据结构
struct Velocity_Payload { 
    float linear_x;  // 线速度 (前后)
    float angular_z; // 角速度 (左右转向)
};

/**
 * @brief 按照通讯协议打包并发送速度指令
 */
void send_velocity_command(int fd, float linear_x, float angular_z) {
    uint8_t tx_buf[32]; 
    int idx = 0;
    Velocity_Payload payload = {linear_x, angular_z};
    uint8_t len = sizeof(payload);

    // 填充协议头
    tx_buf[idx++] = 0xAA; // 帧头 1
    tx_buf[idx++] = 0x55; // 帧头 2
    tx_buf[idx++] = 0x10; // 命令 ID: 设置速度
    tx_buf[idx++] = len;  // 数据长度
    
    // 拷贝速度数据
    memcpy(&tx_buf[idx], &payload, len);
    idx += len;

    // 计算校验和 (防止数据传错)
    uint8_t checksum = 0x10 + len;
    uint8_t *pData = (uint8_t*)&payload;
    for (int i = 0; i < len; i++) checksum += pData[i];
    tx_buf[idx++] = checksum;

    write(fd, tx_buf, idx);
}

// ================= 主循环 (大脑运行逻辑) =================
int main() {
    // 1. 初始化串口
    int serial_fd = open_serial("/dev/ttyAS5");
    if (serial_fd != -1) printf("✅ 串口已连接\n");

    // 2. 初始化摄像头 (设置分辨率为 320x240)
    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    if (!cap.isOpened()) { printf("❌ 无法打开摄像头\n"); return -1; }

    // 3. 加载 YOLO 模型
    Yolo yolo;
    if (yolo.load("/home/orangepi/my_robot/model/yolov8", 320) != 0) return -1;
    
    cv::Mat frame;
    std::vector<Object> objects;
    
    // --- 状态变量 (用于平滑控制) ---
    int lost_target_counter = 0; // 丢失计数器：防止目标闪烁导致车乱跳
    float last_v = 0.0f;
    float last_w = 0.0f;

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        objects.clear();
        yolo.detect(frame, objects); // 执行目标检测

        float v_x = 0.0f; // 本帧线速度
        float w_z = 0.0f; // 本帧角速度
        bool valid_target_found = false;

        // 策略 A：如果有看到物体
        if (!objects.empty()) {
            const auto& obj = objects[0]; // 只追踪看到的第一个物体
            
            if (obj.prob > CONFIDENCE_MIN) {
                valid_target_found = true;
                lost_target_counter = 0; // 看到东西了，重置丢失计时

                // --- 计算偏离距离 ---
                float cx = obj.rect.x + obj.rect.width / 2.0f; // 物体中心 X
                float error = cx - CENTER_X; // 物体相对于屏幕中心的偏离量

                // 只有偏离超过 20 像素才转向，防止原地细微抖动 (死区逻辑)
                if (abs(error) < 20.0f) error = 0.0f;

                // --- 比例控制 (P 控制) ---
                // 物体在左 (error<0) -> 需要左转 (w_z 为正)
                // 物体在右 (error>0) -> 需要右转 (w_z 为负)
                w_z = -error * KP_TURN; 

                // 限速保护：不要超过电机最大负荷
                if (w_z > MAX_TURN_SPEED) w_z = MAX_TURN_SPEED;
                if (w_z < -MAX_TURN_SPEED) w_z = -MAX_TURN_SPEED;

                v_x = FORWARD_SPEED; // 看到目标就匀速前进
                
                // 记录状态，用于丢失后的“惯性”维持
                last_v = v_x;
                last_w = w_z;

                printf("🎯 [追踪中] 偏差: %.1f | 转向速度: %.3f\n", error, w_z);
            }
        }

        // 策略 B：如果目标丢了 (防抖处理)
        if (!valid_target_found) {
            lost_target_counter++;
            
            // 如果丢失时间很短 (少于 5 帧，约 0.2 秒)
            // 机器人会按照上一帧的指令继续跑，避免因为检测断开而急刹车
            if (lost_target_counter < 5) {
                v_x = last_v;
                w_z = last_w;
                printf("⚠️ [目标丢失] 维持惯性... (%d/5)\n", lost_target_counter);
            } else {
                // 彻底丢了，安全停车
                v_x = 0.0f;
                w_z = 0.0f;
            }
        }

        // 4. 发送指令到电机控制板
        if (serial_fd != -1) {
            send_velocity_command(serial_fd, v_x, w_z);
        }
    }

    if (serial_fd != -1) close(serial_fd);
    return 0;
}
