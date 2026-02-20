#ifndef YOLO_H
#define YOLO_H

#include <opencv2/opencv.hpp>
#include <ncnn/net.h>
#include <algorithm>
#include <vector>

// 定义检测到的目标结构体
struct Object {
    cv::Rect_<float> rect; // 目标矩形框 (x, y, w, h)
    int label;             // 类别索引 (如 0 代表人, 1 代表车)
    float prob;            // 置信度分数 (0.0 ~ 1.0)
};

class Yolo {
public:
    Yolo() {
        // 优化内存分配，防止频繁申请内存导致的卡顿
        blob_pool_allocator.set_size_compare_ratio(0.f);
        workspace_pool_allocator.set_size_compare_ratio(0.f);
    }

    /**
     * @brief 加载模型
     * @param modeltype 模型文件路径前缀 (如 "yolov8n")
     * @param target_size 模型要求的输入尺寸 (如 320, 640)
     */
    int load(const char* modeltype, int target_size, float prob_threshold = 0.45f, float nms_threshold = 0.5f) {
        net.clear();
        
        // --- 性能配置区 ---
        net.opt.num_threads = 4;            // 【可改】根据 CPU 核心数调整，橘子派通常设为 4
        net.opt.use_vulkan_compute = false; // 如果有 GPU 且安装了 Vulkan 可设为 true
        net.opt.blob_allocator = &blob_pool_allocator;
        net.opt.workspace_allocator = &workspace_pool_allocator;

        char parampath[256];
        char modelpath[256];
        sprintf(parampath, "%s.param", modeltype); // 结构文件
        sprintf(modelpath, "%s.bin", modeltype);   // 权重文件

        if (net.load_param(parampath) != 0 || net.load_model(modelpath) != 0) {
            fprintf(stderr, "错误：模型加载失败，请检查路径！\n");
            return -1;
        }

        target_size_ = target_size;
        prob_threshold_ = prob_threshold;
        nms_threshold_ = nms_threshold;
        return 0;
    }

    /**
     * @brief 执行检测
     * @param rgb 输入的图像 (OpenCV BGR 格式)
     * @param objects 输出的检测结果列表
     */
    int detect(const cv::Mat& rgb, std::vector<Object>& objects) {
        int width = rgb.cols;
        int height = rgb.rows;

        // --- 1. 图像预处理 (Letterbox) ---
        // 目的：将长方形图片缩放并嵌入到正方形模型输入中，多余部分填黑边
        int w = width;
        int h = height;
        float scale = 1.f;
        if (w > h) {
            scale = (float)target_size_ / w;
            w = target_size_;
            h = h * scale;
        } else {
            scale = (float)target_size_ / h;
            h = target_size_;
            w = w * scale;
        }

        // 缩放并转换格式 (BGR 2 RGB)
        ncnn::Mat in = ncnn::Mat::from_pixels_resize(rgb.data, ncnn::Mat::PIXEL_BGR2RGB, width, height, w, h);
        
        // 计算填充量 (Padding)
        int wpad = target_size_ - w;
        int hpad = target_size_ - h;
        ncnn::Mat in_pad;
        ncnn::copy_make_border(in, in_pad, hpad / 2, hpad - hpad / 2, wpad / 2, wpad - wpad / 2, ncnn::BORDER_CONSTANT, 114.f);

        // 归一化：将 0~255 像素值缩放到 0~1
        const float norm_vals[3] = {1 / 255.f, 1 / 255.f, 1 / 255.f};
        in_pad.substract_mean_normalize(0, norm_vals);

        // --- 2. 推理阶段 ---
        ncnn::Extractor ex = net.create_extractor();
        ex.input("in0", in_pad);  // "in0" 必须与 .param 里的输入节点名对应
        ncnn::Mat out;
        ex.extract("out0", out); // "out0" 必须与 .param 里的输出节点名对应

        // --- 3. 后处理 (解析 [84 x 2100] 数据) ---
        // 2100 是预测框总数，84 = [x, y, w, h] + 80个类别的得分
        std::vector<Object> proposals;
        int num_anchors = out.w; 
        int num_classes = out.h - 4;

        for (int i = 0; i < num_anchors; i++) {
            // 找到得分最高的类别
            int label = -1;
            float score = -FLT_MAX;
            for (int k = 0; k < num_classes; k++) {
                float s = out.row(4 + k)[i]; // 第 4 行之后是类别分数
                if (s > score) {
                    label = k;
                    score = s;
                }
            }

            // 过滤低分框
            if (score > prob_threshold_) {
                float x = out.row(0)[i];
                float y = out.row(1)[i];
                float pb_w = out.row(2)[i] - x; // 将坐标转换为 w
                float pb_h = out.row(3)[i] - y; // 将坐标转换为 h

                // 【核心】坐标还原：减去黑边偏移，除以缩放倍数
                float r_x = (x - (wpad / 2)) / scale;
                float r_y = (y - (hpad / 2)) / scale;
                float r_w = pb_w / scale;
                float r_h = pb_h / scale;

                Object obj;
                obj.rect = cv::Rect_<float>(r_x, r_y, r_w, r_h);
                obj.label = label;
                obj.prob = score;
                proposals.push_back(obj);
            }
        }

        // --- 4. NMS (非极大值抑制) ---
        // 目的：把重叠的多个框合并成一个
        qsort_descent_inplace(proposals);
        std::vector<int> picked;
        nms_sorted_bboxes(proposals, picked, nms_threshold_);

        objects.resize(picked.size());
        for (int i = 0; i < (int)picked.size(); i++) {
            objects[i] = proposals[picked[i]];
        }

        return 0;
    }

private:
    ncnn::Net net;
    int target_size_;
    float prob_threshold_;
    float nms_threshold_;
    ncnn::UnlockedPoolAllocator blob_pool_allocator;
    ncnn::PoolAllocator workspace_pool_allocator;

    // 辅助函数：排序和计算重叠面积 (通常不需要修改)
    static void qsort_descent_inplace(std::vector<Object>& faceobjects) {
        if (faceobjects.empty()) return;
        std::sort(faceobjects.begin(), faceobjects.end(), [](const Object& a, const Object& b) { return a.prob > b.prob; });
    }

    static void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold) {
        picked.clear();
        const int n = faceobjects.size();
        std::vector<float> areas(n);
        for (int i = 0; i < n; i++) areas[i] = faceobjects[i].rect.area();
        for (int i = 0; i < n; i++) {
            const Object& a = faceobjects[i];
            int keep = 1;
            for (int j = 0; j < (int)picked.size(); j++) {
                const Object& b = faceobjects[picked[j]];
                cv::Rect_<float> inter = a.rect & b.rect;
                float inter_area = inter.area();
                float union_area = areas[i] + areas[picked[j]] - inter_area;
                if (inter_area / union_area > nms_threshold) { keep = 0; break; }
            }
            if (keep) picked.push_back(i);
        }
    }
};

#endif

