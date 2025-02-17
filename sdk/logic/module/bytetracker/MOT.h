#pragma once
#include <iostream>
#include <vector>

#include "ByteTrack/BYTETracker.h"
#include "ByteTrack/STrack.h"
#include "DetectionResult.h"
#include "Global.h"
#include "SOT.h"

/**
 * @brief 多目标追踪结果结构体
 */
struct MOTResult {
    int Track_id;
    float x;
    float y;
    float width;
    float height;
    float Score;
    float distance;

    bool operator==(const MOTResult& other) const
    {
        return Track_id == other.Track_id;
    }
    bool operator<(const MOTResult& other) const
    {
        return Track_id < other.Track_id;
    }
};

/**
 * @brief 基于ByteTrack实现的多目标追踪器
 */
class MOT
{
public:
    /**
     * @brief 构造函数，初始化BYTETracker
     *
     * @param frame_rate 帧率，默认值为30
     * @param track_buffer 跟踪缓冲区大小，默认值为30
     * @param track_thresh 跟踪阈值，默认值为0.5
     * @param high_thresh 高阈值，默认值为0.6
     * @param match_thresh 匹配阈值，默认值为0.8
     */
    MOT(const int& frame_rate = 30, const int& track_buffer = 30,
        const float& track_thresh = 0.5, const float& high_thresh = 0.6,
        const float& match_thresh = 0.8);

    /**
     * @brief 目标追踪函数：利用ByteTrack实现的多目标追踪
     *
     * @param yolo_detection_results 当前帧图片的YOLO检测结果(所有结果的置信度以及坐标信息)
     */
    void track(std::vector<YOLODetectionResult>& yolo_detection_results, std::vector<MOTResult>& mot_result);

private:
    BYTETracker* byte; /**< 对象指针：利用ByteTrack实现多目标追踪 */
};