#pragma once
#include <iostream>
#include <vector>

#include "ByteTrack/BYTETracker.h"
#include "ByteTrack/STrack.h"
#include "DetectionResult.h"

/**
 * @brief 将YOLO检测结果转换为ByteTrack所需的Object对象
 *
 * @param yolo_detection_results YOLO检测结果(所有结果的置信度以及坐标信息)
 * @return std::vector<byte_track::Object> ByteTrack所需的Object对象
 */
std::vector<Object> convertDetections2Objects(const std::vector<YOLODetectionResult> &yolo_detection_results);

class SOT
{
public:
    /**
     * @brief 构造函数，初始化BYTETracker
     *
     * @param frame_rate 帧率，默认为30
     * @param track_buffer 跟踪缓冲区大小，默认为30
     * @param track_thresh 跟踪阈值，默认为0.5
     * @param high_thresh 高阈值，默认为0.6
     * @param match_thresh 匹配阈值，默认为0.8
     */
    SOT(const int& frame_rate = 30, const int& track_buffer = 30,
        const float& track_thresh = 0.5, const float& high_thresh = 0.6,
        const float& match_thresh = 0.8);

    ~SOT()
    {
        if (_byte != nullptr) {
            delete _byte;
        }
    }

    /**
     * @brief 初始化ByteTacker单目标追踪器，确定追踪目标
     *
     * @param yolo_detection_results 第一帧图片的YOLO检测结果(所有结果的置信度以及坐标信息)
     * @param index 追踪目标的YOLO检测结果索引(即追踪目标是YOLO检测结果中的第几个)
     */
    void targetInit(const std::vector<YOLODetectionResult>& detectionResults, int index);

    /**
     * @brief 目标追踪函数：利用ByteTrack实现的单目标追踪
     *
     * @param yolo_detection_results 当前帧图片的YOLO检测结果(所有结果的置信度以及坐标信息)
     * @return int 跟踪结果的索引，如果未找到匹配的目标则返回-1
     */
    int track(const std::vector<YOLODetectionResult>& detectionResults);

    /**
     * @brief 清空ByteTrack原有记录的信息，初始化ByteTacker单目标追踪器，确定追踪目标
     *
     * @param yolo_detection_results 新一帧图片的YOLO检测结果(所有结果的置信度以及坐标信息)
     * @param index 追踪目标的YOLO检测结果索引(即追踪目标是YOLO检测结果中的第几个)
     */
    void restart(const std::vector<YOLODetectionResult>& detectionResults, int index);

private:
    /// @brief 对象指针，利用ByteTrack实现多目标跟踪
    BYTETracker* _byte;

    /// @brief 单目标跟踪ID
    int _targetId;

    /// @brief 可调参数：帧率，默认为30
    int _frameRate;

    /// @brief 可调参数：跟踪缓冲区大小，默认为30
    int _trackBuffer;

    /// @brief 可调参数：跟踪阈值，默认为0.5
    float _trackThresh;

    /// @brief 可调参数：高阈值，默认为0.6
    float _highThresh;

    /// @brief 可调参数：匹配阈值，默认为0.8
    float _matchThresh;
};