#include "MOT.h"

/**
 * @brief 构造函数，初始化BYTETracker
 *
 * @param frame_rate 帧率，默认值为30
 * @param track_buffer 跟踪缓冲区大小，默认值为30
 * @param track_thresh 跟踪阈值，默认值为0.5
 * @param high_thresh 高阈值，默认值为0.6
 * @param match_thresh 匹配阈值，默认值为0.8
 */
MOT::MOT(const int& frame_rate, const int& track_buffer, const float& track_thresh, const float& high_thresh, const float& match_thresh)
    : byte(new BYTETracker(frame_rate, track_buffer, track_thresh, high_thresh, match_thresh))
{
    printfXbotGo("MOT frame_rate:%d\n", frame_rate);
}

/**
 * @brief 目标追踪函数：利用ByteTrack实现的多目标追踪
 *
 * @param yolo_detection_results 当前帧图片的YOLO检测结果(所有结果的置信度以及坐标信息)
 */
void MOT::track(std::vector<YOLODetectionResult>& yolo_detection_results, std::vector<MOTResult>& mot_result)
{
    std::vector<Object> objects = convertDetections2Objects(yolo_detection_results);
    std::vector<STrack> outputs;
    byte->update(outputs, objects);
    mot_result.clear();
    for (const auto& output : outputs) {
        MOTResult result;
        result.Track_id = output.track_id;
        result.x = output.tlwh[0];
        result.y = output.tlwh[1];
        result.width = output.tlwh[2];
        result.height = output.tlwh[3];
        result.Score = output.score;
        mot_result.push_back(result);
    }
}