#include "SOT.h"

/**
 * @brief 构造函数，初始化BYTETracker
 *
 * @param frame_rate 帧率，默认为30
 * @param track_buffer 跟踪缓冲区大小，默认为30
 * @param track_thresh 跟踪阈值，默认为0.5
 * @param high_thresh 高阈值，默认为0.6
 * @param match_thresh 匹配阈值，默认为0.8
 */
SOT::SOT(const int& frameRate, const int& trackBuffer, const float& trackThresh, const float& highThresh, const float& matchThresh)
    : _byte(new BYTETracker(frameRate, trackBuffer, trackThresh, highThresh, matchThresh)), _frameRate(frameRate), _trackBuffer(trackBuffer), _trackThresh(trackThresh), _highThresh(highThresh), _matchThresh(matchThresh)
{
}

/**
 * @brief 初始化ByteTacker单目标追踪器，确定追踪目标
 *
 * @param yolo_detection_results 第一帧图片的YOLO检测结果(所有结果的置信度以及坐标信息)
 * @param index 追踪目标的YOLO检测结果索引(即追踪目标是YOLO检测结果中的第几个)
 */
void SOT::targetInit(const std::vector<YOLODetectionResult>& detectionResults, int index)
{
    delete _byte;
    this->_byte = new BYTETracker(_frameRate, _trackBuffer, _trackThresh, _highThresh, _matchThresh);

    std::vector<YOLODetectionResult> nonDetectionResults = detectionResults; // 创建一个非 const 副本

    for (auto& nonDetectionResult : nonDetectionResults) {
        nonDetectionResult.prob = 1.0;
    }

    std::vector<Object> objects = convertDetections2Objects(nonDetectionResults);
    std::vector<STrack> outputs;
    _byte->update(outputs, objects);

    _targetId = outputs[index].track_id;
}

/**
 * @brief 目标追踪函数：利用ByteTrack实现的单目标追踪
 *
 * @param yolo_detection_results 当前帧图片的YOLO检测结果(所有结果的置信度以及坐标信息)
 * @return int 跟踪结果的索引，如果未找到匹配的目标则返回-1
 */
int SOT::track(const std::vector<YOLODetectionResult>& detectionResults)
{
    std::vector<Object> objects = convertDetections2Objects(detectionResults);
    std::vector<STrack> outputs;
    _byte->update(outputs, objects);

    for (const auto& output : outputs) {
        if (_targetId == output.track_id) {
            std::vector<double> det_distance;
            for (const auto& det : detectionResults) {
                det_distance.push_back(pow(det.x - output.tlwh[0], 2) + pow(det.y - output.tlwh[1], 2));
            }
            int min_index = std::min_element(det_distance.begin(), det_distance.end()) - det_distance.begin();
            return min_index;
        }
    }
    return -1;
}

/**
 * @brief 清空ByteTrack原有记录的信息，初始化ByteTacker单目标追踪器，确定追踪目标
 *
 * @param yolo_detection_results 新一帧图片的YOLO检测结果(所有结果的置信度以及坐标信息)
 * @param index 追踪目标的YOLO检测结果索引(即追踪目标是YOLO检测结果中的第几个)
 */
void SOT::restart(const std::vector<YOLODetectionResult>& detectionResults, int index)
{
    targetInit(detectionResults, index);
}

/**
 * @brief 将YOLO检测结果转换为ByteTrack所需的Object格式
 *
 * @param yolo_detection_results YOLO检测结果
 * @return std::vector<byte_track::Object> ByteTrack所需的Object格式
 */
std::vector<Object> convertDetections2Objects(const std::vector<YOLODetectionResult>& yolo_detection_results)
{
    std::vector<Object> objects;
    for (const auto& det : yolo_detection_results) {
        Rect_<float> rect(det.x, det.y, det.width, det.height);
        Object obj(rect, 0, det.prob);

        objects.push_back(obj);
    }
    return objects;
}