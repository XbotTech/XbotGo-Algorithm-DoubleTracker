#include "SingleStrategy.h"
#include "XbotgoStrategy.h"
#include "XbotgoSingleStrategy.h"
#include "XbotgoSingleStrategyCallback.h"
#include <vector>

namespace XbotgoSDK
{

extern XbotgoSingleStrategyCallback*  g_xbotgoSingleStrategyCallbacks;

/// @brief 追踪目标
/// @param detectResult YOLO检测结果
/// @return 是否又找到目标，有则返回目标在数组中对应的下标，没有则返回-1
int SingleStrategy::singleTrack(const std::vector<std::shared_ptr<DetectionResult>>& detectResults) {

    std::cout << "SingleStrategy::track"<<detectResults.size()<<std::endl;

    _featureNet->debug_log_distance_full_ = 0.0;
    _featureNet->debug_log_distance_half_ = 0.0;
    _featureNet->debug_log_distance_pre_ = 0.0;
    _featureNet->debug_log_pose_ = pose_none_;
    _featureNet->debug_log_state_ = reid_none_;

    // isMatchTarget
    if (_isSOTInit == false) {
        std::cout << "SingleStrategy::track sot not init"<<std::endl;        
        int targetIndex = tryToDoTragetInit(detectResults);
        return targetIndex;
    }

    int targetIndex = -1;

    std::vector<YOLODetectionResult> yoloResults = convertToYOLOResults(detectResults);

    if (_isSkipSOT == false) {
        std::vector<YOLODetectionResult> virtualResults;
        convertToVirtualResults(detectResults, virtualResults);
        targetIndex = _sot->track(virtualResults);
    }

    targetIndex = doFeatureProcessing(detectResults, yoloResults, targetIndex);

    double currentZoomValue = g_xbotgoSingleStrategyCallbacks->onGetZoomFactor();
    zoom(yoloResults, targetIndex, currentZoomValue);
    control(yoloResults, targetIndex, currentZoomValue);

    // debug info： _sotFailureCount，state(状态机)
    // state: {
    //     case sot {
    //         case
    //         case 
    //         case 
    //     }
    //     case featureNet {
    //         case 
    //         case 
    //         case 
    //         case 
    //     }
    // }

    g_xbotgoSingleStrategyCallbacks->onDebugFeatureNetInfo(_featureNet->debug_log_state_,
                                                           _featureNet->debug_log_pose_,
                                                           _featureNet->debug_log_list_full_,
                                                           _featureNet->debug_log_list_half_,
                                                           _featureNet->debug_log_distance_full_,
                                                           _featureNet->debug_log_distance_half_,
                                                           _featureNet->debug_log_distance_pre_);


    return targetIndex;
}

std::vector<YOLODetectionResult> SingleStrategy::convertToYOLOResults(const std::vector<std::shared_ptr<DetectionResult>>& detectResults) {
    std::vector<YOLODetectionResult> yoloResults;
    yoloResults.reserve(detectResults.size()); // 预留空间，避免多次内存分配

    for (const auto& result : detectResults) {
        yoloResults.push_back(result->personResult);
    }

    return yoloResults;
}

void SingleStrategy::convertToVirtualResults(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, std::vector<YOLODetectionResult>& virtualResults) {
    virtualResults.clear();
    for (const auto& resultPtr : detectResults) {
        const auto& result = *resultPtr;
        YOLODetectionResult virtualResult(0, result.personResult.prob,
                                          result.personResult.x + result.xOffset,
                                          result.personResult.y + result.yOffset,
                                          result.personResult.width,
                                          result.personResult.height);
        virtualResults.push_back(virtualResult);
    }
}

/// @brief 判断目标检测结果框是否重叠
/// @param yolo_detection_results 当前帧YOLO检测的结果
/// @param index 追踪目标的检测结果的索引
/// @return 是否存在重叠
bool isOverlapping(const std::vector<YOLODetectionResult> &yolo_detection_results, int index) 
{
    for (int i = 0; i < static_cast<int>(yolo_detection_results.size()); i++) {
        if (i != index) {
            int x_overlap = std::max(0.0,
                                     std::min(yolo_detection_results[index].x + 0.5 * yolo_detection_results[index].width,
                                              yolo_detection_results[i].x + 0.5 * yolo_detection_results[i].width) -
                                         std::max(yolo_detection_results[index].x - 0.5 * yolo_detection_results[index].width,
                                                  yolo_detection_results[i].x - 0.5 * yolo_detection_results[i].width));

            if (x_overlap != 0.0) {
                int y_overlap = std::max(0.0,
                                         std::min(yolo_detection_results[index].y + 0.5 * yolo_detection_results[index].height,
                                                  yolo_detection_results[i].y + 0.5 * yolo_detection_results[i].height) -
                                             std::max(yolo_detection_results[index].y - 0.5 * yolo_detection_results[index].height,
                                                      yolo_detection_results[i].y - 0.5 * yolo_detection_results[i].height));

                int intersection_area = x_overlap * y_overlap;
                int target_area = yolo_detection_results[index].width * yolo_detection_results[index].height;

                // 如果重叠部分面积大于目标框的五分之一，则认为重叠
                if (intersection_area > target_area / 5) {
                    return true;
                }
            }
        }
    }

    return false;
}
}