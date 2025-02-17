#include "SingleStrategyPose.h"
#include "XbotgoSingleStrategyCallback.h"


namespace XbotgoSDK
{

extern XbotgoSingleStrategyCallback*  g_xbotgoSingleStrategyCallbacks;

void SingleStrategyPose::init(std::vector<std::shared_ptr<DetectionResult>>& bodyDetectResults, std::shared_ptr<DetectionResult>& target)
{
    _sot = std::make_unique<SOT>();
    _featureNet = std::make_unique<PoseFeatureNet>(deviceVideoBufferWidth, deviceVideoBufferHeight);
    _singleZoom = std::make_unique<SingleZoom>(deviceVideoBufferWidth, deviceVideoBufferHeight, minZoom, maxZoom, 0.3, 0.7, 0.4);

    std::vector<YOLODetectionResult> bodyYoloResultsVirtual = convertToVirtual(bodyDetectResults);

    int targetIndex = findDetectionResultIndex(bodyDetectResults, target);
    _sot->targetInit(bodyYoloResultsVirtual, targetIndex);
    _featureNet->targetInit(target->personResult);
}

HeadAndBodyIndex SingleStrategyPose::track(std::vector<std::shared_ptr<DetectionResult>>& headDetectResults, std::vector<std::shared_ptr<DetectionResult>>& bodyDetectResults)
{
    HeadAndBodyIndex _result;
    std::vector<YOLODetectionResult> bodyYoloResults;
    std::vector<YOLODetectionResult> bodyYoloResultsVirtual;

    if (bodyDetectResults.empty()) {
        // 如果yolo_detection_results为空，直接返回_result
        _result.head_index = -1;
        _result.body_index = -1;
    }
    else {
        std::vector<NewPoseDetectionResult> headDetectResultsPose;
        headDetectResultsPose.reserve(headDetectResults.size());
        for (const auto& detectResult : headDetectResults) {
            auto poseResult = std::dynamic_pointer_cast<NewPoseDetectionResult>(detectResult);
            if (poseResult) {
                headDetectResultsPose.push_back(*poseResult); // 解引用智能指针以获取对象
            } else {
                // TODO: 处理没有找到的情况
            }
        }
        std::vector<NewPoseDetectionResult> bodyDetectResultsPose;
        bodyDetectResultsPose.reserve(bodyDetectResults.size());
        for (const auto& detectResult : bodyDetectResults) {
            auto poseResult = std::dynamic_pointer_cast<NewPoseDetectionResult>(detectResult);
            if (poseResult) {
                bodyDetectResultsPose.push_back(*poseResult); // 解引用智能指针以获取对象
            } else {
                // TODO: 处理没有找到的情况
            }
        }
        personMatching(headDetectResultsPose,bodyDetectResultsPose);

        bodyYoloResults = convertToYOLOResults(bodyDetectResults);
        bodyYoloResultsVirtual = convertToVirtual(bodyDetectResults);
        // track
        _result.body_index = follow(bodyYoloResults, bodyYoloResultsVirtual);
        if (_result.body_index != -1) {
            _result.head_index = bodyDetectResultsPose[_result.body_index].matchedIndex;
        }
        else {
            _result.head_index = -1;
        }
    }

    double currentZoomValue = g_xbotgoSingleStrategyCallbacks->onGetZoomFactor();
    zoom(bodyYoloResults, _result.body_index, currentZoomValue);
    control(bodyYoloResults, _result.body_index, currentZoomValue);

    return _result;
}

int SingleStrategyPose::follow(std::vector<YOLODetectionResult>& bodyDetectResults, std::vector<YOLODetectionResult>& bodyYoloResultsVirtual)
{
    if (_isSkipSOT == false) // 是否跳过ByteTrack判断
    {
        int bt_idx = _sot->track(bodyYoloResultsVirtual); // 利用ByteTrack进行单目标追踪

        if (bt_idx != -1) // 判断ByteTrack单目标追踪是否成功，成功时bt_idx不等于-1
        {
            // _SOTFailureCount = 0; // ByteTrack连续失败次数

            YOLODetectionResult padding_rect = paddingRect(bodyDetectResults[bt_idx], deviceVideoBufferWidth, deviceVideoBufferHeight);
            std::vector<Box> postProcess_result = postProcess(g_xbotgoSingleStrategyCallbacks->onPoseDetect(padding_rect));
            std::tuple<int, std::array<float, 4>, std::array<float, 4>> target_pose = analysisPose(bodyDetectResults[bt_idx], padding_rect, postProcess_result);

            // 判断是否存在重叠，应对多人重叠的情况
            if (_featureNet->isOverLap(bodyDetectResults, bt_idx, postProcess_result) == false)
            {
                if (_skipReIDVerifyCount != 0) // 前几帧存在重叠情况，需要进行上一帧特征Reid验证
                {
                    _skipReIDVerifyCount--;

                    int pose = std::get<0>(target_pose);
                    _featureNet->debug_log_pose_state_ = pose;
                    if (pose != _pose_none_)
                    {
                        YOLODetectionResult target_coord = {0,1,0,0,0,0};
                        if (pose == _pose_full_)
                        {
                            std::array<float, 4> fullbody_rect = std::get<1>(target_pose);
                            target_coord = {0,1, padding_rect.x + (fullbody_rect[0] * padding_rect.width),
                                            padding_rect.y + (fullbody_rect[1] * padding_rect.height),
                                            padding_rect.width * (fullbody_rect[2] - fullbody_rect[0]),
                                            padding_rect.height * (fullbody_rect[3] - fullbody_rect[1])};
                        }
                        else if (pose == _pose_half_ || pose == _pose_shoulder_)
                        {
                            std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
                            target_coord = {0, 1, padding_rect.x + (halfbody_rect[0] * padding_rect.width),
                                            padding_rect.y + (halfbody_rect[1] * padding_rect.height),
                                            padding_rect.width * (halfbody_rect[2] - halfbody_rect[0]),
                                            padding_rect.height * (halfbody_rect[3] - halfbody_rect[1])};
                        }

                        // 上一帧特征Reid验证，将当前成功追踪的目标特征和保存的上一帧追踪目标特征进行比较
                        if (_featureNet->reidLocalVerification(target_coord, bt_idx, bodyDetectResults) == true)
                        {
                            // ------------------------ Debug Log ------------------------
                            g_xbotgoSingleStrategyCallbacks->onDebugFollowMeInfo(_byte_success_, _reid_active_verification_, _featureNet->debug_log_pose_state_,
                            _featureNet->debug_log_list_full_, _featureNet->debug_log_list_half_, 0.0, 0.0, _featureNet->debug_log_distance_pre_);
                            // debug_log_manager_->writeDebugLogs("byte_overlap_");
                            // ------------------------ Debug Log ------------------------

                            // ByteTrack追踪成功且上一帧特征Reid验证通过，但追踪目标和其他人有重叠情况，不更新特征队列
                            return bt_idx;
                        }
                        else
                        {
                            _featureNet->debug_log_byte_state_ = _byte_wrong_;
                        }
                    }
                    else{
                        _featureNet->debug_log_byte_state_ = _byte_wrong_pose_;
                    }
                    _isSkipSOT = true;
                }
                else // 前几帧不存在重叠情况，跳过验证
                {
                    _featureNet->updateTargetFeatureList(padding_rect, target_pose); // 更新追踪目标的特征队列

                    // ------------------------ Debug Log ------------------------
                    g_xbotgoSingleStrategyCallbacks->onDebugFollowMeInfo(_byte_success_, _reid_unstart_, _featureNet->debug_log_pose_state_,
                    _featureNet->debug_log_list_full_, _featureNet->debug_log_list_half_, _featureNet->debug_log_distance_full_, _featureNet->debug_log_distance_half_, 0.0);
                    // debug_log_manager_->writeDebugLogs("byte_only_");
                    // ------------------------ Debug Log ------------------------

                    // ByteTrack追踪成功，且追踪目标没有和其他人有重叠情况
                    return bt_idx;
                }
            }
            else // 如果出现多人重叠的情况，进行上一帧特征Reid验证
            {
                _skipReIDVerifyCount = 3;

                int pose = std::get<0>(target_pose);
                _featureNet->debug_log_pose_state_ = pose;
                if (pose != _pose_none_)
                {
                    YOLODetectionResult target_coord = {0,1,0,0,0,0};
                    if (pose == _pose_full_)
                    {
                        std::array<float, 4> fullbody_rect = std::get<1>(target_pose);
                        target_coord = {0, 1, padding_rect.x + (fullbody_rect[0] * padding_rect.width),
                                        padding_rect.y + (fullbody_rect[1] * padding_rect.height),
                                        padding_rect.width * (fullbody_rect[2] - fullbody_rect[0]),
                                        padding_rect.height * (fullbody_rect[3] - fullbody_rect[1])};
                    }
                    else if (pose == _pose_half_ || pose == _pose_shoulder_)
                    {
                        std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
                        target_coord = {0,1, padding_rect.x + (halfbody_rect[0] * padding_rect.width),
                                        padding_rect.y + (halfbody_rect[1] * padding_rect.height),
                                        padding_rect.width * (halfbody_rect[2] - halfbody_rect[0]),
                                        padding_rect.height * (halfbody_rect[3] - halfbody_rect[1])};
                    }

                    // 上一帧特征Reid验证，将当前成功追踪的目标特征和保存的上一帧追踪目标特征进行比较
                    if (_featureNet->reidLocalVerification(target_coord, bt_idx, bodyDetectResults) == true)
                    {
                        // ------------------------ Debug Log ------------------------
                        g_xbotgoSingleStrategyCallbacks->onDebugFollowMeInfo(_featureNet->debug_log_byte_state_, _reid_overlap_verification_, _featureNet->debug_log_pose_state_,
                        _featureNet->debug_log_list_full_, _featureNet->debug_log_list_half_, 0.0, 0.0, _featureNet->debug_log_distance_pre_);
                        // debug_log_manager_->writeDebugLogs("byte_overlap_");
                        // ------------------------ Debug Log ------------------------

                        // ByteTrack追踪成功且上一帧特征Reid验证通过，但追踪目标和其他人有重叠情况，不更新特征队列
                        return bt_idx;
                    }
                }
                else
                {
                    _featureNet->debug_log_byte_state_ = _byte_wrong_overlap_pose_;
                }
                _isSkipSOT = true;
            }
        }
        else // ByteTrack追踪失败
        {
            _isSkipSOT = true;
            _featureNet->debug_log_byte_state_ = _byte_lose_;
        }
    }
    else {
        _featureNet->debug_log_byte_state_ = _byte_unstart_;
    }

    int fn_idx = _featureNet->reidTrack(bodyDetectResults); // 利用Reid进行单目标追踪
    // _SOTFailureCount++;                            // ByteTrack连续失败次数

    if (fn_idx != -1) // 若Reid追踪成功，重启ByteTrack追踪
    {
        _sot->restart(bodyYoloResultsVirtual, fn_idx);
        // _SOTFailureCount = 0; // ByteTrack连续失败次数
        _isSkipSOT = false;
    }

    // ------------------------ Debug Log ------------------------
    g_xbotgoSingleStrategyCallbacks->onDebugFollowMeInfo(_featureNet->debug_log_byte_state_, _featureNet->debug_log_reid_state_, _featureNet->debug_log_pose_state_,
    _featureNet->debug_log_list_full_, _featureNet->debug_log_list_half_, _featureNet->debug_log_distance_full_, _featureNet->debug_log_distance_half_, _featureNet->debug_log_distance_pre_);
    // debug_log_manager_->writeDebugLogs("reid_" + std::to_string(debug_log_manager_->state_));
    // ------------------------ Debug Log ------------------------

    // 返回Reid追踪结果，若Reid追踪成功则返回追踪结果在当前帧图片对应的YOLO结果索引，若Reid追踪失败则返回-1
    return fn_idx;
}

std::vector<YOLODetectionResult> SingleStrategyPose::convertToYOLOResults(const std::vector<std::shared_ptr<DetectionResult>>& detectResults) 
{
    std::vector<YOLODetectionResult> yoloResults;
    yoloResults.reserve(detectResults.size()); // 预留空间，避免多次内存分配

    for (const auto& result : detectResults) {
        yoloResults.push_back(result->personResult);
    }
    return yoloResults;
}

std::vector<YOLODetectionResult> SingleStrategyPose::convertToVirtual(const std::vector<std::shared_ptr<DetectionResult>>& detectResults)
{
    std::vector<YOLODetectionResult> virtualResults;
    virtualResults.reserve(detectResults.size()); // 预分配内存

    for (const auto& result : detectResults)
    {
        virtualResults.push_back({0,result->personResult.prob,
                                  result->personResult.x + result->xOffset,
                                  result->personResult.y + result->yOffset,
                                  result->personResult.width,
                                  result->personResult.height});
    }
    return virtualResults;
}

void SingleStrategyPose::personMatching(std::vector<NewPoseDetectionResult>& headDetectResults, std::vector<NewPoseDetectionResult>& bodyDetectResults)
{
    // 如果任一输入为空，则直接返回
    if (headDetectResults.empty() || bodyDetectResults.empty())
    {
        return;
    }
    for (size_t i = 0; i < bodyDetectResults.size(); i++)
    {
        double min_distance = std::numeric_limits<double>::max();
        int best_match = -1;

        // 计算人体框的中心点坐标
        // double body_center_x = bodyDetectResults[i].x + 0.5 * bodyDetectResults[i].width;
        // double body_center_y = bodyDetectResults[i].y + 0.5 * bodyDetectResults[i].height;
        double body_center_y = bodyDetectResults[i].personResult.y;

        for (size_t j = 0; j < headDetectResults.size(); j++)
        {
            // 如果该人头框已经匹配过，则跳过
            if (headDetectResults[j].matchedIndex != -1)
            {
                continue;
            }

            // 计算人头框的中心点坐标
            double head_center_x = headDetectResults[j].personResult.x + 0.5 * headDetectResults[j].personResult.width;
            double head_center_y = headDetectResults[j].personResult.y + 0.5 * headDetectResults[j].personResult.height;

            // 判断人头框中心是否在人体框内部
            if (head_center_x > bodyDetectResults[i].personResult.x &&
                head_center_x < bodyDetectResults[i].personResult.x + bodyDetectResults[i].personResult.width &&
                head_center_y > bodyDetectResults[i].personResult.y &&
                head_center_y <= bodyDetectResults[i].personResult.y + bodyDetectResults[i].personResult.height)
            {
                // 计算距离
                // double distance = std::sqrt(std::pow(head_center_x - body_center_x, 2) + std::pow(head_center_y - 0.5 * headDetectResults[j].height - body_center_y, 2));
                double distance = std::abs(head_center_y - 0.5 * headDetectResults[j].personResult.height - body_center_y);

                // 找到距离最近的人头框
                if (distance < min_distance)
                {
                    min_distance = distance;
                    best_match = j;
                }
            }
        }

        // 如果找到最好的匹配，则记录匹配信息
        if (best_match != -1)
        {
            headDetectResults[best_match].matchedIndex = static_cast<int>(i);
            bodyDetectResults[i].matchedIndex = static_cast<int>(best_match);
        }
    }
}

int SingleStrategyPose::findDetectionResultIndex(std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target) {
    for (size_t i = 0; i < detectResults.size(); ++i) {
        if (*detectResults[i] == *target) {
            return static_cast<int>(i);
        }
    }
    return -1; // 如果未找到则返回 -1
}

void SingleStrategyPose::zoom(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue) {

   ZoomResult zoomResult{0,0,0};
    if (index == -1) {
        _zoomLostCount++;

        if (_zoomLostCount > 30) {
            zoomResult = _singleZoom->zoomOut(currentZoomValue);
        }
    }
    else {
        _zoomLostCount = 0;
        zoomResult = _singleZoom->zoom(index, detectResults, currentZoomValue);
    }

    g_xbotgoSingleStrategyCallbacks->onSetZoomFactor(zoomResult.zoom_value, zoomResult.zoom_time, zoomResult.zoom_rate);
}

void SingleStrategyPose::control(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue) {

    double targetYaw = deviceVideoBufferWidth/2.0; // 其实是X
    double targetPitch = deviceVideoBufferHeight/2.0; // 其实是Y

    if (index != -1) {

        // yaw
        if (detectResults[index].x + 0.5 * detectResults[index].width < 0.45 * deviceVideoBufferWidth || detectResults[index].x + 0.5 * detectResults[index].width > 0.55 * deviceVideoBufferWidth) {
            targetYaw = ((detectResults[index].x + 0.5 * detectResults[index].width - 0.5 * deviceVideoBufferWidth) / currentZoomValue) + 0.5 * deviceVideoBufferWidth;
        }
        
        // pitch
        if (detectResults[index].y < 0.3 * deviceVideoBufferHeight) {
            targetPitch = ((2 * detectResults[index].y + 0.5 * detectResults[index].height - 0.8 * deviceVideoBufferHeight) / currentZoomValue) + 0.5 * deviceVideoBufferHeight;
        }
        else {
            targetPitch = ((detectResults[index].y + 0.4 * detectResults[index].height - 0.5 * deviceVideoBufferHeight) / currentZoomValue) + 0.5 * deviceVideoBufferHeight;
        }

    }

    g_xbotgoSingleStrategyCallbacks->onSetGimbal(targetYaw, targetPitch);
}

}
