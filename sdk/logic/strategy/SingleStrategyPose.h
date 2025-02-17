
#ifndef __XBOTGO_SDK_CORE_LOGIC_SINGLSTRATEGYPOSE_H__
#define __XBOTGO_SDK_CORE_LOGIC_SINGLSTRATEGYPOSE_H__
#include <iostream>
#include <memory>
#include <vector>
#include "IStrategy.h"
#include "logic/module/bytetracker/SOT.h"
#include "PoseFeatureNet.h"
#include "SingleZoom.h"

namespace XbotgoSDK
{

class SingleStrategyPose: public IStrategy
{

public: 
    SingleStrategyPose(){
        std::cout << "SingleStrategyPose is being created." << std::endl;
    }
    ~SingleStrategyPose(){
        std::cout << "SingleStrategyPose is being destroyed." << std::endl;
    }

    void init(std::vector<std::shared_ptr<DetectionResult>>& bodyDetectResults, std::shared_ptr<DetectionResult>& target);

    HeadAndBodyIndex track(std::vector<std::shared_ptr<DetectionResult>>& headDetectResults, std::vector<std::shared_ptr<DetectionResult>>& bodyDetectResults);

    int follow(std::vector<YOLODetectionResult>& headDetectResults, std::vector<YOLODetectionResult>& bodyDetectResults);

    void zoom(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue);

    void control(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue);

    int track(const std::vector<YOLODetectionResult>& detectResult, double time)override { return 0;};

private: 
    /// @brief 单人跟踪Single Object Track，基于ByteTrack
    std::unique_ptr<SOT> _sot;

    /// @brief 特征提取及处理
    std::unique_ptr<PoseFeatureNet> _featureNet;

    /// @brief 用来处理单人跟踪的缩放
    std::unique_ptr<SingleZoom> _singleZoom;

    /// @brief 判断是否跳过SOT
    bool _isSkipSOT = false;

    /// @brief SOT连续失败的次数
    int _SOTFailureCount = 0;

    /// @brief SOT是否已经初始化
    bool _isSOTInit = false;

    /// @brief 持续ReID局部验证剩余次数
    int _skipReIDVerifyCount = 0;

    /// @brief zoom时丢失目标的次数
    int _zoomLostCount = 0;

    int findDetectionResultIndex(std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target);

    std::vector<YOLODetectionResult> convertToYOLOResults(const std::vector<std::shared_ptr<DetectionResult>>& detectResults);

    std::vector<YOLODetectionResult> convertToVirtual(const std::vector<std::shared_ptr<DetectionResult>>& detectResults);

    void personMatching(std::vector<NewPoseDetectionResult>& headDetectResults, std::vector<NewPoseDetectionResult>& bodyDetectResults);
};
}

#endif