#ifndef FOLLOW_ME_SINGLE_STRATEGY_H
#define FOLLOW_ME_SINGLE_STRATEGY_H

#include "SingleStrategy.h"
#include "SingleZoom.h"

namespace XbotgoSDK
{

class FollowMeSingleStrategy: public SingleStrategy
{

public: 
    FollowMeSingleStrategy(){
        std::cout << "FollowMeSingleStrategy is being created." << std::endl;
    }
    ~FollowMeSingleStrategy(){
        std::cout << "FollowMeSingleStrategy is being destroyed." << std::endl;
    }

    void targetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target)override;

protected:

    int tryToDoTragetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults)override{
        return 0;
    }

    int doFeatureProcessing(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int index)override;

    void zoom(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue)override;

    void control(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue)override;

private: 

    int handleSOTSuccess(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int index);
    int handleSOTFailure(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int index);

    int doFullFeatureNetFound(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int index);

    /// @brief 根据下标更新FeatureNet特征队列
    /// @param index 
    void updateFeatureNet(int index);

    /// @brief 
    /// @param index 
    /// @return 
    int featureNetVerification(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int index);

    /// @brief 根据目标在数组中找到对应的数组下标
    /// @param detectResults 
    /// @param target 
    /// @return 
    int findDetectionResultIndex(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target);

    /// @brief 转换YOLO数组到RectBox数组
    /// @param results 
    /// @return 
    std::vector<RectBox> extractYOLODetectionResults(const std::vector<YOLODetectionResult>& results);
};
}

#endif // FOLLOW_ME_SINGLE_STRATEGY_H