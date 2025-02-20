#ifndef FOLLOW_DOUBLE_SINGLE_STRATEGY_H
#define FOLLOW_DOUBLE_SINGLE_STRATEGY_H

#include "SingleStrategy.h"
#include "SingleZoom.h"
#include "DoubleFeatureNet.h"

namespace XbotgoSDK
{

class FollowDoubleSingleStrategy : public SingleStrategy
{

public:
    FollowDoubleSingleStrategy()
    {
        std::cout << "FollowDoubleSingleStrategy is being created." << std::endl;
    }
    ~FollowDoubleSingleStrategy()
    {
        std::cout << "FollowDoubleSingleStrategy is being destroyed." << std::endl;
    }

    void targetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target) override;

    /// @brief 通过下标初始化目标,注意此时为两个目标
    /// @param detectResults yolo的检测结果
    /// @param targets 目标的信息 指针
    void myTargetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<std::shared_ptr<DetectionResult>>& targets);

    /// @brief 获取当前的追踪点
    /// @param detectResult  yolo结果
    /// @param time 时间戳
    /// @return
    std::vector<int> getTrackPoint(const std::vector<YOLODetectionResult>& detectResult, double time);

protected:
    int tryToDoTragetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults) override { return 0; }

    /// @brief 做特征处理
    /// @param index SOT得到的结果，-1表示追踪失败
    /// @return 追踪目标的下标，如果返回-1，表示失败
    int doFeatureProcessing(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int index) override { return 0; }

    int mydoFeatureProcessing(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int id, int index);

    void zoom(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue) override;

    void control(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue) override;

private:
    int handleSOTSuccess(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int id, int index);
    int handleSOTFailure(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int id, int index);

    /// @brief
    /// @param detectResults
    /// @param id
    /// @param index
    /// @return
    int doFullFeatureNetFound(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int id, int index);

    /// @brief 根据下标更新FeatureNet特征队列
    /// @param index
    void updateFeatureNet(int id, int index);

    /// @brief
    /// @param id 表示第几个目标
    /// @param index
    /// @return
    int featureNetVerification(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int id, int index);

    /// @brief 根据目标在数组中找到对应的数组下标
    /// @param detectResults
    /// @param target
    /// @return
    int findDetectionResultIndex(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target);

    /// @brief 转换YOLO数组到RectBox数组
    /// @param results
    /// @return
    std::vector<RectBox> extractYOLODetectionResults(const std::vector<YOLODetectionResult>& results);
    /// @brief 特征提取及处理
    std::unique_ptr<DoubleFeatureNet> _doubleFeatureNet;
};
} // namespace XbotgoSDK

#endif // FOLLOW_DOUBLE_SINGLE_STRATEGY_H