
#ifndef __XBOTGO_SDK_CORE_LOGIC_SINGLSTRATEGY_H__
#define __XBOTGO_SDK_CORE_LOGIC_SINGLSTRATEGY_H__
#include <iostream>
#include <memory>
#include "IStrategy.h"
#include "logic/module/bytetracker/SOT.h"
#include "FeatureNet.h"
#include "SingleZoom.h"

namespace XbotgoSDK
{

class SingleStrategy : public IStrategy
{
public:
    SingleStrategy(){
        std::cout << "SingleStrategy is being created." << std::endl;
    }

	virtual ~SingleStrategy(){
        std::cout << "SingleStrategy is being destroyed." << std::endl;
    }

    /// @brief 通过下标初始化目标
    /// @param detectResults YOLO检测的结果
    /// @param targetIndex 目标所在的下标位置
    virtual void targetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target) = 0;

    /// @brief 追踪目标
    /// @param detectResult YOLO检测结果
    /// @return 是否又找到目标，有则返回目标在数组中对应的下标，没有则返回-1
    int singleTrack(const std::vector<std::shared_ptr<DetectionResult>>& detectResults);

    int track(const std::vector<YOLODetectionResult>& detectResult, double time)override {
        return 0;
    };

protected:

    /// @brief 尝试初始化相关信息
    virtual int tryToDoTragetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults)=0;

    /// @brief 做特征处理
    /// @param index SOT得到的结果，-1表示追踪失败
    /// @return 追踪目标的下标，如果返回-1，表示失败
    virtual int doFeatureProcessing(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int index)=0;

    /// @brief 根据目标和跟踪的结果进行缩放，如果没有找到跟踪目标，则不进行缩放
    /// @param detectResult 检测的YOLO信息
    /// @param index 跟踪目标的下标
    /// @param currentZoomValue 当前缩放的系数
    virtual void zoom(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue)=0;

    /// @brief 根据目标和跟踪的结果进行控制
    /// @param detectResult 检测的YOLO信息
    /// @param index 跟踪目标的下标
    /// @param currentZoomValue 当前缩放的系数
    virtual void control(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue)=0;

    /// @brief 抽取输入的DetectionResult中的YOLODetectionResult
    /// @param detectResults 传入的数据
    /// @return YOLODetectionResult
    std::vector<YOLODetectionResult> convertToYOLOResults(const std::vector<std::shared_ptr<DetectionResult>>& detectResults);

    /// @brief 将Buffer中的坐标位置通过X,Y的偏移量转换到虚拟坐标中
    /// @param detectResults 检测出的结果
    /// @param virtualResults 虚拟坐标结果
    void convertToVirtualResults(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, std::vector<YOLODetectionResult>& virtualResults);


protected:
    /// @brief 单人跟踪Single Object Track，基于ByteTrack
    std::unique_ptr<SOT> _sot;

    /// @brief 特征提取及处理
    std::unique_ptr<FeatureNet> _featureNet;

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

};

}

#endif
