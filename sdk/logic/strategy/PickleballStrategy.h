#ifndef __XBOTGO_SDK_CORE_LOGIC_STRATEGY_PICKLEBALL_H__
#define __XBOTGO_SDK_CORE_LOGIC_STRATEGY_PICKLEBALL_H__
#include <iostream>
#include <memory>
#include "IStrategy.h"

namespace XbotgoSDK 
{

class PickleballStrategy: public IStrategy
{

public: 

enum PickleballType {
    Single = 1,
    Double = 2
};
    /// type: 匹克球类型
    PickleballStrategy(PickleballType type):type(type) {}
    ~PickleballStrategy(){ std::cout<<"PickleballStrategy::~PickleballStrategy()"<<std::endl; }

    /// @brief 设置angle信息
    /// @param maxAngle X轴最大转动度数(默认20先)，会根据输入的值判断正负
    /// @param pixelPerAngle 每度的像素偏移量(参考reid)
    void setAngleInfo(double maxAngle, double pixelPerAngle, int maxTurnDownTime);

    int track(const std::vector<YOLODetectionResult>& detectResult, double time)override { return 0;};

    /// @brief 开始跟踪
    /// @param detectResult 检测出的YOLO结果 
    /// @param currentXAngle 当前的X轴角度，左负右正
    /// @return 
    int track(const std::vector<YOLODetectionResult>& detectResult, double currentXAngle, int unused_flag);

private:

    PickleballType type;

    bool _isAngleSet = false;
    double _originalXAngle;

    double _maxXAngle;
    double _pixelXPerAngle;
    int _maxTurnDownTime;

    int _noPersonDetectTime = 0;
    int _maxNoPersonDetectTime = 20;

    int _turnDownTime = 0;

    void singleTrack(const std::vector<YOLODetectionResult>& detectResult, double currentXAngle);
    void doubleTrack(const std::vector<YOLODetectionResult>& detectResult, double currentXAngle);

    std::vector<YOLODetectionResult> filterFeetUnder(const std::vector<YOLODetectionResult>& allDetectResults, float y);
    std::vector<YOLODetectionResult> sortByClose(std::vector<YOLODetectionResult> allDetectResults);
    std::vector<YOLODetectionResult> sortByCenter(std::vector<YOLODetectionResult> allDectionResults, double xOffset, double deviceVideoBufferWidth);

    std::tuple<double, double> getTheCenterOfTwoDetectionResult(YOLODetectionResult firstDetectionResult, YOLODetectionResult secondDetectionResult);
};

}

#endif
