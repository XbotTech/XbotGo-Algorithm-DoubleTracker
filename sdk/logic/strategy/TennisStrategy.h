#ifndef __XBOTGO_SDK_CORE_LOGIC_STRATEGY_TENNIS_H__
#define __XBOTGO_SDK_CORE_LOGIC_STRATEGY_TENNIS_H__
#include <iostream>
#include <memory>
#include "IStrategy.h"

namespace XbotgoSDK
{

class TennisStrategy : public IStrategy
{
public:
    TennisStrategy(){}
	virtual ~TennisStrategy(){ std::cout<<"TennisStrategy::~TennisStrategy()"<<std::endl; }

    int track(const std::vector<YOLODetectionResult>& detectResult, double time)override{return 0;}

    int track(const std::vector<YOLODetectionResult>& yoloResult,  const std::vector<BallDetectionResult>& hoopResult,int type);

private:
    static std::vector<BallDetectionResult> findLargestRectArea(const std::vector<BallDetectionResult>& list);
    static std::vector<YOLODetectionResult> findLastRect(const std::vector<YOLODetectionResult>& list);
    static std::vector<YOLODetectionResult> sortByBottomDescending(std::vector<YOLODetectionResult>& list);
    static std::vector<YOLODetectionResult> filterByBottomAndLeftRight(const BallDetectionResult& entity, const std::vector<YOLODetectionResult>& list);
    static std::vector<YOLODetectionResult> filterBelowMiddleOfEntityBottom(const BallDetectionResult& entity, const std::vector<YOLODetectionResult>& list,float bufferHeight);
    static std::vector<YOLODetectionResult> filterByBottom(const BallDetectionResult& entity, const std::vector<YOLODetectionResult>& list);

};


}

#endif