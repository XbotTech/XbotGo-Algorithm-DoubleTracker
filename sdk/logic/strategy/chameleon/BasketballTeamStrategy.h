
#ifndef __XBOTGO_SDK_CORE_LOGIC_BASKETABLLTEAMSTRATEGY_H__
#define __XBOTGO_SDK_CORE_LOGIC_BASKETABLLTEAMSTRATEGY_H__

#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "Global.h"
#include "TeamStrategy.h"

namespace XbotgoSDK
{

class BasketballTeamStrategy : public TeamStrategy
{
public:
    BasketballTeamStrategy(const StrategyParam& param)
    {
        pitchPID = new PID(3.0, 0.0, 0.0, 1440, 15, 1000, "Pitch");
        yawPID = new PID(0.85, 0.0, 0.0, 2560, 15, 750, "Yaw");
        yawSmoother = new Smoother(120, 40, 40, 80, "Yaw");
        this->param = param;
    }
    ~BasketballTeamStrategy()
    {
        printfXbotGo("TeamStrategyBasketball::~\n");
    }

    int init() override;

    int dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults) override;

    int caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult) override;

    TrackResult caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults);
};

}

#endif