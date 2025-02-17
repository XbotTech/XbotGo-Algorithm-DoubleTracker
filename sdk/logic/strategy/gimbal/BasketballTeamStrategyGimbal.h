
#ifndef __XBOTGO_SDK_CORE_LOGIC_BASKETABLLTEAMSTRATEGYGIMBAL_H__
#define __XBOTGO_SDK_CORE_LOGIC_BASKETABLLTEAMSTRATEGYGIMBAL_H__

#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "Global.h"
#include "TeamStrategyGimbal.h"

namespace XbotgoSDK
{

class BasketballTeamStrategyGimbal : public TeamStrategyGimbal
{
public:
    BasketballTeamStrategyGimbal(const StrategyParam& param)
    {
        pitchPID = new PID(0.9, 0.0, 0.0, 1440, 15, 700, "Pitch");
        yawPID = new PID(1.0, 0.0, 0.0, 2560, 15, 700, "Yaw");
        yawSmoother = new Smoother(120, 40, 40, 80, "Yaw");
        this->param = param;
    }
    ~BasketballTeamStrategyGimbal()
    {
        printfXbotGo("BasketballTeamStrategyGimbal::~BasketballTeamStrategyGimbal()\n");
    }

    int init() override;

    int dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults, std::vector<DelatTrackInfo>& filterMovedResults) override;

    int caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult) override;

    TrackResult caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults);
};

}

#endif