
#ifndef __XBOTGO_SDK_CORE_LOGIC_FOOTABLLTEAMSTRATEGY_H__
#define __XBOTGO_SDK_CORE_LOGIC_FOOTABLLTEAMSTRATEGY_H__

#include <iostream>
#include <memory>

#include "Global.h"
#include "TeamStrategy.h"

namespace XbotgoSDK
{

class FootballTeamStrategy : public TeamStrategy
{
public:
    FootballTeamStrategy(const StrategyParam& param)
    {
        pitchPID = new PID(3.0, 0.0, 0.0, 1440, 15, 1000, "Pitch");
        yawPID = new PID(0.85, 0.0, 0.0, 2560, 15, 750, "Yaw");
        yawSmoother = new Smoother(120, 40, 40, 80, "Yaw");
        this->param = param;
    }
    ~FootballTeamStrategy()
    {
        printfXbotGo("FootballTeamStrategy::~FootballTeamStrategy()\n");
    }

    int init() override;

    int dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults) override;

    int caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult) override;

    TrackResult caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults);

    int autoTracking(TrackResult& trackPoint, const std::vector<MOTResult>& motResult);

    void control(const TrackResult& trackResult) override;

private:
    int autoTrackingFrameIntervals;
};

}

#endif