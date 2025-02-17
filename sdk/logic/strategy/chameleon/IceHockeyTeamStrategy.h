
#ifndef __XBOTGO_SDK_CORE_LOGIC_ICEHOCKEYTEAMSTRATEGY_H__
#define __XBOTGO_SDK_CORE_LOGIC_ICEHOCKEYTEAMSTRATEGY_H__

#include <iostream>
#include <memory>

#include "TeamStrategy.h"

namespace XbotgoSDK
{

class IceHockeyTeamStrategy : public TeamStrategy
{
public:
    IceHockeyTeamStrategy(const StrategyParam& param)
    {
        pitchPID = new PID(3.0, 0.0, 0.0, 1440, 15, 1000, "Pitch");
        yawPID = new PID(0.85, 0.0, 0.0, 2560, 15, 750, "Yaw");
        yawSmoother = new Smoother(120, 40, 40, 80, "Yaw");
        this->param = param;
    }
    ~IceHockeyTeamStrategy()
    {
        printfXbotGo("TeamIceHockey:~\n");
    }

    int init() override;

    int dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults) override;

    TrackResult caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults);

    int autoTracking(TrackResult& trackPoint, const std::vector<MOTResult>& motResult);

    int zoomTrackPoint(const std::vector<MOTResult>& motResults, const TrackResult& trackResult) override;

    void control(const TrackResult& trackResult) override;

private:
    int autoTrackingFrameIntervals;
};

}

#endif