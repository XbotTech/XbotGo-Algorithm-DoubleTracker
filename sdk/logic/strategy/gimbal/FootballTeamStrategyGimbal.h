
#ifndef __XBOTGO_SDK_CORE_LOGIC_FOOTABLLTEAMSTRATEGYGIMBAL_H__
#define __XBOTGO_SDK_CORE_LOGIC_FOOTABLLTEAMSTRATEGYGIMBAL_H__

#include <iostream>
#include <memory>

#include "Global.h"
#include "TeamStrategyGimbal.h"

namespace XbotgoSDK
{

class FootballTeamStrategyGimbal : public TeamStrategyGimbal
{
public:
    FootballTeamStrategyGimbal(const StrategyParam& param)
    {
        pitchPID = new PID(0.9, 0.0, 0.0, 1440, 15, 600, "Pitch");
        yawPID = new PID(1.0, 0.0, 0.0, 2560, 15, 600, "Yaw");
        yawSmoother = new Smoother(120, 40, 40, 80, "Yaw");
        this->param = param;
    }
    ~FootballTeamStrategyGimbal()
    {
        printfXbotGo("FootballTeamStrategyGimbal::~FootballTeamStrategyGimbal()\n");
    }

    int init() override;

    int dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults, std::vector<DelatTrackInfo>& filterMovedResults) override;

    int caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult) override;

    TrackResult caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults);

    int autoTracking(TrackResult& trackPoint, const std::vector<MOTResult>& motResult);

private:
    int autoTrackingFrameIntervals;
};

}

#endif