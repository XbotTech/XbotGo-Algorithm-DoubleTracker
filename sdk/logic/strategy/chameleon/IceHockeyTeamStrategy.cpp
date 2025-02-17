#include "IceHockeyTeamStrategy.h"
#include "XbotgoStrategyCallback.h"
#include <cmath>
#include <numeric>

namespace XbotgoSDK
{
/*策略描述：

*/
extern XbotgoStrategyCallback* g_xbotgoStrategyCallbacks;

int IceHockeyTeamStrategy::init()
{
    // 追踪人策略相关
    if (mot == nullptr) {
        mot = new MOT(param.motFPS);
    } else {
        delete mot;
        mot = new MOT(param.motFPS);
    }
    motResults.clear();
    // 追踪点相关
    lastPoint = {0, 0};
    frameSequence = 0;
    stayTrackResultTimes = 0;
    // autoTrackingFrameCount = 0;
    autoTrackingFrameIntervals = 0;
    // 坐标转换相关
    if (chamCalibrateXY == nullptr) {
        chamCalibrateXY = new ChamCalibrateXY(controllerManger->deviceVideoBufferWidth, controllerManger->deviceVideoBufferHeight);
    }
    // 变焦相关
    isZoom = false;
    scaleFactor = controllerManger->defalutZoom;
    defaultZoomCounter = 0;
    // 初始角度相关
    needPitchDown = controllerManger->needPitchDown; // 初始化时是否需要向下俯
    completePitchDownCount = 0;
    printfXbotGo("TeamIceHockey:yawLimitAngle=%.2f FastPeopleThreshold=%d yoloAreaThreshold=%.2f pitchInitialAngle=%.2f\n", controllerManger->yawLimitAngle, param.FastPeopleThreshold, param.yoloAreaThreshold, controllerManger->pitchInitialAngle);
    return 0;
}

int IceHockeyTeamStrategy::dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults)
{
    float threshold = param.FastPeopleThreshold;
    std::vector<DelatTrackInfo> filter;
    for (const auto& obj : delatTrackInfoResults) {
        if (obj.distance > threshold) {
            filter.emplace_back(obj);
        }
    }
    printfXbotGo("TeamIceHockey:filterFastPeopleSize=%zu", filter.size());
    int ret = 1;
    if (filter.size() >= 3) {
        std::sort(filter.begin(), filter.end(), [](const DelatTrackInfo& a, const DelatTrackInfo& b) { return a.distance > b.distance; });
        std::vector<DelatTrackInfo> fast(filter.begin(), filter.begin() + std::min(3, (int)filter.size()));
        filterTrackedResults = std::move(fast);
        printfXbotGo(" fastPeopleSize=%zu", filterTrackedResults.size());
    } else {
        ret = -1;
        printfXbotGo(" FastPeoplethreshold=%.2f", threshold);
    }
    printfXbotGo("\n");
    return ret;
}

TrackResult IceHockeyTeamStrategy::caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults)
{
    TrackResult trackPoint;
    std::vector<DelatTrackInfo> fastThreePeopleResults;
    int ret = 0;
    ret = dynamicFastPeopleFilter(delatTrackInfoResults, fastThreePeopleResults);
    if (ret > 0) {
        if (fastThreePeopleResults.size() > 0) {
            trackPoint = caculateFastTrackPoint(fastThreePeopleResults);
            trackPoint.stratety = 1;
            lastPoint = trackPoint;
            stayTrackResultTimes = 0;
        }
    } else {
        if (stayTrackResultTimes < 10 && lastPoint.x != 0) {
            stayTrackResultTimes++;
            trackPoint = lastPoint;
        }
    }
    autoTracking(trackPoint, motResult);

    printfXbotGo("TeamIceHockey:TrackPoint output x=%.2f y=%.2f direction=%d stayTrackResultTimes=%d\n", trackPoint.x, trackPoint.y, trackPoint.direction, stayTrackResultTimes);
    return trackPoint;
}

int IceHockeyTeamStrategy::zoomTrackPoint(const std::vector<MOTResult>& motResults, const TrackResult& trackResult)
{
    printfXbotGo("TeamIceHockey:zoom");
    int deviceVideoBufferWidth = controllerManger->deviceVideoBufferWidth;
    int deviceVideoBufferHeight = controllerManger->deviceVideoBufferHeight;
    if (std::abs(currentGyroscope.yawAngle) >= ZOOM_LIMITE_UPPER_ANGEL) {
        TrackPoint centerPoint = chamCalibrateXY->real2virXY(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);

        std::vector<MOTResult> filterMOTResults;
        for (const auto& obj : motResults) {
            if (std::abs(obj.x - centerPoint.x) < ZOOM_LIMITE_OFFSET) {
                filterMOTResults.emplace_back(obj);
            }
        }
        if (filterMOTResults.size() >= ZOOM_LIMITE_UPPER_PERSON_NUM) {
            if (defaultZoomCounter >= 100) {
                if (isZoom == false) {
                    scaleFactor = controllerManger->maxZoom;
                    isZoom = true;
                }
            }
        } else if (filterMOTResults.size() <= ZOOM_LIMITE_LOWER_PERSON_NUM) {
            if (isZoom == true) {
                scaleFactor = controllerManger->defalutZoom;
                defaultZoomCounter = 0;
                isZoom = false;
            }
        }
        defaultZoomCounter++;
        printfXbotGo(" filterResultsSize= %zu", filterMOTResults.size());
    } else if (std::abs(currentGyroscope.yawAngle) <= ZOOM_LIMITE_LOWER_ANGEL) {
        defaultZoomCounter++;
        if (isZoom == true) {
            scaleFactor = controllerManger->defalutZoom;
            defaultZoomCounter = 0;
            isZoom = false;
        }
    }

    printfXbotGo(" scaleFactor=%.2f maxZoom=%.2f Counter=%d\n", scaleFactor, controllerManger->maxZoom, defaultZoomCounter);

    g_xbotgoStrategyCallbacks->onTrackingZoom(scaleFactor);

    return 0;
}

int IceHockeyTeamStrategy::autoTracking(TrackResult& trackPoint, const std::vector<MOTResult>& motResult)
{
    if (trackPoint.stratety > 0) {
        autoTrackingFrameIntervals = 0;
    } else {
        autoTrackingFrameIntervals++;
    }

    if (autoTrackingFrameIntervals > 80) {

        TrackPoint centralPoint = chamCalibrateXY->real2virXY(controllerManger->deviceVideoBufferWidth / 2, controllerManger->deviceVideoBufferHeight / 2, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);

        printfXbotGo("TeamIceHockey:autoTracking");

        if (motResult.size() > 3) {

            int leftSize = 0;
            int middleSize = 0;
            int rightSize = 0;

            for (const auto& obj : motResult) {
                TrackPoint point = chamCalibrateXY->vir2realXY(obj.x, obj.y, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);
                if (point.x < (controllerManger->deviceVideoBufferWidth / 3) + 100) {
                    leftSize++;
                } else if (point.x > (controllerManger->deviceVideoBufferWidth / 3 * 2) - 200) {
                    rightSize++;
                } else {
                    middleSize++;
                }
            }

            if (leftSize > middleSize && leftSize > rightSize) {
                trackPoint.x = centralPoint.x - 100;
                trackPoint.y = centralPoint.y;
                trackPoint.speed = 75;
            } else if (rightSize > middleSize && rightSize > leftSize) {
                trackPoint.x = centralPoint.x + 100;
                trackPoint.y = centralPoint.y;
                trackPoint.speed = 75;
            } else {
                trackPoint.speed = 0;
            }
            trackPoint.stratety = 5;

            printfXbotGo(" x =%.2f stratety =%d leftSize =%d middleSize =%d rightSize =\n", trackPoint.x, trackPoint.stratety, leftSize, middleSize, rightSize);
        }

        else {
            if (currentGyroscope.yawAngle < -3) // 朝左
            {
                trackPoint.x = centralPoint.x + 100;
                trackPoint.y = centralPoint.y;
            } else if (currentGyroscope.yawAngle > 3) // 朝右
            {
                trackPoint.x = centralPoint.x - 100;
                trackPoint.y = centralPoint.y;
            }
            trackPoint.stratety = 6;
            trackPoint.speed = 75;

            printfXbotGo(" x =%.2f stratety =%d\n", trackPoint.x, trackPoint.stratety);
        }
    }

    return 0;
}

void IceHockeyTeamStrategy::control(const TrackResult& trackResult)
{
    float x = trackResult.x;
    float y = trackResult.y;
    yawPID->setMaxOut(trackResult.speed * 2.5);
    float yawOut = yawPID->update(trackResult.x);
    yawOut = yawSmoother->smooth(yawOut);
    float pitchOut = pitchPID->Compute(trackResult.y);
    printfXbotGo("TeamIceHockey:control yawOut=%.2f pitchOut=%.2f x = %.2f y = %.2f\n", yawOut, pitchOut, x, y);
    g_xbotgoStrategyCallbacks->onControlSpeed(yawOut, pitchOut);
}

}
