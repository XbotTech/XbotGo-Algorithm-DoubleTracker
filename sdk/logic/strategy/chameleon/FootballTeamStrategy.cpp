#include <cmath>
#include <numeric>

#include "FootballTeamStrategy.h"
#include "XbotgoStrategyCallback.h"

namespace XbotgoSDK
{
/*策略描述：

*/
extern XbotgoStrategyCallback* g_xbotgoStrategyCallbacks;

int FootballTeamStrategy::init()
{
    // 追踪人策略相关
    if (mot == nullptr) {
        mot = new MOT(param.motFPS);
    } else {
        delete mot;
        mot = new MOT(param.motFPS);
    }
    motResults.clear();
    // 追踪球策略相关
    if (ballTracker == nullptr) {
        ballTracker = new BallTracker(10, 5, 900, 900, 200);
    } else {
        delete ballTracker;
        ballTracker = new BallTracker(10, 5, 900, 900, 200);
    }
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
    printfXbotGo("TeamFootball:yawLimitAngle=%.2f FastPeopleThreshold=%d yoloAreaThreshold=%.2f pitchInitialAngle=%.2f\n", controllerManger->yawLimitAngle, param.FastPeopleThreshold, param.yoloAreaThreshold, controllerManger->pitchInitialAngle);
    return 0;
}

int FootballTeamStrategy::dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults)
{
    float threshold = param.FastPeopleThreshold;
    std::vector<DelatTrackInfo> filter;
    for (const auto& obj : delatTrackInfoResults) {
        if (obj.distance > threshold) {
            filter.emplace_back(obj);
        }
    }
    printfXbotGo("TeamFootball:filterFastPeopleSize=%zu", filter.size());
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

int FootballTeamStrategy::caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult)
{
    std::vector<BallDetectionResult> virtualBallDetectionResult;
    for (const auto& obj : yoloDetectResult) {
        if (obj.lable == 1) {
            // 左上角转到中心点
            int x = obj.x + obj.width / 2;
            int y = obj.y + obj.height / 2;
            YOLODetectionResult detectionOb(obj.lable, obj.prob, x, y, obj.width, obj.height);
            YOLODetectionResult virtulaDetectionResult = chamCalibrateXY->real2virXY(detectionOb, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);

            BallDetectionResult ballResult(virtulaDetectionResult.lable, virtulaDetectionResult.prob, virtulaDetectionResult.x, virtulaDetectionResult.y, virtulaDetectionResult.width, virtulaDetectionResult.height);
            virtualBallDetectionResult.emplace_back(ballResult);
        }
    }

    int ret = 0;
    auto updateResult = ballTracker->update(virtualBallDetectionResult);
    int direction = updateResult.first;
    BallDetectionResult result = updateResult.second;
    if (result.is_empty()) {
        printfXbotGo("TeamFootball:BallTrackFailed\n");
        ret = -1;
    } else {
        trackResult.x = result.x;
        trackResult.y = result.y;
        trackResult.stratety = 3;

        TrackPoint devicePoint = chamCalibrateXY->vir2realXY(result.x, result.y, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);
        int offset = 80;
        printfXbotGo("TeamFootball:BallTrackPoint device x=%.2f y=%.2f prob=%.2f\n", devicePoint.x, devicePoint.y, result.prob);
        if (direction < 0) {
            if (devicePoint.x > controllerManger->deviceVideoBufferWidth / 2 - offset) {
                // not moved
                trackResult.x = 0;
            } else {
                trackResult.x = result.x;
                trackResult.y = result.y;
            }
        } else if (direction > 0) {
            if (devicePoint.x < controllerManger->deviceVideoBufferWidth / 2 + offset) {
                // not moved
                trackResult.x = 0;
            } else {
                trackResult.x = result.x;
                trackResult.y = result.y;
            }
        } else {
            trackResult.x = 0;
        }
        trackResult.direction = direction;
        trackResult.speed = 200;

        printfXbotGo("TeamFootball:BallTrack output x=%.2f y=%.2f direction=%d\n", trackResult.x, trackResult.y, trackResult.direction);
    }
    return ret;
}

TrackResult FootballTeamStrategy::caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults)
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
    printfXbotGo("TeamFootball:PeopleTrackPoint output x=%.2f y=%.2f direction=%d stayTrackResultTimes=%d\n", trackPoint.x, trackPoint.y, trackPoint.direction, stayTrackResultTimes);

    ret = caculateBallTrackPoint(yoloDetectResult, trackPoint);
    if (ret == 0) {
        lastPoint = {0, 0};
        stayTrackResultTimes = 0;
    }
    autoTracking(trackPoint, motResult);

    printfXbotGo("TeamFootball:TrackPoint output x=%.2f y=%.2f strategy=%d direction=%d speed=%.2f\n", trackPoint.x, trackPoint.y, trackPoint.stratety, trackPoint.direction, trackPoint.speed);

    return trackPoint;
}

int FootballTeamStrategy::autoTracking(TrackResult& trackPoint, const std::vector<MOTResult>& motResult)
{
    if (trackPoint.stratety > 0) {
        autoTrackingFrameIntervals = 0;
    } else {
        autoTrackingFrameIntervals++;
    }

    if (autoTrackingFrameIntervals > 30) {

        int deviceVideoBufferWidth = controllerManger->deviceVideoBufferWidth;
        int deviceVideoBufferHeight = controllerManger->deviceVideoBufferHeight;
        TrackPoint centralPoint = chamCalibrateXY->real2virXY(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);

        printfXbotGo("TeamFootball:autoTracking");

        // 按device坐标排序
        std::vector<TrackPoint> filterMotResult;
        for (const auto& obj : motResult) {
            TrackPoint point = chamCalibrateXY->vir2realXY(obj.x, obj.y, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);
            filterMotResult.emplace_back(point);
        }
        std::sort(filterMotResult.begin(), filterMotResult.end(), [](const TrackPoint& a, const TrackPoint& b) { return a.x < b.x; });

        // 如果没有人
        if (filterMotResult.size() > 0) {
            // 往左计算最左边人和分界线的距离来决定转动
            TrackPoint left = filterMotResult.front();
            TrackPoint right = filterMotResult.back();
            if (currentGyroscope.yawAngle < -3) {
                if (left.x > deviceVideoBufferWidth * 0.25) {
                    trackPoint.x = centralPoint.x + 100;
                    trackPoint.y = centralPoint.y;
                    trackPoint.speed = 50;
                    printfXbotGo(" leftX =%.2f boundary =%.2f", trackPoint.x, deviceVideoBufferWidth * 0.25);
                } else {
                    trackPoint.speed = 0;
                }

            } else if (currentGyroscope.yawAngle > 3) {
                if (right.x < deviceVideoBufferWidth * 0.75) {
                    trackPoint.x = centralPoint.x - 100;
                    trackPoint.y = centralPoint.y;
                    trackPoint.speed = 50;
                    printfXbotGo(" rightX =%.2f boundary =%.2f", trackPoint.x, deviceVideoBufferWidth * 0.75);
                } else {
                    trackPoint.speed = 0;
                }
            } else {
                trackPoint.speed = 0;
            }
            trackPoint.stratety = 5;
            printfXbotGo(" x =%.2f stratety =%d\n", trackPoint.x, trackPoint.stratety);
        }

        else {
            // 回中
            if (currentGyroscope.yawAngle < -3) // 朝左就向右转
            {
                trackPoint.x = centralPoint.x + 100;
                trackPoint.y = centralPoint.y;
            } else if (currentGyroscope.yawAngle > 3) // 朝右就向左转
            {
                trackPoint.x = centralPoint.x - 100;
                trackPoint.y = centralPoint.y;
            }
            trackPoint.stratety = 6;
            trackPoint.speed = 50;

            printfXbotGo(" x =%.2f stratety =%d\n", trackPoint.x, trackPoint.stratety);
        }
    }

    return 0;
}

void FootballTeamStrategy::control(const TrackResult& trackResult)
{
    float x = trackResult.x;
    float y = trackResult.y;
    yawPID->setMaxOut(trackResult.speed * 2.5);
    float yawOut = yawPID->update(trackResult.x);
    yawOut = yawSmoother->smooth(yawOut);
    float pitchOut = pitchPID->Compute(trackResult.y);
    printfXbotGo("TeamFootball:control yawOut=%.2f pitchOut=%.2f x = %.2f y = %.2f\n", yawOut, pitchOut, x, y);
    g_xbotgoStrategyCallbacks->onControlSpeed(yawOut, pitchOut);
}

}
