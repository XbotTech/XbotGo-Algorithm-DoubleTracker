#include <cmath>
#include <numeric>

#include "FootballTeamStrategyGimbal.h"
#include "XbotgoStrategyCallback.h"

namespace XbotgoSDK
{
/*策略描述：

*/
extern XbotgoStrategyCallback* g_xbotgoStrategyCallbacks;

int FootballTeamStrategyGimbal::init()
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
    // 变焦相关
    isZoom = false;
    scaleFactor = controllerManger->defalutZoom;
    defaultZoomCounter = 0;
    // 初始角度相关
    needPitchDown = controllerManger->needPitchDown; // 初始化时是否需要向下俯
    return 0;
}

int FootballTeamStrategyGimbal::dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults, std::vector<DelatTrackInfo>& filterMovedResults)
{
    float threshold = param.FastPeopleThreshold;
    std::vector<DelatTrackInfo> filter;
    for (const auto& obj : delatTrackInfoResults) {
        if (obj.distance > threshold) {
            filter.emplace_back(obj);
        }
    }
    int ret = 1;
    if (filter.size() >= 3) {
        std::sort(filter.begin(), filter.end(), [](const DelatTrackInfo& a, const DelatTrackInfo& b) { return a.distance > b.distance; });
        std::vector<DelatTrackInfo> fast(filter.begin(), filter.begin() + std::min(3, (int)filter.size()));
        filterTrackedResults = std::move(fast);
        return ret;
    }

    ret = -1;
    // 全部移动的人
    filter.clear();
    for (const auto& obj : delatTrackInfoResults) {
        if (obj.distance > 20) {
            filter.emplace_back(obj);
        }
    }
    if (filter.size() > 0) {
        std::sort(filter.begin(), filter.end(), [](const DelatTrackInfo& a, const DelatTrackInfo& b) { return a.distance > b.distance; });
        filterMovedResults = std::move(filter);
    }
    printfXbotGo("FootballTeamStrategyGimbal::dynamicFastPeopleFilter fllow all moved person delatTrackInfoResults.size()=%zu  filterMovedResults.size()=%zu\n", delatTrackInfoResults.size(), filterMovedResults.size());
    return ret;
}

int FootballTeamStrategyGimbal::caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult)
{
    printfXbotGo("FootballTeamStrategyGimbal::caculateBallTrackPoint trackResult input point.x=%.2f  point.y=%.2f strategy=%d direction=%d\n", trackResult.x, trackResult.y, trackResult.stratety, trackResult.direction);

    std::vector<BallDetectionResult> virtualBallDetectionResult;
    for (const auto& obj : yoloDetectResult) {
        if (obj.lable == 1) {
            // 左上角转到中心点
            int x = obj.x + obj.width / 2;
            int y = obj.y + obj.height / 2;
            YOLODetectionResult detectionOb(obj.lable, obj.prob, x, y, obj.width, obj.height);
            YOLODetectionResult virtulaDetectionResult = CoordinateTransform::coordinateToVirtual(detectionOb, currentGyroscope, controllerManger->deviceVideoBufferWidth, controllerManger->deviceVideoBufferHeight);

            BallDetectionResult ballResult(virtulaDetectionResult.lable, virtulaDetectionResult.prob, virtulaDetectionResult.x, virtulaDetectionResult.y, virtulaDetectionResult.width, virtulaDetectionResult.height);
            virtualBallDetectionResult.emplace_back(ballResult);
        }
    }

    int ret = 0;
    auto updateResult = ballTracker->update(virtualBallDetectionResult);
    int direction = updateResult.first;
    BallDetectionResult result = updateResult.second;
    if (result.is_empty()) {
        printfXbotGo("FootballTeamStrategyGimbal::caculateBallTrackPoint failed\n");
        ret = -1;
    } else {
        printfXbotGo("FootballTeamStrategyGimbal::caculateBallTrackPoint success\n");

        trackResult.x = result.x;
        trackResult.y = result.y;
        trackResult.stratety = 3;

        TrackPoint devicePoint = CoordinateTransform::coordinateToDeviceBuffer(TrackPoint(result.x, result.y), currentGyroscope, controllerManger->deviceVideoBufferWidth, controllerManger->deviceVideoBufferHeight);
        int offset = 50;
        printfXbotGo("FootballTeamStrategyGimbal::caculateBallTrackPoint ball device point.x=%.2f  point.y=%.2f prob=%.2f\n", devicePoint.x, devicePoint.y, result.prob);
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
        trackResult.speed = 150;

        printfXbotGo("FootballTeamStrategyGimbal::caculateBallTrackPoint trackResult output point.x=%.2f  point.y=%.2f strategy=%d direction=%d\n", trackResult.x, trackResult.y, trackResult.stratety, trackResult.direction);
    }

    return ret;
}

TrackResult FootballTeamStrategyGimbal::caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults)
{
    TrackResult trackPoint;
    std::vector<DelatTrackInfo> fastThreePeopleResults;
    std::vector<DelatTrackInfo> movedPeopleResults;
    int ret = 0;
    ret = dynamicFastPeopleFilter(delatTrackInfoResults, fastThreePeopleResults, movedPeopleResults);
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

    printfXbotGo("FootballTeamStrategyGimbal::caculateTrackPoint ret=%d delatTrackInfoResults.size()=%zu  fastThreePeopleResults.size()=%zu  movedPeopleResults.size()=%zu stayTrackResultTimes=%d\n", ret, delatTrackInfoResults.size(), fastThreePeopleResults.size(), movedPeopleResults.size(), stayTrackResultTimes);
    ret = caculateBallTrackPoint(yoloDetectResult, trackPoint);
    if (ret == 0) {
        lastPoint = {0, 0};
        stayTrackResultTimes = 0;
    }

    autoTracking(trackPoint, motResult);

    printfXbotGo("FootballTeamStrategyGimbal::caculateTrackPoint trackResult output point.x=%.2f  point.y=%.2f strategy=%d direction=%d speed=%.2f\n", trackPoint.x, trackPoint.y, trackPoint.stratety, trackPoint.direction, trackPoint.speed);

    return trackPoint;
}

int FootballTeamStrategyGimbal::autoTracking(TrackResult& trackPoint, const std::vector<MOTResult>& motResult)
{
    if (trackPoint.stratety > 0) {
        autoTrackingFrameIntervals = 0;
    } else {
        autoTrackingFrameIntervals++;
    }

    if (autoTrackingFrameIntervals > 120) {

        int deviceVideoBufferWidth = controllerManger->deviceVideoBufferWidth;
        int deviceVideoBufferHeight = controllerManger->deviceVideoBufferHeight;
        TrackPoint centralPoint = CoordinateTransform::coordinateToVirtual(TrackPoint(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2), currentGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);

        // 过滤极限角的人
        int offset = 0;
        std::vector<MOTResult> filterMotResult;
        for (const auto& obj : motResult) {
            TrackPoint point = CoordinateTransform::coordinateToDeviceBuffer(TrackPoint(obj.x, obj.y), currentGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);
            MOTResult mot = obj;
            mot.x = point.x;
            mot.y = point.y;
            if (currentGyroscope.yawAngle < -3) // 左边
            {
                offset = (deviceVideoBufferWidth / 2 - (90 + currentGyroscope.yawAngle) * 23) < 0 ? 0 : (deviceVideoBufferWidth / 2 - (90 + currentGyroscope.yawAngle) * 23);
                if (point.x > offset) {
                    filterMotResult.emplace_back(mot);
                }
            } else if (currentGyroscope.yawAngle > 3) // 右边
            {
                offset = (deviceVideoBufferWidth / 2 + (90 - currentGyroscope.yawAngle) * 23) > deviceVideoBufferWidth ? deviceVideoBufferWidth : (deviceVideoBufferWidth / 2 + (90 - currentGyroscope.yawAngle) * 23);
                if (point.x < offset) {
                    filterMotResult.emplace_back(mot);
                }
            }
        }

        // 按device坐标排序
        std::sort(filterMotResult.begin(), filterMotResult.end(), [](const MOTResult& a, const MOTResult& b) { return a.x < b.x; });

        // 如果没有人
        if (filterMotResult.size() > 0) {
            // 往左计算最左边人和分界线的距离来决定转动
            MOTResult left = filterMotResult.front();
            MOTResult right = filterMotResult.back();
            if (currentGyroscope.yawAngle < -3) {
                if (left.x > deviceVideoBufferWidth * 0.2) {
                    trackPoint.x = centralPoint.x + 100;
                    trackPoint.speed = 50;
                } else {
                    trackPoint.speed = 0;
                }

            } else if (currentGyroscope.yawAngle > 3) {
                if (right.x < deviceVideoBufferWidth * 0.8) {
                    trackPoint.x = centralPoint.x - 100;
                    trackPoint.speed = 50;
                } else {
                    trackPoint.speed = 0;
                }
            }
            trackPoint.stratety = 0;
            return 0;
        }

        // 回中
        if (currentGyroscope.yawAngle < -3) // 朝左就向右转
        {
            trackPoint.x = centralPoint.x + 100;
        } else if (currentGyroscope.yawAngle > 3) // 朝右就向左转
        {
            trackPoint.x = centralPoint.x - 100;
        }
        trackPoint.stratety = 0;
        trackPoint.speed = 50;
        printfXbotGo("FootballTeamStrategyGimbal::autoTracking trackResult.x =%.2f currentGyroscope.yawAngle=%.2f\n", trackPoint.x, currentGyroscope.yawAngle);
    }

    return 0;
}

}
