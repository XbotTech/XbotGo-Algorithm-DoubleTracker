#include <cmath>
#include <numeric>

#include "BasketballTeamStrategyGimbal.h"
#include "XbotgoStrategyCallback.h"

namespace XbotgoSDK
{
/*策略描述：

*/
extern XbotgoStrategyCallback* g_xbotgoStrategyCallbacks;

int BasketballTeamStrategyGimbal::init()
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
        ballTracker = new BallTracker(15, 5, 1600, 1600, 200);
    } else {
        delete ballTracker;
        ballTracker = new BallTracker(15, 5, 1600, 1600, 200);
    }
    // 追踪点相关
    lastPoint = {0, 0};
    frameSequence = 0;
    stayTrackResultTimes = 0;
    // autoTrackingFrameCount = 0;
    // 变焦相关
    isZoom = false;
    scaleFactor = controllerManger->defalutZoom;
    defaultZoomCounter = 0;
    // 初始角度相关
    needPitchDown = controllerManger->needPitchDown; // 初始化时是否需要向下俯
    return 0;
}

int BasketballTeamStrategyGimbal::dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults, std::vector<DelatTrackInfo>& filterMovedResults)
{
    float threshold = param.FastPeopleThreshold;
    std::vector<DelatTrackInfo> filter;
    for (const auto& obj : delatTrackInfoResults) {
        if (obj.distance > threshold) {
            filter.emplace_back(obj);
        }
    }
    int ret = 1;
    if (filter.size() >= 1) {
        std::sort(filter.begin(), filter.end(), [](const DelatTrackInfo& a, const DelatTrackInfo& b) { return a.distance > b.distance; });
        std::vector<DelatTrackInfo> fast(filter.begin(), filter.begin() + std::min(3, (int)filter.size()));
        filterTrackedResults = std::move(fast);

        return ret;
    }

    ret = -1;
    printfXbotGo("BasketballTeamStrategyGimbal::dynamicFastPeopleFilter  threshold=%.2f  delatTrackInfoResults.size()=%zu  filterTrackedResults.size()=%zu  filterMovedResults.size()=%zu\n", threshold, delatTrackInfoResults.size(), filterTrackedResults.size(), filterMovedResults.size());
    return ret;
}

int BasketballTeamStrategyGimbal::caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult)
{
    printfXbotGo("BasketballTeamStrategyGimbal::caculateBallTrackPoint trackResult input point.x=%.2f  point.y=%.2f strategy=%d direction=%d\n", trackResult.x, trackResult.y, trackResult.stratety, trackResult.direction);

    int deviceVideoBufferWidth = controllerManger->deviceVideoBufferWidth;
    int deviceVideoBufferHeight = controllerManger->deviceVideoBufferHeight;
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
        printfXbotGo("BasketballTeamStrategyGimbal::caculateBallTrackPoint failed direction=%d\n", direction);
        ret = -1;
    } else {
        printfXbotGo("BasketballTeamStrategyGimbal::caculateBallTrackPoint success direction=%d\n", direction);
        trackResult.x = result.x;
        trackResult.y = result.y;
        trackResult.stratety = 3;
        TrackPoint centralPoint = CoordinateTransform::coordinateToVirtual(TrackPoint(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2), currentGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);
        TrackPoint devicePoint = CoordinateTransform::coordinateToDeviceBuffer(TrackPoint(result.x, result.y), currentGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);
        printfXbotGo("BasketballTeamStrategyGimbal::caculateBallTrackPoint result.x=%.2f  result.y=%.2f devicePoint.x=%.2f  devicePoint.y=%.2f\n", result.x, result.y, devicePoint.x, devicePoint.y);
        if (direction < 0) {
            if (direction < -5) // 过1/3提前转
            {
                if (devicePoint.x > deviceVideoBufferWidth * 3 / 4) {
                    // not moved
                    trackResult.x = 0;
                } else {
                    trackResult.x = centralPoint.x + (devicePoint.x - deviceVideoBufferWidth * 3 / 4) * 0.5;
                }
            } else // 过中间点转
            {
                if (devicePoint.x > deviceVideoBufferWidth / 2) {
                    // not moved
                    trackResult.x = 0;
                } else {
                    trackResult.x = result.x;
                    trackResult.y = result.y;
                }
            }
        } else if (direction > 0) {
            if (direction > 5) // 过1/3提前转
            {
                if (devicePoint.x < deviceVideoBufferWidth / 4) {
                    // not moved
                    trackResult.x = 0;
                } else {
                    trackResult.x = centralPoint.x + (devicePoint.x - deviceVideoBufferWidth / 4) * 0.5;
                }
            } else {
                if (devicePoint.x < deviceVideoBufferWidth / 2) {
                    // not moved
                    trackResult.x = 0;
                } else {
                    trackResult.x = result.x;
                    trackResult.y = result.y;
                }
            }
        } else {
            trackResult.x = 0;
        }

        trackResult.direction = direction;
    }
    return ret;
}

TrackResult BasketballTeamStrategyGimbal::caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults)
{
    TrackResult trackPoint;
    std::vector<DelatTrackInfo> fastThreePeopleResults;
    std::vector<DelatTrackInfo> movedPeopleResults;
    int ret = 0;
    ret = dynamicFastPeopleFilter(delatTrackInfoResults, fastThreePeopleResults, movedPeopleResults);
    if (ret > 0) {
        trackPoint = caculateFastTrackPoint(fastThreePeopleResults);
        trackPoint.stratety = 1;
        lastPoint = trackPoint;
        stayTrackResultTimes = 0;
    } else {
        if (stayTrackResultTimes < 10 && lastPoint.x != 0) {
            stayTrackResultTimes++;
            trackPoint = lastPoint;
        }
    }
    ret = caculateBallTrackPoint(yoloDetectResult, trackPoint);
    if (ret == 0) {
        lastPoint = {0, 0};
        stayTrackResultTimes = 0;
    }

    printfXbotGo("BasketballTeamStrategyGimbal::caculateTrackPoint trackResult output point.x=%.2f  point.y=%.2f strategy=%d direction=%d\n", trackPoint.x, trackPoint.y, trackPoint.stratety, trackPoint.direction);
    return trackPoint;
}

}
