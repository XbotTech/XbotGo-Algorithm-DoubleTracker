#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <numeric>
#include <thread>

#include "TeamStrategy.h"
#include "XbotgoStrategy.h"
#include "XbotgoStrategyCallback.h"

namespace XbotgoSDK
{
/*策略描述：

    base策略，跟最快的三个人的中心点

*/
extern XbotgoStrategyCallback* g_xbotgoStrategyCallbacks;

int TeamStrategy::init()
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
    printfXbotGo("Team:yawLimitAngle=%.2f FastPeopleThreshold=%d yoloAreaThreshold=%.2f pitchInitialAngle=%.2f\n", controllerManger->yawLimitAngle, param.FastPeopleThreshold, param.yoloAreaThreshold, controllerManger->pitchInitialAngle);
    return 0;
}

int TeamStrategy::track(const std::vector<YOLODetectionResult>& detectResult, double time_stamp)
{
    // 记录时间与输入的AI检测结果数量
    loggingTime(detectResult, time_stamp);
    // 记录当前上报角度值
    bestGyroscope = {controllerManger->yaw, controllerManger->pitch};
    printfXbotGo("Team:yaw=%.2f pitch=%.2f\n", bestGyroscope.yawAngle, bestGyroscope.pitchAngle);
    angleDeadZoneCompensation(bestGyroscope);
    printfXbotGo("Team:compensate yaw=%.2f pitch=%.2f\n", currentGyroscope.yawAngle, currentGyroscope.pitchAngle);

    // 判断是否启动追踪
    if (controllerManger->isTracking == false) {
        printfXbotGo("Team:SDK deactivate\n");
        return -1;
    }
    // 激活追踪
    else {
        // 判断角度是否处在极限角以外区间
        if (std::abs(currentGyroscope.yawAngle) > (controllerManger->yawLimitAngle / 2) + 3) {
            printfXbotGo("Team:out of limit yaw angle=%.2f\n", controllerManger->yawLimitAngle);

            TrackResult trackpoint;
            if (currentGyroscope.yawAngle > 0) {
                trackpoint.x = (controllerManger->deviceVideoBufferWidth / 2) - 200; // 当前朝向右，则向左转动回到右极限角附近
            } else {
                trackpoint.x = (controllerManger->deviceVideoBufferWidth / 2) + 200; // 当前朝向左，则向右转动回到左极限角附近
            }
            trackpoint.y = controllerManger->deviceVideoBufferHeight / 2;
            trackpoint.speed = 50;

            // g_xbotgoStrategyCallbacks->onTrackingPoint(trackpoint);
            control(trackpoint);
            return -1;
        }
        // 角度处在极限角区间内
        else {
            // 虚拟坐标转换
            std::vector<YOLODetectionResult> virtualYOLODetectionResult;
            for (const auto& obj : detectResult) {
                if (obj.lable == 0) {
                    YOLODetectionResult virtulaDetectionResult = chamCalibrateXY->real2virXY(obj, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);
                    virtualYOLODetectionResult.emplace_back(virtulaDetectionResult);
                }
            }
            // 过滤小于指定面积大小的人
            std::vector<YOLODetectionResult> virtualYOLODetectionResultFilter;
            followExcludeAreaFilter(virtualYOLODetectionResult, virtualYOLODetectionResultFilter);
            // 调用MOT
            std::vector<MOTResult> mot_result;
            mot->track(virtualYOLODetectionResultFilter, mot_result);
            if (mot_result.size() == 0) {
                printfXbotGo("Team:mot track back zero DetectionResult size=%zu DetectionResultFilter size=%zu\n", virtualYOLODetectionResult.size(), virtualYOLODetectionResultFilter.size());
            }
            // 策略处理
            strategyProcess(detectResult, mot_result);
            // 长期无识别结果自动回中
            // autoTracking(detectResult);
        }
    }

    return 0;
}

Gyroscope TeamStrategy::angleDeadZoneCompensation(Gyroscope& gyroscope)
{
    currentGyroscope = bestGyroscope;
    float offset = 2.0;

    if (currentGyroscope.yawAngle - lastGyroscope.yawAngle > 0.0) { // 当前帧向右转
        if (turnRight) {
            if (std::abs(currentGyroscope.yawAngle - turnAroundYawAngle) <= offset) {

                currentGyroscope.yawAngle = turnAroundYawAngle;
            } else {
                currentGyroscope.yawAngle += offset;
            }
        } else {
            if (std::abs(currentGyroscope.yawAngle - lastGyroscope.yawAngle) <= offset) {

                turnAroundYawAngle = lastGyroscope.yawAngle;
                currentGyroscope.yawAngle = turnAroundYawAngle;
            } else {
                currentGyroscope.yawAngle += offset;
            }
        }
        turnRight = true;

    } else if (currentGyroscope.yawAngle - lastGyroscope.yawAngle < 0.0) { // 当前帧向左转
        if (turnRight) {
            if (std::abs(currentGyroscope.yawAngle - lastGyroscope.yawAngle) <= offset) {
                turnAroundYawAngle = lastGyroscope.yawAngle; // 当前帧向左，上一帧向右
                currentGyroscope.yawAngle = turnAroundYawAngle - offset;
            }
        } else {
            if (std::abs(currentGyroscope.yawAngle - turnAroundYawAngle) <= offset) {
                currentGyroscope.yawAngle = turnAroundYawAngle - offset;
            }
        }
        turnRight = false;

    } else {
        if (turnRight) {
            currentGyroscope.yawAngle += offset;
        }
    }

    lastGyroscope = bestGyroscope;

    return currentGyroscope;
}

TrackResult TeamStrategy::caculateFastTrackPoint(const std::vector<DelatTrackInfo>& trackResults)
{
    TrackResult result; // 当前帧的跟踪点
    if (trackResults.size() == 0) {
        return result;
    }
    // 排序：最快三人移动方向和距离
    std::vector<DelatTrackInfo> sortResult = trackResults;
    std::sort(sortResult.begin(), sortResult.end(), [](const DelatTrackInfo& a, const DelatTrackInfo& b) { return a.x < b.x; });
    for (const auto& delatTrackInfo : sortResult) {
        printfXbotGo("Team:FastPeopleDeque distance= %.2f id=%d x=%.2f y=%.2f w=%.2f h=%.2f deltx=%.2f delty=%.2f\n", delatTrackInfo.distance, delatTrackInfo.trackId, delatTrackInfo.x, delatTrackInfo.y, delatTrackInfo.width, delatTrackInfo.height, delatTrackInfo.delaX, delatTrackInfo.delaY);
    }
    // 判断方向找 向左，都在右半边没过中心点 ，不动，否则跟最左边的人； 向右，都在左半边没过中心点，不动，否则跟最右边的人
    // 如果向左：如果最右边的人没过中心点，不动；跟最左边的人
    bool allLeft = std::all_of(sortResult.begin(), sortResult.end(), [](DelatTrackInfo& obj) { return obj.delaX < 0; });
    auto first = sortResult.front();
    auto last = sortResult.back();
    int deviceVideoBufferWidth = controllerManger->deviceVideoBufferWidth;
    int deviceVideoBufferHeight = controllerManger->deviceVideoBufferHeight;
    // 虚拟坐标转换
    TrackPoint pointFirst = chamCalibrateXY->vir2realXY(first.x, first.y, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);
    TrackPoint pointlast = chamCalibrateXY->vir2realXY(last.x, last.y, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);
    if (allLeft) {
        if (pointFirst.x > deviceVideoBufferWidth / 2 && pointlast.x > deviceVideoBufferWidth / 2) {
            //
        } else {
            result.x = first.x;
            result.y = first.y;
        }
        result.direction = -1;
        result.speed = first.delaX / 1.3 / 1.25;

        return result;
    }
    // 如果向右：如果最右边的人没有过中点，不转；如过了中点跟最右边的人
    bool allRight = std::all_of(sortResult.begin(), sortResult.end(), [](DelatTrackInfo& obj) { return obj.delaX > 0; });
    if (allRight) {
        if (pointFirst.x < deviceVideoBufferWidth / 2 && pointlast.x < deviceVideoBufferWidth / 2) {
            //
        } else {
            result.x = last.x;
            result.y = last.y;
        }
        result.direction = 1;
        result.speed = last.delaX / 1.3 / 1.25;
        return result;
    }
    // 没有方向取中心点
    double centerX = 0.0;
    double centerY = 0.0;
    float speedtotalX = 0;
    float speedtotalY = 0;
    for (const auto& point : sortResult) {
        centerX += point.x;
        centerY += point.y;
        speedtotalX += point.delaX;
        speedtotalY += point.delaY;
    }
    centerX /= sortResult.size();
    centerY /= sortResult.size();
    speedtotalX /= sortResult.size();
    speedtotalY /= sortResult.size();
    TrackPoint centralPoint = chamCalibrateXY->real2virXY(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);
    if (speedtotalX < 0) // 向左
    {
        if (centerX > centralPoint.x) {
            //
        } else {
            result.x = centerX;
        }
    } else if (speedtotalX > 0) {
        if (centerX < centralPoint.x) {
            //
        } else {
            result.x = centerX;
        }
    }

    if (speedtotalY < 0) // 向上
    {
        if (centerY > centralPoint.y) {
            //
        } else {
            result.y = centerY;
        }
    } else if (speedtotalY > 0) {
        if (centerY < centralPoint.y) {
            //
        } else {
            result.y = centerY;
        }
    }

    result.direction = 0;
    result.speed = speedtotalX / 1.3 / 1.25;
    return result;
}

int TeamStrategy::strategyProcess(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult)
{
    if (static_cast<int>(motResults.size()) >= param.frameInterval) {
        motResults.erase(motResults.begin());
    }
    motResults.emplace_back(motResult);
    // 计算跟踪点
    TrackResult trackPoint;
    std::vector<DelatTrackInfo> delatTrackInfoResults;
    caculatePeopleDistanceInQueue(motResults, delatTrackInfoResults);
    trackPoint = caculateTrackPoint(yoloDetectResult, motResult, delatTrackInfoResults);

    zoomTrackPoint(motResult, trackPoint);
    clampingLimiteAnger(trackPoint);
    adjustPitchAngle(trackPoint);
    printfXbotGo("Team:device x=%.2f y=%.2f strategy=%d direction=%d speed=%.2f\n", trackPoint.x, trackPoint.y, trackPoint.stratety, trackPoint.direction, trackPoint.speed);

    // g_xbotgoStrategyCallbacks->onTrackingPoint(trackPoint);
    control(trackPoint);
    return 0;
}

int TeamStrategy::clampingLimiteAnger(TrackResult& trackResult)
{
    int deviceVideoBufferWidth = controllerManger->deviceVideoBufferWidth;
    int deviceVideoBufferHeight = controllerManger->deviceVideoBufferHeight;
    if (trackResult.x == 0) {
        trackResult.x = deviceVideoBufferWidth / 2;
    } else {
        // 钳位控制算法
        Gyroscope leftGyroscope = {-controllerManger->yawLimitAngle / 2, 0};
        TrackPoint limitLeft = chamCalibrateXY->real2virXY(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2, leftGyroscope.yawAngle, leftGyroscope.pitchAngle);
        Gyroscope rightGyroscope = {controllerManger->yawLimitAngle / 2, 0};
        TrackPoint limitRight = chamCalibrateXY->real2virXY(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2, rightGyroscope.yawAngle, rightGyroscope.pitchAngle);
        if (trackResult.x < 0 && trackResult.x <= limitLeft.x) {
            trackResult.x = limitLeft.x;
        } else if (trackResult.x > 0 && trackResult.x >= limitRight.x) {
            trackResult.x = limitRight.x;
        }
        TrackPoint point = chamCalibrateXY->vir2realXY(trackResult.x, 0, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);
        trackResult.x = point.x;
    }

    trackResult.y = deviceVideoBufferHeight / 2;

    return 0;
}

int TeamStrategy::zoomTrackPoint(const std::vector<MOTResult>& motResults, const TrackResult& trackResult)
{
    printfXbotGo("Team:zoom");
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
            if (defaultZoomCounter >= 60) {
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

void TeamStrategy::adjustPitchAngle(TrackResult& trackResult)
{
    if (needPitchDown == true) {
        float angleDiff = currentGyroscope.pitchAngle - controllerManger->pitchInitialAngle;
        if (std::abs(angleDiff) < 0.5) {
            completePitchDownCount++;
            printfXbotGo("Team:Complete initial Pitch adjustment Count=%d\n", completePitchDownCount);
            if (completePitchDownCount >= 5) {
                needPitchDown = false;
            }
        } else {
            // pitch轴角度上负下正
            trackResult.y = (controllerManger->deviceVideoBufferHeight / 2) - (angleDiff * 25); // 云台一度位移约25像素
            completePitchDownCount = 0;
            printfXbotGo("Team:adjust initial Pitch Angle trackPoint.y=%.2f\n", trackResult.y);
        }
    }
}

int TeamStrategy::dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults)
{
    float threshold = param.FastPeopleThreshold;
    std::vector<DelatTrackInfo> filter;
    for (const auto& obj : delatTrackInfoResults) {
        if (obj.distance > threshold) {
            filter.emplace_back(obj);
        }
    }
    printfXbotGo("Team:filterFastPeopleSize=%zu\n", filter.size());
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

/*
int TeamStrategy::autoTracking(const std::vector<YOLODetectionResult>& detectResult)
{
    if (detectResult.size() > 0) {
        autoTrackingFrameCount = 0;
    } else {
        autoTrackingFrameCount++;
    }

    // 按最高帧率30帧每秒，900帧约30秒
    if ((autoTrackingFrameCount > 900) && std::abs(currentGyroscope.yawAngle) < controllerManger->yawLimitAngle / 2) {

        TrackResult trackResult(controllerManger->deviceVideoBufferWidth / 2, controllerManger->deviceVideoBufferHeight / 2);

        if (currentGyroscope.yawAngle < -3) // 朝左
        {
            trackResult.x = trackResult.x + 100;
        } else if (currentGyroscope.yawAngle > 3) // 朝右
        {
            trackResult.x = trackResult.x - 100;
        }

        trackResult.stratety = 0;
        g_xbotgoStrategyCallbacks->onTrackingPoint(trackResult);
        // control(trackResult);

        printfXbotGo("Team:Timeout back to center trackPoint.x=%.2f\n", trackResult.x);
    }
    return 0;
}
*/

int TeamStrategy::caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult)
{
    printfXbotGo("Team:flowBall\n");
    return 0;
}

int TeamStrategy::followExcludeAreaFilter(const std::vector<YOLODetectionResult>& detectResult, std::vector<YOLODetectionResult>& filteResult)
{
    float threshold = param.yoloAreaThreshold;
    for (const auto& obj : detectResult) {
        float objArea = obj.width * obj.height;
        if (objArea > threshold) {
            filteResult.emplace_back(obj);
        }
    }
    return 0;
}

int TeamStrategy::caculatePeopleDistanceInQueue(const std::vector<std::vector<MOTResult>>& motResults, std::vector<DelatTrackInfo>& delatTrackInfoResults)
{
    std::map<int, std::vector<MOTResult>> tid;
    for (const auto& mot : motResults) {
        for (const auto& person : mot) {
            auto it = tid.find(person.Track_id);
            if (it != tid.end()) {
                it->second.emplace_back(person);
            } else {
                std::vector<MOTResult> motPersons;
                motPersons.emplace_back(person);
                tid[person.Track_id] = motPersons;
            }
        }
    }

    std::vector<DelatTrackInfo> tempResult;
    // 计算每个人的移动距离
    for (const auto& pair : tid) {
        // int trackId = pair.first;
        std::vector<MOTResult> motPersons = pair.second;
        MOTResult first = motPersons.front();
        MOTResult last = motPersons.back();
        // float distance = std::abs(last.x-first.x);
        float distance = hypotf(((last.x + last.width / 2) - (first.x + first.width / 2)), (last.y + last.height / 2) - (first.y + first.height / 2)) / last.height * 100;
        float deltaX = last.x - first.x;
        float deltaY = last.y - first.y;
        DelatTrackInfo delatTrackInfo = {last.Track_id, last.x, last.y, last.width, last.height, last.Score, deltaX, deltaY, distance};
        tempResult.emplace_back(delatTrackInfo);
        // printfXbotGo("Team:PeopleDeque distance=%.2f id=%d x=%.2f y=%.2f w=%.2f h=%.2f deltx=%.2f delty=%.2f\n", distance, delatTrackInfo.trackId, delatTrackInfo.x, delatTrackInfo.y, delatTrackInfo.width, delatTrackInfo.height, delatTrackInfo.delaX, delatTrackInfo.delaY);
    }
    delatTrackInfoResults = std::move(tempResult);
    printfXbotGo("Team:PeopleDequeSize=%zu\n", delatTrackInfoResults.size());
    return 0;
}

TrackResult TeamStrategy::caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults)
{
    TrackResult trackPoint;
    std::vector<DelatTrackInfo> fastThreePeopleResults;
    int ret = 0;
    ret = dynamicFastPeopleFilter(delatTrackInfoResults, fastThreePeopleResults);
    if (ret > 0) {
        trackPoint = caculateFastTrackPoint(fastThreePeopleResults);
        trackPoint.stratety = 1;
        lastPoint = trackPoint;
        stayTrackResultTimes = 0;
    } else {
        if (stayTrackResultTimes < 10 && lastPoint.x != 0) // 如果没找到最快的三个人，持续追踪当前最快的三个10帧
        {
            stayTrackResultTimes++;
            trackPoint = lastPoint;
        }
    }

    printfXbotGo("Team:TrackPoint output x=%.2f y=%.2f direction=%d stayTrackResultTimes=%d\n", trackPoint.x, trackPoint.y, trackPoint.direction, stayTrackResultTimes);
    return trackPoint;
}

void TeamStrategy::control(const TrackResult& trackResult)
{
    float x = trackResult.x;
    float y = trackResult.y;
    float yawOut = yawPID->update(trackResult.x);
    yawOut = yawSmoother->smooth(yawOut);
    float pitchOut = pitchPID->Compute(trackResult.y);
    printfXbotGo("Team:control yawOut=%.2f pitchOut=%.2f x = %.2f y = %.2f\n", yawOut, pitchOut, x, y);
    g_xbotgoStrategyCallbacks->onControlSpeed(yawOut, pitchOut);
}

void TeamStrategy::loggingTime(const std::vector<YOLODetectionResult>& detectResult, double time_stamp)
{
    printfXbotGo("\n");
    frameSequence++;

    std::tm time_info;

    // 使用 strftime 格式化日期时间
    char time_str[40];

    // 获取当前时间
    if (time_stamp == 0.0f) {
        auto now = std::chrono::system_clock::now();
        auto now_since_epoch = now.time_since_epoch();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now_since_epoch) % 1000000;
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        time_info = *std::localtime(&now_c);
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &time_info);
        // 包含日期、时间和微秒
        printfXbotGo("Team:time=%s.%02lld\n", time_str, micros.count());
    } else {
        std::time_t time_c = (int)time_stamp;
        time_info = *std::localtime(&time_c);
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &time_info);
        // 包含日期、时间和微秒
        printfXbotGo("Team:time=%s %f\n", time_str, time_stamp - (int)time_stamp);
    }

    // 包含 frameSequence 和 detectResult size
    printfXbotGo("Team:-frameSeq=%ld- detectResult size=%zu\n", frameSequence, detectResult.size());
}

int TeamStrategy::setYawPIDMaxSpeed(int maxSpeed)
{
    yawMaxSpeed = maxSpeed;
    return 0;
}

}
