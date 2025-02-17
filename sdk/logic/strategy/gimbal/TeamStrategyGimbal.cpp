#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <numeric>
#include <thread>

#include "TeamStrategyGimbal.h"
#include "XbotgoStrategy.h"
#include "XbotgoStrategyCallback.h"

#define ZOOM_LIMITE_UPPER_ANGEL 20
#define ZOOM_LIMITE_LOWER_ANGEL 10
#define ZOOM_LIMITE_OFFSET 450
#define ZOOM_LIMITE_UPPER_PERSON_NUM 6
#define ZOOM_LIMITE_LOWER_PERSON_NUM 3

namespace XbotgoSDK
{
/*策略描述：

    base策略，跟最快的三个人的中心点

*/
extern XbotgoStrategyCallback* g_xbotgoStrategyCallbacks;

int TeamStrategyGimbal::init()
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
    // 变焦相关
    isZoom = false;
    scaleFactor = controllerManger->defalutZoom;
    defaultZoomCounter = 0;
    // 初始角度相关
    needPitchDown = controllerManger->needPitchDown; // 初始化时是否需要向下俯
    return 0;
}

int TeamStrategyGimbal::track(const std::vector<YOLODetectionResult>& detectResult, double time)
{
    // 记录时间与输入的AI检测结果数量
    loggingTime(detectResult, time);
    // 记录当前上报角度值
    currentGyroscope = {controllerManger->yaw, controllerManger->pitch};
    printfXbotGo("TeamStrategyGimbal:track currentGyroscope yaw=%.2f pitch=%.2f\n", currentGyroscope.yawAngle, currentGyroscope.pitchAngle);

    // 判断是否启动追踪
    if (controllerManger->isTracking == false) {
        printfXbotGo("Xbotgo SDK deactivate\n");
        return -1;
    }
    // 激活追踪
    else {
        // 判断角度是否处在极限角以外区间
        if (std::abs(currentGyroscope.yawAngle) > (controllerManger->yawLimitAngle / 2) + 3) {
            printfXbotGo("TeamStrategy:out of limit angle\n");

            TrackResult trackpoint;
            if (currentGyroscope.yawAngle > 0) {
                trackpoint.x = (controllerManger->deviceVideoBufferWidth / 2) - 200; // 当前朝向右，则向左转动回到右极限角附近
            } else {
                trackpoint.x = (controllerManger->deviceVideoBufferWidth / 2) + 200; // 当前朝向左，则向右转动回到左极限角附近
            }
            trackpoint.y = controllerManger->deviceVideoBufferHeight / 2;
            trackpoint.speed = 50;

            g_xbotgoStrategyCallbacks->onTrackingPoint(trackpoint);
            return -1;
        }
        // 角度处在极限角区间内
        else {
            // 虚拟坐标转换
            std::vector<YOLODetectionResult> virtualYOLODetectionResult;
            for (const auto& obj : detectResult) {
                if (obj.lable == 0) {
                    YOLODetectionResult virtulaDetectionResult = CoordinateTransform::coordinateToVirtual(obj, currentGyroscope, controllerManger->deviceVideoBufferWidth, controllerManger->deviceVideoBufferHeight);
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
                printfXbotGo("TeamStrategyGimbal:track mot->track back zero virtulaDetectionResult size=%zu virtualYOLODetectionResultFilter size=%zu\n", virtualYOLODetectionResult.size(), virtualYOLODetectionResultFilter.size());
            }
            // 策略处理
            strategyProcess(detectResult, mot_result);
            // 长期无识别结果自动回中
            // autoTracking(detectResult);
        }
    }

    return 0;
}

TrackResult TeamStrategyGimbal::caculateFastTrackPoint(const std::vector<DelatTrackInfo>& trackResults)
{
    TrackResult result; // 当前帧的跟踪点
    if (trackResults.size() == 0) {
        return result;
    }
    // 排序：最快三人移动方向和距离
    std::vector<DelatTrackInfo> sortResult = trackResults;
    std::sort(sortResult.begin(), sortResult.end(), [](const DelatTrackInfo& a, const DelatTrackInfo& b) { return a.x < b.x; });
    for (const auto& delatTrackInfo : sortResult) {
        printfXbotGo("TeamStrategyGimbal:caculateFastTrackPoint distance= %.2f track_id=%d x=%.2f y=%.2f w=%.2f h=%.2f deltx=%.2f delty=%.2f\n", delatTrackInfo.distance, delatTrackInfo.trackId, delatTrackInfo.x, delatTrackInfo.y, delatTrackInfo.width, delatTrackInfo.height, delatTrackInfo.delaX, delatTrackInfo.delaY);
    }
    // 判断方向找 向左，都在右半边没过中心点 ，不动，否则跟最左边的人； 向右，都在左半边没过中心点，不动，否则跟最右边的人
    // 如果向左：如果最右边的人没过中心点，不动；跟最左边的人
    bool allLeft = std::all_of(sortResult.begin(), sortResult.end(), [](DelatTrackInfo& obj) { return obj.delaX < 0; });
    auto first = sortResult.front();
    auto last = sortResult.back();
    int deviceVideoBufferWidth = controllerManger->deviceVideoBufferWidth;
    int deviceVideoBufferHeight = controllerManger->deviceVideoBufferHeight;
    // 虚拟坐标转换
    TrackPoint pointFirst = CoordinateTransform::coordinateToDeviceBuffer(TrackPoint(first.x, first.y), currentGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);
    TrackPoint pointlast = CoordinateTransform::coordinateToDeviceBuffer(TrackPoint(last.x, last.y), currentGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);
    if (allLeft) {
        if (pointFirst.x > deviceVideoBufferWidth / 2 && pointlast.x > deviceVideoBufferWidth / 2) {
            //
        } else {
            result.x = first.x;
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
        }
        result.direction = 1;
        result.speed = last.delaX / 1.3 / 1.25;
        return result;
    }
    // 没有方向取中心点
    double centerX = 0.0;
    // double centerY = 0.0;
    float speedtotal = 0;
    for (const auto& point : sortResult) {
        centerX += point.x;
        speedtotal += point.delaX;
    }
    centerX /= sortResult.size();
    speedtotal /= sortResult.size();
    TrackPoint centralPoint = CoordinateTransform::coordinateToVirtual(TrackPoint(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2), currentGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);
    if (speedtotal < 0) // 向左
    {
        if (centerX > centralPoint.x) {
            //
        } else {
            result.x = centerX;
        }
    } else if (speedtotal > 0) {
        if (centerX < centralPoint.x) {
            //
        } else {
            result.x = centerX;
        }
    }
    result.direction = 0;
    result.speed = speedtotal / 1.3 / 1.25;
    return result;
}

int TeamStrategyGimbal::strategyProcess(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult)
{
    printfXbotGo("TeamStrategyGimbal:param yawLimitAngle=%.2f FastPeopleThreshold=%d yoloAreaThreshold=%.2f pitchInitialAngle=%.2f\n", controllerManger->yawLimitAngle, param.FastPeopleThreshold, param.yoloAreaThreshold, controllerManger->pitchInitialAngle);

    if (static_cast<int>(motResults.size()) >= param.frameInterval) {
        motResults.erase(motResults.begin());
    }
    motResults.emplace_back(motResult);
    // 计算跟踪点
    TrackResult trackPoint;
    // int ret = 0;
    if (static_cast<int>(motResults.size()) != param.frameInterval) {
        // 前N帧数据
        return 1;
    }
    std::vector<DelatTrackInfo> delatTrackInfoResults;
    caculatePeopleDistanceInQueue(motResults, delatTrackInfoResults);
    trackPoint = caculateTrackPoint(yoloDetectResult, motResult, delatTrackInfoResults);
    printfXbotGo("TeamStrategyGimbal:strategyProcess virtual      trackpoint.x=%.2f trackpoint.y=%.2f strategy=%d direction=%d speed=%.2f  yaw=%.2f  pitch=%.2f\n", trackPoint.x, trackPoint.y, trackPoint.stratety, trackPoint.direction, trackPoint.speed, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);

    zoomTrackPoint(motResult, trackPoint);
    clampingLimiteAnger(trackPoint);
    printfXbotGo("TeamStrategyGimbal:strategyProcess devicebuffer trackpoint.x=%.2f trackpoint.y=%.2f strategy=%d direction=%d speed=%.2f  yaw=%.2f  pitch=%.2f\n", trackPoint.x, trackPoint.y, trackPoint.stratety, trackPoint.direction, trackPoint.speed, currentGyroscope.yawAngle, currentGyroscope.pitchAngle);

    g_xbotgoStrategyCallbacks->onTrackingPoint(trackPoint);
    // control(trackPoint);
    return 0;
}

int TeamStrategyGimbal::clampingLimiteAnger(TrackResult& trackResult)
{
    int deviceVideoBufferWidth = controllerManger->deviceVideoBufferWidth;
    int deviceVideoBufferHeight = controllerManger->deviceVideoBufferHeight;
    if (trackResult.x == 0) {
        trackResult.x = deviceVideoBufferWidth / 2;
    } else {
        // 钳位控制算法
        Gyroscope leftGyroscope = {-controllerManger->yawLimitAngle / 2, currentGyroscope.pitchAngle};
        TrackPoint limitLeft = CoordinateTransform::coordinateToVirtual(TrackPoint(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2), leftGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);

        Gyroscope rightGyroscope = {controllerManger->yawLimitAngle / 2, currentGyroscope.pitchAngle};
        TrackPoint limitRight = CoordinateTransform::coordinateToVirtual(TrackPoint(deviceVideoBufferWidth / 2, deviceVideoBufferHeight / 2), rightGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);

        if (trackResult.x < 0 && trackResult.x <= limitLeft.x) {
            trackResult.x = limitLeft.x;
        } else if (trackResult.x > 0 && trackResult.x >= limitRight.x) {
            trackResult.x = limitRight.x;
        }
        TrackPoint point = CoordinateTransform::coordinateToDeviceBuffer(TrackPoint(trackResult.x, 0), currentGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);
        trackResult.x = point.x;
    }
    if (trackResult.y == 0) {
        trackResult.y = deviceVideoBufferHeight / 2;
    } else {
        // trackResult.y = point.y;
    }

    return 0;
}

int TeamStrategyGimbal::zoomTrackPoint(const std::vector<MOTResult>& motResults, const TrackResult& trackResult)
{
    int deviceVideoBufferWidth = controllerManger->deviceVideoBufferWidth;
    int deviceVideoBufferHeight = controllerManger->deviceVideoBufferHeight;
    if (std::abs(currentGyroscope.yawAngle) >= ZOOM_LIMITE_UPPER_ANGEL) {

        TrackPoint centerPoint = CoordinateTransform::coordinateToVirtual(TrackPoint(deviceVideoBufferWidth / 2, deviceVideoBufferWidth / 2), currentGyroscope, deviceVideoBufferWidth, deviceVideoBufferHeight);

        std::vector<MOTResult> filterMOTResults;
        for (const auto& obj : motResults) {
            if (std::abs(obj.x - centerPoint.x) < ZOOM_LIMITE_OFFSET) {
                filterMOTResults.emplace_back(obj);
            }
        }
        if (filterMOTResults.size() >= ZOOM_LIMITE_UPPER_PERSON_NUM) {
            scaleFactor = controllerManger->maxZoom;
        } else if (filterMOTResults.size() <= ZOOM_LIMITE_LOWER_PERSON_NUM) {
            scaleFactor = controllerManger->defalutZoom;
        }

    } else if (std::abs(currentGyroscope.yawAngle) <= ZOOM_LIMITE_LOWER_ANGEL) {
        scaleFactor = controllerManger->defalutZoom;
    }

    g_xbotgoStrategyCallbacks->onTrackingZoom(scaleFactor);

    // printfXbotGo("TeamStrategyGimbal::zoomTrackPoint  scaleFactor=%.2f controllerManger->defalutZoom= %.2f\n", scaleFactor, controllerManger->defalutZoom);

    return 0;
}

void TeamStrategyGimbal::adjustPitchAngle(TrackResult& trackResult)
{
    if (needPitchDown == true) {
        if (std::abs(currentGyroscope.pitchAngle - controllerManger->pitchInitialAngle) < 0.5) {
            needPitchDown = false;
            printfXbotGo("TeamStrategy::Complete initial Pitch Angle adjustment yaw=%.2f pitch=%.2f\n", currentGyroscope.yawAngle, controllerManger->pitch);
        } else {
            // pitch轴角度上负下正
            if (currentGyroscope.pitchAngle - controllerManger->pitchInitialAngle > 0) {
                trackResult.y = (controllerManger->deviceVideoBufferHeight / 2) - 200;
            } else {
                trackResult.y = (controllerManger->deviceVideoBufferHeight / 2) + 200;
            }
            printfXbotGo("TeamStrategy::adjust initial Pitch Angle trackResult.y=%.2f yaw=%.2f pitch=%.2f\n", trackResult.y, currentGyroscope.yawAngle, controllerManger->pitch);
        }
    }
}

int TeamStrategyGimbal::dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults, std::vector<DelatTrackInfo>& filterMovedResults)
{
    float threshold = param.FastPeopleThreshold; // default 30
    for (const auto& obj : delatTrackInfoResults) {
        if (obj.distance > threshold) {
            filterTrackedResults.emplace_back(obj);
        }
    }
    int ret = 1;
    if (filterTrackedResults.size() < 3) {
        ret = -1;
    }
    printfXbotGo("TeamStrategyGimbal::dynamicFastPeopleFilter  threshold=%.2f filterTrackedResults.size()=%zu\n", threshold, filterTrackedResults.size());
    return ret;
}

/*
int TeamStrategyGimbal::autoTracking(const std::vector<YOLODetectionResult>& detectResult)
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

        printfXbotGo("TeamStrategyGimbal::Timeout back to center trackResult.x=%.2f currentGyroscope.yawAngle=%.2f\n", trackResult.x, currentGyroscope.yawAngle);
    }
    return 0;
}
*/

int TeamStrategyGimbal::caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult)
{
    printfXbotGo("TeamStrategyGimbal::flowTrackBall \n");
    return 0;
}

int TeamStrategyGimbal::followExcludeAreaFilter(const std::vector<YOLODetectionResult>& detectResult, std::vector<YOLODetectionResult>& filteResult)
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

int TeamStrategyGimbal::caculatePeopleDistanceInQueue(const std::vector<std::vector<MOTResult>>& motResults, std::vector<DelatTrackInfo>& delatTrackInfoResults)
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
        // printfXbotGo("TeamStrategyGimbal:caculatePeopleDistanceInQueue (std::abs(distance))= %.2f track_id=%d x=%.2f y=%.2f w=%.2f h=%.2f deltx=%.2f delty=%.2f\n", distance, delatTrackInfo.trackId, delatTrackInfo.x, delatTrackInfo.y, delatTrackInfo.width, delatTrackInfo.height, delatTrackInfo.delaX, delatTrackInfo.delaY);
    }
    delatTrackInfoResults = std::move(tempResult);
    return 0;
}

TrackResult TeamStrategyGimbal::caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults)
{
    TrackResult trackPoint;
    std::vector<DelatTrackInfo> fastThreePeopleResults;
    std::vector<DelatTrackInfo> movedPeopleResults;
    int ret = 0;
    ret = dynamicFastPeopleFilter(delatTrackInfoResults, fastThreePeopleResults, movedPeopleResults); // 1:fast 4:moved>7 2:all move
    if (ret > 0) {
        trackPoint = caculateFastTrackPoint(fastThreePeopleResults);
        trackPoint.stratety = 1;
        lastPoint = trackPoint;
        stayTrackResultTimes = 0;
    } else {
        if (stayTrackResultTimes < 10) // 如果没找到最快的三个人，持续追踪当前最快的三个10帧
        {
            stayTrackResultTimes++;
            trackPoint = lastPoint;
        }
    }

    printfXbotGo("TeamStrategyGimbal::caculateTrackPoint trackResult output point.x=%.2f  point.y=%.2f strategy=%d direction=%d\n", trackPoint.x, trackPoint.y, trackPoint.stratety, trackPoint.direction);
    return trackPoint;
}

void TeamStrategyGimbal::control(const TrackResult& trackResult)
{
    float yawOut = yawPID->update(trackResult.x);
    yawOut = yawSmoother->smooth(yawOut);
    float pitchOut = pitchPID->update(trackResult.y);
    // printfXbotGo("TeamStrategyGimbal::control yawOut=%.2f pitchOut=%.2f yawMaxOut=%.2f pitchMaxOut=%.2f\n", yawOut, pitchOut, yawPID->getMaxOut(), pitchPID->getMaxOut());
    g_xbotgoStrategyCallbacks->onControlSpeed(yawOut, pitchOut);
}

int TeamStrategyGimbal::setYawPIDMaxSpeed(int maxSpeed)
{
    yawMaxSpeed = maxSpeed;
    return 0;
}

void TeamStrategyGimbal::loggingTime(const std::vector<YOLODetectionResult>& detectResult, double time)
{
    printfXbotGo("\n");
    frameSequence++;

    std::tm time_info;

    // 使用 strftime 格式化日期时间
    char time_str[20];

    // 获取当前时间
    if (time == 0.0f) {
        auto now = std::chrono::system_clock::now();
        auto now_since_epoch = now.time_since_epoch();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now_since_epoch) % 1000000;
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        time_info = *std::localtime(&now_c);
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &time_info);
        // 包含日期、时间和微秒
        printfXbotGo("2:TeamStrategy:time=%s.%02lld\n", time_str, micros.count());
    } else {
        std::time_t time_c = (int)time;
        time_info = *std::localtime(&time_c);
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &time_info);
        // 包含日期、时间和微秒
        printfXbotGo("3:TeamStrategy:time=%s %f\n", time_str, time - (int)time);
    }

    // 包含 frameSequence 和 detectResult size
    printfXbotGo("TeamStrategy:--new frameSeq=%ld-- detectResult size=%zu\n", frameSequence, detectResult.size());
}

}
