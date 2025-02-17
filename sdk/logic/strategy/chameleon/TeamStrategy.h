
#ifndef __XBOTGO_SDK_CORE_LOGIC_TEAMSTRATEGY_H__
#define __XBOTGO_SDK_CORE_LOGIC_TEAMSTRATEGY_H__

#include <map>
#include <memory>
#include <vector>

#include "Global.h"
#include "IStrategy.h"
#include "MOT.h"
#include "logic/module/BallTracker.h"
#include "logic/module/control/PID.h"
#include "logic/module/control/Smoother.h"
#include "logic/util/CoordinateTransform.h"
#include "logic/util/chamCalibrateXY.h"

#define ZOOM_LIMITE_UPPER_ANGEL 20
#define ZOOM_LIMITE_LOWER_ANGEL 10
#define ZOOM_LIMITE_OFFSET 450
#define ZOOM_LIMITE_UPPER_PERSON_NUM 8
#define ZOOM_LIMITE_LOWER_PERSON_NUM 3

namespace XbotgoSDK
{

class TeamStrategy : public IStrategy
{
public:
    TeamStrategy()
        : mot(nullptr), ballTracker(nullptr), chamCalibrateXY(nullptr), pitchPID(nullptr), yawPID(nullptr), yawSmoother(nullptr)
    {
    }
    virtual ~TeamStrategy()
    {
        if (mot != nullptr) {
            delete mot;
            mot = nullptr;
        }
        if (ballTracker != nullptr) {
            delete ballTracker;
            ballTracker = nullptr;
        }
        if (chamCalibrateXY != nullptr) {
            delete chamCalibrateXY;
            chamCalibrateXY = nullptr;
        }
        if (pitchPID != nullptr) {
            delete pitchPID;
            pitchPID = nullptr;
        }
        if (yawPID != nullptr) {
            delete yawPID;
            yawPID = nullptr;
        }
        if (yawSmoother != nullptr) {
            delete yawSmoother;
            yawSmoother = nullptr;
        }
        printfXbotGo("Team:~\n");
    }

    int track(const std::vector<YOLODetectionResult>& detectResult, double time) override;

    int init() override;

    int setYawPIDMaxSpeed(int maxSpeed);

protected:
    // 跟移动最快的三个人
    TrackResult caculateFastTrackPoint(const std::vector<DelatTrackInfo>& trackResults);

    // zoom控制算法
    virtual int zoomTrackPoint(const std::vector<MOTResult>& motResults, const TrackResult& trackResult);

    // 动态过滤算法
    virtual int dynamicFastPeopleFilter(const std::vector<DelatTrackInfo>& delatTrackInfoResults, std::vector<DelatTrackInfo>& filterTrackedResults);

    // track ball算法
    virtual int caculateBallTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, TrackResult& trackResult);

    // 自动回中策略
    // virtual int autoTracking(const std::vector<YOLODetectionResult>& detectResult);

    // 计算策略的跟踪点
    virtual TrackResult caculateTrackPoint(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult, const std::vector<DelatTrackInfo> delatTrackInfoResults);

    virtual void control(const TrackResult& trackResult);

private:
    // 按人员面积过滤
    int followExcludeAreaFilter(const std::vector<YOLODetectionResult>& detectResult, std::vector<YOLODetectionResult>& filteResult);

    // 计算移动最快的三个人，队尾到队首所有的帧
    int caculatePeopleDistanceInQueue(const std::vector<std::vector<MOTResult>>& motResults, std::vector<DelatTrackInfo>& delatTrackInfoResults);

    // 策略处理
    int strategyProcess(const std::vector<YOLODetectionResult>& yoloDetectResult, const std::vector<MOTResult>& motResult);

    // 钳位控制算法
    int clampingLimiteAnger(TrackResult& trackResult);

    // 角度死区补偿算法
    Gyroscope angleDeadZoneCompensation(Gyroscope& gyroscope);

    // 记录日志时间
    void loggingTime(const std::vector<YOLODetectionResult>& detectResult, double time_stamp);

    // 初始化俯仰控制
    void adjustPitchAngle(TrackResult& trackResult);

protected:
    // 追踪人策略相关
    MOT* mot;
    std::vector<std::vector<MOTResult>> motResults;

    // 追踪球策略相关
    BallTracker* ballTracker;

    // 追踪点相关
    TrackResult lastPoint;
    int stayTrackResultTimes;
    // int autoTrackingFrameCount;

    // 陀螺仪相关
    bool turnRight = false;
    float turnAroundYawAngle = 0.0;
    Gyroscope lastGyroscope = {0.0, 0.0};
    Gyroscope currentGyroscope;
    Gyroscope bestGyroscope;

    // 坐标转换相关
    ChamCalibrateXY* chamCalibrateXY;

    // zoom相关
    bool isZoom = false;
    float scaleFactor = 1.0;
    int defaultZoomCounter = 0;

    // 控制相关
    PID* pitchPID;
    PID* yawPID;
    Smoother* yawSmoother;
    int yawMaxSpeed = 800;
    int pitchMaxSpeed = 800;

    // 策略参数
    StrategyParam param;

    // 是否需要设置云台初始角度
    bool needPitchDown = false;
    int completePitchDownCount = 0;
};

}

#endif
