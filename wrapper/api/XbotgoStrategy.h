
#ifndef __XBOTGO_STRATEGY_H__
#define __XBOTGO_STRATEGY_H__

#include <iostream>

#include "DetectionResult.h"
#include "XbotgoStrategyCommon.h"

// #define XBOTSDK_API extern "C"
#define XBOTSDK_API

namespace XbotgoSDK
{
class XbotgoStrategyCallback;

extern XbotgoStrategyCallback* g_xbotgoStrategyCallbacks;

/**
 * @brief  跟踪点
 *
 */
struct TrackPoint {
    TrackPoint(float xValue, float yValue)
        : x(xValue), y(yValue)
    {
    }
    float x;
    float y;

    TrackPoint& operator=(const TrackPoint& other)
    {
        if (this != &other) {
            x = other.x;
            y = other.y;
        }
        return *this;
    }
};

/**
 * @brief  跟踪点, 保留扩展
 *
 */
struct TrackResult {
    TrackResult()
        : x(0), y(0), direction(0), stratety(0), speed(0), yawAngle(0), pitchAngle(0), currentYawAngle(0), currentPitchAngle(0)
    {
    }
    TrackResult(float xValue, float yValue)
        : x(xValue), y(yValue)
    {
    }
    float x;
    float y;
    int direction; // left:-1 middel:0 right:1
    int stratety;  // fast three person:1 ｜ all moved person:2 ｜ ball:3  ｜ moved>7 person:4  | no track：0
    float speed;
    float yawAngle;          // best yaw
    float pitchAngle;        // best pitch
    float currentYawAngle;   // current yaw
    float currentPitchAngle; // current pitch
    TrackResult& operator=(const TrackResult& other)
    {
        if (this != &other) {
            x = other.x;
            y = other.y;
            direction = other.direction;
            stratety = other.stratety;
            speed = other.speed;
            yawAngle = other.yawAngle;
            pitchAngle = other.pitchAngle;
            currentYawAngle = other.currentYawAngle;
            currentPitchAngle = other.currentPitchAngle;
        }
        return *this;
    }
};

/**
 * @brief 注册camonlone设备回调
 *
 * 用来注册camonlone设备相关回调，使用者需要自己实现回调接口并将其指针传入
 *
 * @see XbotgoStrategyCallback
 *
 * @param [in] callback - 回调指针
 *
 * @return 返回接口调用结果
 *
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_registerCallbacks(XbotgoStrategyCallback* callback);

/**
 * @brief 跟踪算法
 *
 * @param [in] detectResult yolo检测结果

 * @return 返回接口调用结果
  * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_track(const std::vector<YOLODetectionResult>& detectResult);

/**
 * @brief 跟踪算法, 仅供内部测试用
 *
 * @param [in] detectResult yolo检测结果

 * @return 返回接口调用结果
  * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_track_for_test(const std::vector<YOLODetectionResult>& detectResult, double time);

/**
 * @brief 设置zoom最大最小系数, 分辨率有变化时调用
 *
 * @param [in] minZoom, maxZoom

 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_setZoomInfo(double minZoom, double maxZoom);

/**
 * @brief 设置video buffer size, 分辨率有变化时调用
 *
 * @param [in] wdith ，height

 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_setVideoBufferSize(int width, int height);

/**
 * @brief 设置设备摄像头的buffer size，分辨率有变化时调用
 *
 * @param [in] wdith ，height

 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_device_setVideoBufferSize(int width, int height);

/**
 * @brief 设置陀螺的数据
 *
 * @param [float] yaw ，pitch

 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_setGyroscope(float yaw, float pitch);

/**
 * @brief 设置zoom的参数
 *
 * @param defaultZoom:zoomout maxZoom:zoomin

 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_setZoomParam(float defaultZoom, float maxZoom);

/**
 * @brief 设置极限角参数
 *
 * @param angle

 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_set_yawLimitAngle(int angle);

/**
 * @brief 设置pitch初始角度
 *
 * @param pitchAngle 

 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_set_pitch_initialAngle(float angle);

/**
 * @brief pitch轴限位开关
 *

 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_reachPitchLimit();

/**
 * @brief 设置yawPID最大速度参数
 *
 * @param maxSpeed

 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_set_yawMaxSpeed(int maxSpeed);

}

#endif