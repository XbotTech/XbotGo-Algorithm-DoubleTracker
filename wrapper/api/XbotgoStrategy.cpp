#include <iostream>
#include <memory>
#include <vector>

#include "CoreSdk.h"
#include "Strategy.h"
#include "XbotgoStrategy.h"

namespace XbotgoSDK
{

extern std::shared_ptr<CoreSdk> xbotgosdkPtr;

XBOTSDK_API int Xbotgo_Strategy_registerCallbacks(XbotgoStrategyCallback* callback)
{
    if (g_xbotgoStrategyCallbacks == nullptr) {
        g_xbotgoStrategyCallbacks = callback;
    }
    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_track(const std::vector<YOLODetectionResult>& detectResult)
{
    double time_stamp = 0;
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }

    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }
    int result = context->track(detectResult, time_stamp);

    return result;
}

XBOTSDK_API int Xbotgo_Strategy_track_for_test(const std::vector<YOLODetectionResult>& detectResult, double time_stamp)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }

    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }
    int result = context->track(detectResult, time_stamp);

    return result;
}

XBOTSDK_API int Xbotgo_Strategy_setZoomInfo(double minZoom, double maxZoom)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }

    // TODO: 先暂时为了单人模式，后续会改成统一的config信息
    context->setZoomInfo(minZoom, maxZoom);

    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_setVideoBufferSize(int width, int height)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }
    auto controller = xbotgosdkPtr->getControllerManger();
    controller->setVideoBufferSize(width, height);
    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_device_setVideoBufferSize(int width, int height)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }

    // TODO: 先暂时为了单人模式，后续会改成统一的config信息
    context->setVideoBufferSize(width, height);

    auto controller = xbotgosdkPtr->getControllerManger();
    controller->setDeviceVideoBufferSize(width, height);
    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_setGyroscope(float yaw, float pitch)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    auto context = xbotgosdkPtr->getControllerManger();
    if (context == nullptr) {
        return -1;
    }
    context->setGyroscope(yaw, pitch);

    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_set_yawLimitAngle(int angle)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    auto controller = xbotgosdkPtr->getControllerManger();
    if (controller == nullptr) {
        return -1;
    }
    controller->setYawLimitAngle(angle);

    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_setZoomParam(float defaultZoom, float maxZoom)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    auto controller = xbotgosdkPtr->getControllerManger();
    if (controller == nullptr) {
        return -1;
    }
    controller->setZoomParam(defaultZoom, maxZoom);
    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_set_pitch_initialAngle(float angle)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    auto controller = xbotgosdkPtr->getControllerManger();
    if (controller == nullptr) {
        return -1;
    }
    controller->setPitchInitialAngle(angle);

    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_reachPitchLimit()
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    auto controller = xbotgosdkPtr->getControllerManger();
    if (controller == nullptr) {
        return -1;
    }
    controller->reachPitchLimit();

    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_set_yawMaxSpeed(int maxSpeed)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }

    TeamStrategy* tcontext = dynamic_cast<TeamStrategy*>(context);
    if (tcontext == nullptr) {
        return -1;
    }
    int result = tcontext->setYawPIDMaxSpeed(maxSpeed);

    return result;
}

}
