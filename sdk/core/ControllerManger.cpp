#include "ControllerManger.h"
#include "XbotgoStrategyCallback.h"

namespace XbotgoSDK
{

extern XbotgoStrategyCallback* g_xbotgoStrategyCallbacks;

void ControllerManger::setTrackingActivate()
{
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto now_since_epoch = now.time_since_epoch();
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now_since_epoch) % 1000000;
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm time_info = *std::localtime(&now_c);

    // 使用 strftime 格式化日期时间
    char time_str[20];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &time_info);

    // 包含日期、时间和微秒
    printfXbotGo("time=%s.%02lld\n", time_str, micros.count());
    this->isTracking = true;
}

void ControllerManger::setTrackingDeactivate()
{
    // 获取当前时间
    auto now = std::chrono::system_clock::now();
    auto now_since_epoch = now.time_since_epoch();
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(now_since_epoch) % 1000000;
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm time_info = *std::localtime(&now_c);

    // 使用 strftime 格式化日期时间
    char time_str[20];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &time_info);

    // 包含日期、时间和微秒
    printfXbotGo("\ntime=%s.%02lld\n", time_str, micros.count());
    this->isTracking = false;
    g_xbotgoStrategyCallbacks->onTrackingZoom(defalutZoom);
}

void ControllerManger::setGyroscope(float yaw, float pitch)
{
    this->yaw = yaw;
    this->pitch = pitch;
}

void ControllerManger::setVideoBufferSize(int width, int height)
{
    this->videoBufferHight = height;
    this->videoBufferWidth = width;
}

void ControllerManger::setDeviceVideoBufferSize(int width, int height)
{
    this->deviceVideoBufferWidth = width;
    this->deviceVideoBufferHeight = height;
}

void ControllerManger::setYawLimitAngle(int angle)
{
    this->yawLimitAngle = angle;
}

void ControllerManger::setPitchInitialAngle(float angle)
{
    this->needPitchDown = true;
    this->pitchInitialAngle = angle;
}

void ControllerManger::reachPitchLimit()
{
    this->pitchInitialAngle = this->pitch;
}

void ControllerManger::setZoomParam(float defaultZoom, float maxZoom)
{
    this->defalutZoom = defalutZoom;
    this->maxZoom = maxZoom;
}

}
