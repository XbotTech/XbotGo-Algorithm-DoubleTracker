
#ifndef __XBOTGO_SDK_CORE_CONTROLLERMANGER_H__
#define __XBOTGO_SDK_CORE_CONTROLLERMANGER_H__

#include "Global.h"
#include "XbotgoStrategy.h"

#include <iostream>
#include <memory>
#include <thread>
#include <vector>

namespace XbotgoSDK
{

class ControllerManger
{
public:
    ControllerManger()
        : yaw(0.0f), pitch(0.0f), videoBufferWidth(1920), videoBufferHight(1080), deviceVideoBufferWidth(2560), deviceVideoBufferHeight(1440), isTracking(false), needPitchDown(false), pitchInitialAngle(0.0), yawLimitAngle(120.0), defalutZoom(1.0), minZoom(1.0), maxZoom(1.2) {}
    virtual ~ControllerManger() {printfXbotGo("ControllerManger::~\n");}
    void setTrackingActivate();
    void setTrackingDeactivate();
    void setGyroscope(float yaw, float pitch);
    void setVideoBufferSize(int width, int height);
    void setDeviceVideoBufferSize(int width, int height);
    void setYawLimitAngle(int angle);
    void reachPitchLimit();
    void setPitchInitialAngle(float angle);
    void setZoomParam(float defaultZoom, float maxZoom);
public:
    float yaw;
    float pitch;
    int videoBufferWidth;
    int videoBufferHight;
    int deviceVideoBufferWidth; 
    int deviceVideoBufferHeight;
    bool isTracking;         // 是否开启跟踪
    bool needPitchDown;      // 初始状态是否需要俯仰
    float pitchInitialAngle; // 初始俯仰角度
    float yawLimitAngle;     // 水平方向角度限制
    float defalutZoom;
    float minZoom;
    float maxZoom;
};

}

#endif
