
#ifndef __XBOTGO_SDK_CORE_LOGIC_STRATEGY_H__
#define __XBOTGO_SDK_CORE_LOGIC_STRATEGY_H__

#include <iostream>
#include <memory>

#include "ControllerManger.h"
#include "DetectionResult.h"

#include "Global.h"

namespace XbotgoSDK
{

struct DelatTrackInfo {
    int trackId;
    float x;
    float y;
    float width;
    float height;
    float score;
    float delaX;
    float delaY;
    float distance;
    DelatTrackInfo()
    {
    }
    DelatTrackInfo(int tId, float xPos, float yPos, float w, float h, float s, float dx, float dy, float d)
        : trackId(tId), x(xPos), y(yPos), width(w), height(h), score(s), delaX(dx), delaY(dy), distance(d)
    {
    }
};

/**
 * @brief  策略参数
 */
struct StrategyParam {
    int motFPS;              // MOT帧率 默认10
    int frameInterval;       // 缓存帧间间隔 默认10
    int caculateInterval;    // 计算跟踪点频率 默认10
    int FastPeopleThreshold; // 最快三个人阈值 默认30~60
    float yoloAreaThreshold; // 面积过滤阈值 默认2000

    struct StrategyParam& operator=(const struct StrategyParam& other)
    {
        if (this != &other) {
            motFPS = other.motFPS;
            frameInterval = other.frameInterval;
            caculateInterval = other.caculateInterval;
            FastPeopleThreshold = other.FastPeopleThreshold;
            yoloAreaThreshold = other.yoloAreaThreshold;
        }
        return *this;
    }
};

class IStrategy
{
public:
    IStrategy()
        : frameSequence(0)
    {
    }

    virtual ~IStrategy()
    {
        printfXbotGo("IStrategy:~\n");
    }

    virtual int track(const std::vector<YOLODetectionResult>& detectResult, double time) = 0;

    virtual int init()
    {
        printfXbotGo("IStrategy:init()\n");
        return 0;
    }

public:
    void setVideoBufferSize(int width, int height);
    void setZoomInfo(double minZoom, double maxZoom);
    void setGyroscopeController(std::shared_ptr<ControllerManger>& controllerManger);
    void setFps(int fps);

protected:
    double minZoom;
    double maxZoom;

    int deviceVideoBufferWidth;
    int deviceVideoBufferHeight;

    int fps;

    long frameSequence;

    std::shared_ptr<ControllerManger> controllerManger;
};

}

#endif
