#ifndef __XBOTGO_STRATEGY_TENNIS_CALLBACK_H__
#define __XBOTGO_STRATEGY_TENNIS_CALLBACK_H__

#include "MOT.h"

namespace XbotgoSDK
{
struct TrackResult;

class XbotgoTennisStrategyCallback
{
public:
    XbotgoTennisStrategyCallback() {};
    virtual ~XbotgoTennisStrategyCallback() {};

public:
    // 返回跟踪的点
    virtual void onTrackingPoint(TrackResult point) = 0;
};

}

#endif
