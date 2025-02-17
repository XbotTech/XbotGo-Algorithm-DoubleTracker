
#ifndef __XBOTGO_STRATEGY_CALLBACK_H__
#define __XBOTGO_STRATEGY_CALLBACK_H__

#include "MOT.h"

namespace XbotgoSDK
{

class XbotgoStrategyCallback
{
public:
    XbotgoStrategyCallback(){};
    virtual ~XbotgoStrategyCallback(){};

public:
    
    //返回跟踪的点
    virtual void onTrackingPoint(TrackResult point)=0;

    //zoom 回调接口
    virtual void onTrackingZoom(float scaleFactor)=0;

    //控制回调接口
    virtual void onControlSpeed(int yawSpeed, int pitchSpeed)=0;

};


}

#endif