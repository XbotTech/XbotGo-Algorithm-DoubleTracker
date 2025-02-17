#ifndef __XBOTGO_STRATEGY_CONTROL_CALLBACK_H__
#define __XBOTGO_STRATEGY_CONTROL_CALLBACK_H__

#include "MOT.h"

namespace XbotgoSDK
{

class XbotgoControlCallback
{

public:
    XbotgoControlCallback() {};
    virtual ~XbotgoControlCallback() {};

public:
    virtual void onSetTarget(double x, double y) = 0;
};
}

#endif
