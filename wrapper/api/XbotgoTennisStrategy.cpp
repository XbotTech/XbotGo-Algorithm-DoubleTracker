#include "XbotgoTennisStrategy.h"
#include "CoreSdk.h"

namespace XbotgoSDK
{
extern std::shared_ptr<CoreSdk> xbotgosdkPtr;

XBOTSDK_API int Xbotgo_Tennis_Strategy_registerCallbacks(XbotgoTennisStrategyCallback* callback)
{
    if (g_xbotgoTennisStrategyCallback == nullptr) {
        g_xbotgoTennisStrategyCallback = callback;
    }

    return 0;
}

XBOTSDK_API int Xbotgo_Tennis_Strategy_track(const std::vector<YOLODetectionResult>& yoloResult, const std::vector<BallDetectionResult>& hoopResult, int type)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }

    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }
    TennisStrategy* tennis = dynamic_cast<TennisStrategy*>(context);
    if (tennis == nullptr) {
        return -1;
    }
    int result = tennis->track(yoloResult, hoopResult, type);

    return result;
}

}