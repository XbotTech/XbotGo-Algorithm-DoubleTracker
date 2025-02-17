#include "XbotgoPickleballStrategy.h"
#include "CoreSdk.h"

namespace XbotgoSDK
{
extern std::shared_ptr<CoreSdk> xbotgosdkPtr;

XBOTSDK_API int Xbotgo_Pickleball_Strategy_registerControlCallback(XbotgoControlCallback* callback)
{
    if (g_xbotgoControlCallback == nullptr) {
        g_xbotgoControlCallback = callback;
    }

    return 0;
}

/// @brief 设置angle信息
/// @param maxAngle X轴最大转动度数(默认20先)，会根据输入的值判断正负
/// @param pixelPerAngle 每度的像素偏移量(参考reid)
XBOTSDK_API int Xbotgo_Pickleball_Strategy_SetAngleInfo(double maxAngle, double pixelPerAngle, int maxTurnDownTime)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }

    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }
    PickleballStrategy* pickleballStrategy = dynamic_cast<PickleballStrategy*>(context);
    if (pickleballStrategy == nullptr) {
        return -1;
    }
    pickleballStrategy->setAngleInfo(maxAngle, pixelPerAngle, maxTurnDownTime);

    return 0;
}

XBOTSDK_API int Xbotgo_Pickleball_Strategy_track(const std::vector<YOLODetectionResult>& yoloResult, double currentXAngle)
{
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }

    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }
    PickleballStrategy* pickleballStrategy = dynamic_cast<PickleballStrategy*>(context);
    if (pickleballStrategy == nullptr) {
        return -1;
    }
    int result = pickleballStrategy->track(yoloResult, currentXAngle, 0);

    return result;
}

}
