#include <iostream>
#include <memory>
#include <vector>

#include "CoreSdk.h"
#include "XbotgoControlCallback.h"
#include "XbotgoSingleStrategyCallback.h"
#include "XbotgoStrategyCallback.h"
#include "XbotgoStrategyCommon.h"
#include "XbotgoTennisStrategyCallback.h"

namespace XbotgoSDK
{

XbotgoStrategyCallback* g_xbotgoStrategyCallbacks = nullptr;
XbotgoSingleStrategyCallback* g_xbotgoSingleStrategyCallbacks = nullptr;
XbotgoTennisStrategyCallback* g_xbotgoTennisStrategyCallback = nullptr;
XbotgoControlCallback* g_xbotgoControlCallback = nullptr;

std::shared_ptr<CoreSdk> xbotgosdkPtr = nullptr;

XBOTSDK_API int Xbotgo_Strategy_init(InitStrategyType type, std::string url, int enableLog)
{
    printf("XbotgoSDK V2.4.2\n");
    if (xbotgosdkPtr != nullptr) {
        printf("Strategy init aready exists!\n");
        return -1;
    }

    xbotgosdkPtr = std::make_shared<CoreSdk>();
    if (xbotgosdkPtr == nullptr) {
        return -1;
    }
    xbotgosdkPtr->setLogfilePath(url, enableLog);

    auto strategyManger = xbotgosdkPtr->getStrategyManger();
    strategyManger->setContext(type);
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }
    auto controller = xbotgosdkPtr->getControllerManger();
    context->setGyroscopeController(controller);

    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_uninit()
{
    printf("XbotgoSDK V2.4.2 uninit\n");
    if (xbotgosdkPtr != nullptr) {
        if (g_xbotgoStrategyCallbacks != nullptr) {
            delete g_xbotgoStrategyCallbacks;
            g_xbotgoStrategyCallbacks = nullptr;
        }
        if (g_xbotgoSingleStrategyCallbacks != nullptr) {
            delete g_xbotgoSingleStrategyCallbacks;
            g_xbotgoSingleStrategyCallbacks = nullptr;
        }
        if (g_xbotgoTennisStrategyCallback != nullptr) {
            delete g_xbotgoTennisStrategyCallback;
            g_xbotgoTennisStrategyCallback = nullptr;
        }
        if (g_xbotgoControlCallback != nullptr) {
            delete g_xbotgoControlCallback;
            g_xbotgoControlCallback = nullptr;
        }
        xbotgosdkPtr.reset();
    }

    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_activate()
{
    if (xbotgosdkPtr == nullptr) {
        printf("Strategy uninitialized!\n");
        return -1;
    }
    auto controller = xbotgosdkPtr->getControllerManger();
    if (controller == nullptr) {
        return -1;
    }
    controller->setTrackingActivate();
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();
    if (context == nullptr) {
        return -1;
    }
    context->init();
    printf("XbotgoSDK activate\n");

    return 0;
}

XBOTSDK_API int Xbotgo_Strategy_deactivate()
{
    if (xbotgosdkPtr == nullptr) {
        printf("Strategy uninitialized!\n");
        return -1;
    }
    auto controller = xbotgosdkPtr->getControllerManger();
    if (controller == nullptr) {
        return -1;
    }
    controller->setTrackingDeactivate();
    printf("XbotgoSDK deactivate\n");

    return 0;
}

XBOTSDK_API int Xbotgo_Set_Printf_Enabled(bool enablePrint)
{
    setPrintfEnabled(enablePrint);
    return 0;
}

}
