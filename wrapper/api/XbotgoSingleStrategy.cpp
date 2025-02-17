#include <iostream>
#include <memory>
#include <vector>

#include "CoreSdk.h"
#include "Strategy.h"
#include "XbotgoSingleStrategy.h"

namespace XbotgoSDK
{

extern std::shared_ptr<CoreSdk> xbotgosdkPtr;

/**
 * @brief 注册单人跟踪策略回调
 *
 * 用来注册单人跟踪策略相关回调，使用者需要自己实现回调接口并将其指针传入
 *
 * @see XbotgoSingleStrategyCallback
 *
 * @param [in] callback - 回调指针
 *
 * @return 返回接口调用结果
 *
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Single_Strategy_registerCallbacks(XbotgoSingleStrategyCallback* callback)
{
    if (g_xbotgoSingleStrategyCallbacks == nullptr) {
        g_xbotgoSingleStrategyCallbacks = callback;
    }

    return 0;
}

/**
 * @brief 设置Single模式的初始化跟踪信息
 *
 * @param [in] detectResult yolo检测结果
 * @param [in] target 跟踪目标信息
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Single_Strategy_target_init(std::vector<std::shared_ptr<DetectionResult>> detectResults, std::shared_ptr<DetectionResult> target)
{
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();

    if (context == nullptr) {
        return -1;
    }

    SingleStrategy* singleContext = dynamic_cast<SingleStrategy*>(context);
    if (singleContext == nullptr) {
        return -1;
    }

    singleContext->targetInit(detectResults, target);

    // TODO: return -1 or 0?
    return 0;
}

/**
 * @brief 设置Single模式的初始化跟踪信息
 *
 * @param [in] detectResult yolo检测结果
 * @param [in] targetIndex 跟踪目标下标
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Single_Strategy_track(const std::vector<std::shared_ptr<DetectionResult>>& detectResults)
{
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();

    if (context == nullptr) {
        return -1;
    }

    SingleStrategy* singleContext = dynamic_cast<SingleStrategy*>(context);
    if (singleContext == nullptr) {
        return -1;
    }

    int result = singleContext->singleTrack(detectResults);
    return result;
}

XBOTSDK_API int Xbotgo_Single_Strategy_init(std::vector<std::shared_ptr<DetectionResult>>& bodyDetectResults, std::shared_ptr<DetectionResult>& target)
{
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();

    if (context == nullptr) {
        return -1;
    }

    SingleStrategyPose* singleContext = dynamic_cast<SingleStrategyPose*>(context);
    if (singleContext == nullptr) {
        return -1;
    }

    singleContext->init(bodyDetectResults, target);

    return 0;
}

XBOTSDK_API HeadAndBodyIndex Xbotgo_Single_Strategy_track(std::vector<std::shared_ptr<DetectionResult>>& headDetectResults, std::vector<std::shared_ptr<DetectionResult>>& bodyDetectResults)
{
    auto context = xbotgosdkPtr->getStrategyManger()->getContext();

    if (context == nullptr) {
        return {-1, -1};
    }

    SingleStrategyPose* singleContext = dynamic_cast<SingleStrategyPose*>(context);
    if (singleContext == nullptr) {
        return {-1, -1};
    }

    HeadAndBodyIndex result = singleContext->track(headDetectResults, bodyDetectResults);
    return result;
}

}
