#ifndef __XBOTGO_SINGLE_STRATEGY_H__
#define __XBOTGO_SINGLE_STRATEGY_H__

#include <iostream>
#include <memory>

#include "DetectionResult.h"
#include "XbotgoStrategyCommon.h"

// #define XBOTSDK_API extern "C"
#define XBOTSDK_API

namespace XbotgoSDK
{

class XbotgoSingleStrategyCallback;

extern XbotgoSingleStrategyCallback* g_xbotgoSingleStrategyCallbacks;

/**
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
XBOTSDK_API int Xbotgo_Single_Strategy_registerCallbacks(XbotgoSingleStrategyCallback* callback);

/**
 * @brief 设置Single模式的初始化跟踪信息
 *
 * @param [in] detectResult yolo检测结果
 * @param [in] target 跟踪目标信息
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Single_Strategy_target_init(std::vector<std::shared_ptr<DetectionResult>> detectResults, std::shared_ptr<DetectionResult> target);

/**
 * @brief 设置Single模式的初始化跟踪信息
 *
 * @param [in] detectResult yolo检测结果
 * @param [in] targetIndex 跟踪目标下标
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Single_Strategy_track(const std::vector<std::shared_ptr<DetectionResult>>& detectResults);

XBOTSDK_API int Xbotgo_Single_Strategy_init(std::vector<std::shared_ptr<DetectionResult>>& bodyDetectResults, std::shared_ptr<DetectionResult>& target);

XBOTSDK_API HeadAndBodyIndex Xbotgo_Single_Strategy_track(std::vector<std::shared_ptr<DetectionResult>>& headDetectResults, std::vector<std::shared_ptr<DetectionResult>>& bodyDetectResults);

}

#endif
