#ifndef __XBOTGO_STRATEGY_TENNIS_H__
#define __XBOTGO_STRATEGY_TENNIS_H__

#include <iostream>

#include "DetectionResult.h"
#include "XbotgoStrategyCommon.h"

// #define XBOTSDK_API extern "C"
#define XBOTSDK_API

namespace XbotgoSDK
{
class XbotgoTennisStrategyCallback;

extern XbotgoTennisStrategyCallback* g_xbotgoTennisStrategyCallback;

/**
 * @brief 注册camonlone设备回调
 *
 * 用来注册camonlone设备相关回调，使用者需要自己实现回调接口并将其指针传入
 *
 * @see XbotgoTennisStrategyCallback
 *
 * @param [in] callback - 回调指针
 *
 * @return 返回接口调用结果
 *
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Tennis_Strategy_registerCallbacks(XbotgoTennisStrategyCallback* callback);

/**
 * @brief 跟踪算法
 *
 * @param [in] yoloResult yolo检测结果
 * @param [in] hoopResult 球框检测结果
 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Tennis_Strategy_track(const std::vector<YOLODetectionResult>& yoloResult, const std::vector<BallDetectionResult>& hoopResult, int type);

}

#endif