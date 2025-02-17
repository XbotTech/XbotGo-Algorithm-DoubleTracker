#ifndef __XBOTGO_STRATEGY_PICKLEBALL_H__
#define __XBOTGO_STRATEGY_PICKLEBALL_H__

#include <iostream>

#include "DetectionResult.h"
#include "XbotgoStrategyCommon.h"

#define XBOTSDK_API

namespace XbotgoSDK
{

class XbotgoControlCallback;

extern XbotgoControlCallback* g_xbotgoControlCallback;

/**
 * @brief 注册camonlone设备回调
 *
 * 用来注册camonlone设备相关回调，使用者需要自己实现回调接口并将其指针传入
 *
 * @see XbotgoControlCallback
 *
 * @param [in] callback - 回调指针
 *
 * @return 返回接口调用结果
 *
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Pickleball_Strategy_registerControlCallback(XbotgoControlCallback* callback);

/// @brief 设置angle信息
/// @param maxAngle X轴最大转动度数(默认20先)，会根据输入的值判断正负
/// @param pixelPerAngle 每度的像素偏移量(参考reid)
XBOTSDK_API int Xbotgo_Pickleball_Strategy_SetAngleInfo(double maxAngle, double pixelPerAngle, int maxTurnDownTime);

/**
 * @brief 跟踪算法
 *
 * @param [in] yoloResult yolo检测结果
 * @param [in] xOffset 球框检测结果
 * @param [in] yOffset 球框检测结果
 * @return 返回接口调用结果
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Pickleball_Strategy_track(const std::vector<YOLODetectionResult>& yoloResult, double currentXAngle);

}

#endif
