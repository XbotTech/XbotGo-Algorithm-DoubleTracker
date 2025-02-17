
#ifndef __XBOTGO_STRATEGY_COMMON_H__
#define __XBOTGO_STRATEGY_COMMON_H__

#include <iostream>

#include "DetectionResult.h"
#include "Global.h"

// #define XBOTSDK_API extern "C"
#define XBOTSDK_API

namespace XbotgoSDK
{

/**
 * @brief 球检测模型 篮球、足球
 * lable :球框 带球的人
 */
struct BallDetectionResult {
    int lable; // 0:人 1:球 2:球筐
    float prob;
    float x;
    float y;
    float width;
    float height;

    BallDetectionResult()
        : lable(0), prob(0.0), x(0.0), y(0.0), width(0.0), height(0.0)
    {
    }

    BallDetectionResult(int lable, float prob, float x, float y, float w, float h)
        : lable(lable), prob(prob), x(x), y(y), width(w), height(h)
    {
    }

    BallDetectionResult& operator=(const BallDetectionResult& other)
    {
        if (this != &other) {
            lable = other.lable;
            prob = other.prob;
            x = other.x;
            y = other.y;
            width = other.width;
            height = other.height;
        }
        return *this;
    }

    bool is_empty() const
    {
        return lable == 0 && prob == 0.0 && x == 0.0 && y == 0.0 && width == 0.0 && height == 0.0;
    }

    void clear()
    {
        lable = 0;
        prob = 0;
        x = 0;
        y = 0;
        width = 0;
        height = 0;
    }
};

/**
 * 初始化策略参数类型
 */
enum class InitStrategyType {
    INIT_STRATEGY_TYPE_TEAM,                               // 团队模式
    INIT_STRATEGY_TYPE_SOCCER_5V5_OVER14_CHAMELEON,        // 足球    5v5    十四岁以上  chameleon
    INIT_STRATEGY_TYPE_SOCCER_5V5_UNDER14_CHAMELEON,       // 足球    5v5    十四岁以下  chameleon
    INIT_STRATEGY_TYPE_SOCCER_7V7_OVER14_CHAMELEON,        // 足球    7v7    十四岁以上  chameleon
    INIT_STRATEGY_TYPE_SOCCER_7V7_UNDER14_CHAMELEON,       // 足球    7v7    十四岁以下  chameleon
    INIT_STRATEGY_TYPE_SOCCER_11V11_OVER14_CHAMELEON,      // 足球    11v11  十四岁以上  chameleon
    INIT_STRATEGY_TYPE_SOCCER_11V11_UNDER14_CHAMELEON,     // 足球    11v11  十四岁以下  chameleon
    INIT_STRATEGY_TYPE_BASKETBALL_WHOLE_OVER14_CHAMELEON,  // 篮球    全场   十四岁以上  chameleon
    INIT_STRATEGY_TYPE_BASKETBALL_WHOLE_UNDER14_CHAMELEON, // 篮球    全场   十四岁以下  chameleon
    INIT_STRATEGY_TYPE_BASKETBALL_HALF_OVER14_CHAMELEON,   // 篮球    半场   十四岁以上  chameleon
    INIT_STRATEGY_TYPE_BASKETBALL_HALF_UNDER14_CHAMELEON,  // 篮球    半场   十四岁以下  chameleon
    INIT_STRATEGY_TYPE_WHEELCHAIR_SOCCER,                  // 轮椅足球
    INIT_STRATEGY_TYPE_RUGBY_WHOLE_OVER14,                 // 橄榄球  全场   十四岁以上
    INIT_STRATEGY_TYPE_RUGBY_WHOLE_UNDER14,                // 橄榄球  全场   十四岁以下
    INIT_STRATEGY_TYPE_HOCKEY_WHOLE_OVER14,                // 曲棍球  全场   十四岁以上
    INIT_STRATEGY_TYPE_HOCKEY_WHOLE_UNDER14,               // 曲棍球  全场   十四岁以下
    INIT_STRATEGY_TYPE_ICE_HOCKEY_WHOLE_OVER14,            // 冰球    全场   十四岁以上
    INIT_STRATEGY_TYPE_ICE_HOCKEY_WHOLE_UNDER14,           // 冰球    全场   十四岁以下
    INIT_STRATEGY_TYPE_TENNIS_SINGLE,                      // 网球    单打
    INIT_STRATEGY_TYPE_TENNIS_DOUBLE,                      // 网球    双打
    INIT_STRATEGY_TYPE_HANDBALL_WHOLE_OVER14,              // 手球    全场   十四岁以上
    INIT_STRATEGY_TYPE_HANDBALL_WHOLE_UNDER14,             // 手球    全场   十四岁以下
    INIT_STRATEGY_TYPE_HANDBALL_HALF_OVER14,               // 手球    半场   十四岁以上
    INIT_STRATEGY_TYPE_HANDBALL_HALF_UNDER14,              // 手球    半场   十四岁以下
    INIT_STRATEGY_TYPE_BROOM_BALL_WHOLE_OVER14,            // 扫帚球  全场   十四岁以上
    INIT_STRATEGY_TYPE_BROOM_BALL_WHOLE_UNDER14,           // 扫帚球  全场   十四岁以下
    INIT_STRATEGY_TYPE_PICKLEBALL_SINGLE,                  // 匹克球  单打
    INIT_STRATEGY_TYPE_PICKLEBALL_DOUBLE,                  // 匹克球  双打
    INIT_STRATEGY_TYPE_FOLLOWMEPOSE,                       // 单人新Pose模式
    INIT_STRATEGY_TYPE_JERSEYNUMBER,                       // 球衣号码
    INIT_STRATEGY_TYPE_SOCCER_5V5_OVER14_GIMBAL,           // 足球    5v5    十四岁以上  gimbal
    INIT_STRATEGY_TYPE_SOCCER_5V5_UNDER14_GIMBAL,          // 足球    5v5    十四岁以下  gimbal
    INIT_STRATEGY_TYPE_SOCCER_7V7_OVER14_GIMBAL,           // 足球    7v7    十四岁以上  gimbal
    INIT_STRATEGY_TYPE_SOCCER_7V7_UNDER14_GIMBAL,          // 足球    7v7    十四岁以下  gimbal
    INIT_STRATEGY_TYPE_SOCCER_11V11_OVER14_GIMBAL,         // 足球    11v11  十四岁以上  gimbal
    INIT_STRATEGY_TYPE_SOCCER_11V11_UNDER14_GIMBAL,        // 足球    11v11  十四岁以下  gimbal
    INIT_STRATEGY_TYPE_BASKETBALL_WHOLE_OVER14_GIMBAL,     // 篮球    全场   十四岁以上  gimbal
    INIT_STRATEGY_TYPE_BASKETBALL_WHOLE_UNDER14_GIMBAL,    // 篮球    全场   十四岁以下  gimbal
    INIT_STRATEGY_TYPE_BASKETBALL_HALF_OVER14_GIMBAL,      // 篮球    半场   十四岁以上  gimbal
    INIT_STRATEGY_TYPE_BASKETBALL_HALF_UNDER14_GIMBAL,     // 篮球    半场   十四岁以下  gimbal
    INIT_STRATEGY_TYPE_ICE_HOCKEY_WHOLE_OVER14_GIMBAL,     // 冰球    全场   十四岁以上 gimbal
    INIT_STRATEGY_TYPE_ICE_HOCKEY_WHOLE_UNDER14_GIMBAL,    // 冰球    全场   十四岁以下 gimbal
};

/**
 * @brief 初始化SDK
 *
 * 用来初始化SDK，在初始化过程中会根据参数配置来初始化不同模块
 *
 * @param [in] type 策略类型 参数type的含义@see InitStrategyType
 * @param [in] enableLog 是否开启log
 * @param [in] url  日志路径
 * @return 返回接口调用结果
 *
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_init(InitStrategyType type, std::string url = "./", int enableLog = 1);

/**
 * @brief 卸载SDK
 *
 * 用来卸载SDK，在卸载过程中释放资源
 *
 * @return 返回接口调用结果
 *
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_uninit();

/**
 * @brief 开始运行SDK策略
 *
 * 用于激活SDK策略运行
 *
 * @return 返回接口调用结果
 *
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_activate();

/**
 * @brief 结束运行SDK策略
 *
 * 用于结束SDK策略运行
 *
 * @return 返回接口调用结果
 *
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Strategy_deactivate();

/**
 * @brief 设置是否开启打印输出
 *
 *
 * @param [in] enablePrint 是否开启打印输出
 * @return 返回接口调用结果
 *
 * @retval 0 - Success\n
 * @retval -1 - Failed
 */
XBOTSDK_API int Xbotgo_Set_Printf_Enabled(bool enablePrint);

}

#endif