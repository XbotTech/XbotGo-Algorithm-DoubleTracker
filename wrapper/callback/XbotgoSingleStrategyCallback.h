#ifndef __XBOTGO_STRATEGY_SINGLE_CALLBACK_H__
#define __XBOTGO_STRATEGY_SINGLE_CALLBACK_H__

#include <vector>

#include "MOT.h"

namespace XbotgoSDK
{

class XbotgoSingleStrategyCallback
{
public:
    XbotgoSingleStrategyCallback() {};
    virtual ~XbotgoSingleStrategyCallback() {};

public:
    /// FeatureNet

    /// @brief 获取所有人的特征值
    /// @return 所有人的特征值数组
    virtual std::vector<std::vector<double>> onGetAllFeatures() = 0;

    /// @brief 获取目标下标人的全身特征值
    /// @param index 需要获取特征值的目标
    /// @return 对应目标的特征值信息
    virtual std::vector<double> onGetFeature(int index) = 0;

    /// @brief 获取目标下标人的半身特征值
    /// @param index 需要获取特征值的目标
    /// @return 对应目标的特征值信息
    virtual std::vector<double> onGetHalfFeature(int index) = 0;

    /// @brief 获取对应目标的人全身、半身信息
    /// @param index 需要获取特征值的目标
    /// @return 对应的姿态，0：全身，1：半身，2：未检测出，3：检测到多人
    virtual int onGetPose(int index) = 0;

    /// @brief 获取目标区域的特征值
    /// @param target_coord 需要获取特征值的目标
    /// @return 对应目标区域的特征值信息
    virtual std::vector<double> onGetFeature(YOLODetectionResult target_coord) = 0;

    /// @brief 获取目标区域人的姿态
    /// @param target_coord 需要获取姿态的目标
    /// @return 对应目标的姿态信息
    virtual std::vector<std::vector<float>> onPoseDetect(YOLODetectionResult target_coord) = 0;

    /// Control

    /// @brief 获取当前云台角度Yaw
    /// @return 当前云台角度Yaw
    virtual double onGetCurrentYaw() = 0;

    /// @brief 获取当前云台角度Pitch
    /// @return 当前云台角度Pitch
    virtual double onGetCurrentPitch() = 0;

    /// @brief 设置当前云台角度
    /// @param yaw  横向角度
    /// @param pitch 纵向角度
    virtual void onSetGimbal(double yaw, double pitch) = 0;

    /// Zoom

    /// @brief 获取当前的Zoom系数
    /// @return 当前Zoom的系数
    virtual double onGetZoomFactor() = 0;

    /// @brief 设置当前的Zoom系数
    /// @param factor Zoom系数
    /// @param rate Zoom频率
    virtual void onSetZoomFactor(double factor, double time, double rate) = 0;

    // virtual void onDebugTrackInfo(int sotFailureCount, int state)=0;

    /// @brief 打印FeatureNet相关的Debug信息
    /// @param state 当前FeatureNet状态
    /// @param pose 识别到的人的POSE
    /// @param listFull 全身队列长度
    /// @param listHalf 半身队列长度
    /// @param distanceFull 全身距离
    /// @param distanceHalf 半身距离
    /// @param distancePre 与上一帧的距离
    virtual void onDebugFeatureNetInfo(int state, int pose, int listFull, int listHalf, double distanceFull, double distanceHalf, double distancePre) = 0;

    virtual void onDebugFollowMeInfo(int byteState, int reidState, int poseState, int listFull, int listHalf, double distanceFull, double distanceHalf, double distancePre) = 0;

    /// @brief 球衣号码找回的状态
    /// @param state
    /// 1: 通过球衣号码找回
    /// 2: 通过纯Byte找回(没有交叉)
    /// 3: ReID局部验证找回
    /// 4: ReID全局验证找回
    /// 5: ReID全局验证找回且号码验证失败
    /// 6: 找回失败
    virtual void onDebugJerseyNumberTrackState(int state) = 0;
};

}

#endif
