#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "DetectionResult.h"
#include "PosePostProcess.h"

namespace XbotgoSDK
{

#define _byte_unstart_ 0               /**< 标志位：ByteTrack追踪未启动 */
#define _byte_success_ 1               /**< 标志位：ByteTrack追踪成功，且追踪目标没有和其他人有重叠情况 */
#define _byte_success_overlap_front_ 2 /**< 标志位：ByteTrack追踪成功且Reid上下帧验证通过，追踪目标存在和其他人有重叠情况，目标在所有重叠对象前面 */
#define _byte_success_overlap_back_ 3  /**< 标志位：ByteTrack追踪成功且Reid上下帧验证通过，追踪目标存在和其他人有重叠情况，目标在重叠对象后面 */
#define _byte_wrong_overlap_front_ 4   /**< 标志位：ByteTrack追踪错误未通过Reid上下帧验证，追踪目标存在和其他人有重叠情况，错误目标在重叠的前面 */
#define _byte_wrong_overlap_back_ 5    /**< 标志位：ByteTrack追踪错误未通过Reid上下帧验证，追踪目标存在和其他人有重叠情况，错误目标在重叠的后面 */
#define _byte_wrong_overlap_pose_ 6    /**< 标志位：ByteTrack追踪错误未通过Reid上下帧验证，追踪目标存在和其他人有重叠情况，错误目标未检出骨骼点 */
#define _byte_wrong_pose_ 7            /**< 标志位：ByteTrack追踪错误未通过Reid上下帧验证，错误目标未检出骨骼点 */
#define _byte_wrong_ 8                 /**< 标志位：ByteTrack追踪错误未通过Reid上下帧主动验证 */
#define _byte_lose_ 9                  /**< 标志位：ByteTrack追踪失败，丢失目标 */

#define _reid_unstart_ 0              /**< 标志位：全局Reid追踪未启动 */
#define _reid_lose_ 1                 /**< 标志位：全局Reid追踪失败 */
#define _reid_full_ 2                 /**< 标志位：全局Reid追踪成功,从全身特征队列中找回目标 */
#define _reid_half_ 3                 /**< 标志位：全局Reid追踪成功,从半身特征队列中找回目标 */
#define _reid_full_base_ 4            /**< 标志位：全局Reid追踪成功,从全身基特征队列中找回目标 */
#define _reid_half_base_ 5            /**< 标志位：全局Reid追踪成功,从半身基特征队列中找回目标 */
#define _reid_overlap_verification_ 6 /**< 标志位：byte追踪目标周围存在和其他人重叠，启动Reid上下帧验证通过 */
#define _reid_active_verification_ 7  /**< 标志位：byte追踪目标上几帧存在和其他人重叠，启动Reid主动验证通过 */

bool yoloOverLap(const std::vector<YOLODetectionResult>& yolo_detection_results, int index);

bool poseOverLap(const std::vector<Box>& pose_boxes);

bool isTargetInFront(int index, const std::vector<YOLODetectionResult>& body_detection_results);

YOLODetectionResult paddingRect(YOLODetectionResult target_coord, double screen_width, double screen_height);

/**
 * @brief 特征向量网络类，用于目标特征的提取、保存和Reid单目标追踪
 */
class PoseFeatureNet
{
public:
    /**
     * @brief 构造函数
     */
    PoseFeatureNet(const double& screen_width, const double& screen_height);

    /**
     * @brief 初始化目标特征列表和阈值
     *
     * @param targetFeatures 要追踪目标的特征
     * @param reid_threshold reid阈值
     */
    void targetInit(YOLODetectionResult target_body_coord);

    /**
     * @brief Reid单目标追踪
     *
     * @return int 跟踪结果的索引，如果未找到匹配的目标则返回-1
     */
    int reidTrack(std::vector<YOLODetectionResult>& body_detection_results);

    /**
     * @brief Reid局部验证——
     * @brief 将当前成功追踪的目标特征和保存的上一帧追踪目标特征进行比较，如果距离大于设定的欧式距离阈值，验证失败，否则验证成功
     *
     * @return true 验证成功
     * @return false 验证失败
     */
    bool reidLocalVerification(const YOLODetectionResult& target_body_coord, int index, const std::vector<YOLODetectionResult>& body_detection_results);

    /**
     * @brief 更新追踪目标的特征队列——
     * @brief 根据当前追踪到的对象姿态是全身还是半身，若为全身特征则同时更新全身与半身特征队列，若为半身特征则仅更新半身特征队列。全身和半身特征队列有数量上限list_limit_
     *
     * @param index 当前追踪到的对象的索引
     */
    void updateTargetFeatureList(YOLODetectionResult target_body_coord, std::tuple<int, std::array<float, 4>, std::array<float, 4>> target_pose);

    /**
     * @brief 调用API获取指定索引的目标特征
     *
     * @param index 目标的索引
     */
    void getFeature(YOLODetectionResult target_body_coord);

    bool isOverLap(const std::vector<YOLODetectionResult>& yolo_detection_results, int index, const std::vector<Box>& pose_boxes);

private:
    /**
     * @brief 计算两个特征之间的欧氏距离
     *
     * @param feature1 特征向量1
     * @param feature2 特征向量2
     * @return double 欧氏距离
     */
    double calculateDistance(const std::vector<double> feature1, const std::vector<double> feature2);

    /**
     * @brief 计算两个特征列表之间的最小欧氏距离
     *
     * @param feature_list1 特征列表1
     * @param feature_list2 特征列表2
     * @return std::vector<double> 包含每个feature_list2中特征与feature_list1中特征的最小距离
     */
    std::vector<double> calculateMinimalDistances(const std::vector<std::vector<double>> feature_list1, const std::vector<std::vector<double>> feature_list2);

    /**
     * @brief 在当前帧中搜索和目标特征列表匹配的低于Reid阈值的目标，且若成功匹配更新上一帧的目标特征
     *
     * @param FeatureList 目标特征列表
     * @param debug_log_state 调试日志变量
     * @return int 匹配目标的索引，如果未找到匹配的目标则返回-1
     */
    int searchTarget(const std::vector<std::vector<double>>& feature_list, const std::vector<std::vector<double>>& frame_features, int debug_log_state, double reid_threshold);

    /**
     * @brief 更新目标特征列表——
     * @brief 将当前追踪到的对象特征与目标特征列表中所有特征进行比较，所有欧式距离都大于阈值，视为更新成功，否则视为更新失败
     *
     * @param target_feature_list 目标特征列表
     * @param debug_log_distance 调试日志变量
     * @param threshold 两特征向量之间欧式距离阈值
     * @return true 更新成功
     * @return false 更新失败
     */
    bool updateFeatureList(std::vector<std::vector<double>>& target_feature_list, double& debug_log_distance, const double threshold);

private:
    int pose_;                                                  /**< 当前追踪到的对象的姿态 */
    std::vector<double> previous_target_feature_;               /**< 上一帧的目标特征 */
    std::vector<double> current_target_feature_;                /**< 当前帧的目标特征 */
    std::vector<std::vector<double>> current_frame_features_;   /**< 当前帧的所有人的特征 */
    std::vector<std::vector<double>> target_feature_base_full_; /**< 目标特征的全身基础列表 */
    std::vector<std::vector<double>> target_feature_base_half_; /**< 目标特征的半身基础列表 */
    std::vector<std::vector<double>> target_feature_list_full_; /**< 目标特征的全身列表 */
    std::vector<std::vector<double>> target_feature_list_half_; /**< 目标特征的半身列表 */

    double screen_width_;
    double screen_height_;

    int list_limit_ = 8;                         /**< 可调参数：特征列表的长度限制 */
    double reid_full_threshold_ = 2.8;           /**< 可调参数：全局全身Reid阈值 */
    double reid_half_threshold_ = 2.5;           /**< 可调参数：全局半身Reid阈值 */
    double front_verification_threshold_ = 5.85; /**< 可调参数：目标人在前重叠上下帧Reid验证阈值 */
    double back_verification_threshold_ = 3.0;   /**< 可调参数：目标人在后重叠上下帧Reid验证阈值 */
    double feature_list_full_threshold_ = 2.0;   /**< 可调参数：全身特征列表的阈值 */
    double feature_list_half_threshold_ = 2.0;   /**< 可调参数：半身特征列表的阈值 */

public:
    bool full_isInit = false;
    bool half_isInit = false;
    int debug_log_byte_state_ = 1;         /**< 调试日志变量：byte追踪状态 */
    int debug_log_reid_state_ = 0;         /**< 调试日志变量：reid追踪状态 */
    int debug_log_pose_state_ = 0;         /**< 调试日志变量：追踪目标的姿态 */
    int debug_log_list_full_ = 0;          /**< 调试日志变量：全身特征的数量 */
    int debug_log_list_half_ = 0;          /**< 调试日志变量：半身特征的数量 */
    double debug_log_distance_full_ = 0.0; /**< 调试日志变量：与全身队列的最小欧式距离 */
    double debug_log_distance_half_ = 0.0; /**< 调试日志变量：与半身队列的最小欧式距离 */
    double debug_log_distance_pre_ = 0.0;  /**< 调试日志变量：与上一帧目标的欧式距离 */
};
}
