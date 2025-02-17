#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

#include "DetectionResult.h"

#define pose_full_ 0
#define pose_half_ 1
#define pose_none_ 2
#define pose_multi_ 3

namespace XbotgoSDK
{

#define byte_none_ 0    /**< 标志位：ByteTrack追踪失败，且连续失败次数小于2 */
#define byte_only_ 1    /**< 标志位：ByteTrack追踪成功，且追踪目标没有和其他人有重叠情况 */
#define byte_overlap_ 2 /**< 标志位：ByteTrack追踪成功且Reid局部验证通过，但追踪目标存在和其他人有重叠情况 */
#define reid_none_ 3    /**< 标志位：Reid追踪失败 */
#define reid_full_ 4    /**< 标志位：Reid追踪成功,从全身特征队列中追踪到目标 */
#define reid_half_ 5    /**< 标志位：Reid追踪成功,从半身特征队列中追踪到目标 */
#define reid_base_ 6    /**< 标志位：Reid追踪成功,从基础特征队列中追踪到目标 */

/**
 * @brief 判断追踪目标检测结果框是否重叠或即将出现重叠
 *
 * @param yolo_detection_results 当前帧图片的YOLO检测结果
 * @param index 追踪目标检测结果的索引
 * @param scale 扩大检测结果框的尺度，默认为1.0倍
 * @return true 如果存在重叠
 * @return false 如果不重叠
 */
bool isOverLapping(const std::vector<YOLODetectionResult>& yolo_detection_results, int index, float scale = 1.0);

/**
 * @brief 特征向量网络类，用于目标特征的提取、保存和Reid单目标追踪
 */
class FeatureNet
{
public:
    /**
     * @brief 构造函数
     *
     * @param feature_extractor_ FeatureExtractor对象指针
     */
    FeatureNet();

    /**
     * @brief 初始化目标特征列表和阈值
     *
     * @param index 要追踪目标的下标
     * @param reid_threshold reid阈值
     */
    void targetInit(int index, double reid_threshold);

    /**
     * @brief Reid单目标追踪
     *
     * @return int 跟踪结果的索引，如果未找到匹配的目标则返回-1
     */
    int reidTrack();

    /**
     * @brief Reid局部验证——
     * @brief 将当前成功追踪的目标特征和保存的上一帧追踪目标特征进行比较，如果距离大于设定的欧式距离阈值，验证失败，否则验证成功
     *
     * @return true 验证成功
     * @return false 验证失败
     */
    bool reidLocalVerification();

    /**
     * @brief 更新追踪目标的特征队列——
     * @brief 根据当前追踪到的对象姿态是全身还是半身，若为全身特征则同时更新全身与半身特征队列，若为半身特征则仅更新半身特征队列。全身和半身特征队列有数量上限list_limit_
     *
     * @param index 当前追踪到的对象的索引
     */
    void updateTargetFeatureList(const int index);

    /**
     * @brief 调用API获取指定索引的目标特征
     *
     * @param index 目标的索引
     */
    void getFeature(const int index);

    /**
     * @brief 更新上一帧的目标特征
     */
    void updatePreviousTargetFeature();

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
    int searchTarget(const std::vector<std::vector<double>>& feature_list, int state);

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
    bool updateFeatureList(std::vector<std::vector<double>>& target_feature_list, double& feature_extractor_distance, const double threshold);

private:
    int pose_;                                                  /**< 当前追踪到的对象的姿态 */
    std::vector<double> previous_target_feature_;               /**< 上一帧的目标特征 */
    std::vector<double> current_target_feature_;                /**< 当前帧的目标特征 */
    std::vector<std::vector<double>> current_frame_features_;   /**< 当前帧的所有人的特征 */
    std::vector<std::vector<double>> target_feature_base_;      /**< 目标特征的基础列表 */
    std::vector<std::vector<double>> target_feature_list_full_; /**< 目标特征的全身列表 */
    // std::vector<std::vector<double>> target_feature_list_half_; /**< 目标特征的半身列表 */

    double reid_threshold_;                     /**< 用户传入参数：全局Reid阈值。目前iOS为2.5，Android为1.7*/
    int list_limit_ = 8;                        /**< 固定参数：特征列表的长度限制 */
    double local_verification_threshold_ = 3.0; /**< 固定参数：局部Reid验证的阈值。目前iOS为2.0，Android为1.5 */
    double feature_list_full_threshold_ = 3.0;  /**< 固定参数：全身特征列表的阈值。目前iOS为2.0，Android为1.5 */
    // double feature_list_half_threshold_ = 2.0;  /**< 固定参数：半身特征列表的阈值。目前iOS为2.0，Android为1.5 */

public:
    int debug_log_state_ = 0;              /**< 调试日志变量：追踪状态 */
    int debug_log_pose_ = 0;               /**< 调试日志变量：追踪目标的姿态 */
    int debug_log_list_full_ = 0;          /**< 调试日志变量：全身特征的数量 */
    int debug_log_list_half_ = 0;          /**< 调试日志变量：半身特征的数量 */
    double debug_log_distance_full_ = 0.0; /**< 调试日志变量：与全身队列的最小欧式距离 */
    double debug_log_distance_half_ = 0.0; /**< 调试日志变量：与半身队列的最小欧式距离 */
    double debug_log_distance_pre_ = 0.0;  /**< 调试日志变量：与上一帧目标的欧式距离 */
};
}
