#include <vector>

#include "DoubleFeatureNet.h"
#include "XbotgoSingleStrategyCallback.h"

namespace XbotgoSDK
{

extern XbotgoSingleStrategyCallback* g_xbotgoSingleStrategyCallbacks;

/**
 * @brief 构造函数，FeatureExtractor
 */
DoubleFeatureNet::DoubleFeatureNet()
{
    // 构造函数实现
}

/**
 * @brief 初始化目标特征列表和阈值
 *
 * @param index 要追踪的2个目标的下标
 * @param reid_threshold reid阈值
 */
void DoubleFeatureNet::targetInit(std::vector<int> index, double reid_threshold)
{
    reid_threshold_ = reid_threshold;
    for (int i = 0; i < index.size(); i++) {
        // 获取每一个目标的特征
        std::vector<double> full_feature = g_xbotgoSingleStrategyCallbacks->onGetFeature(index[i]);
        target_feature_base_.push_back(full_feature);
        previous_target_feature_.push_back(full_feature); // 初始化时取第一个目标的特征作为上一帧的目标特征
        current_target_feature_ = previous_target_feature_;
        target_feature_list_full_[i]
            .push_back(full_feature);
    }

    std::cout << "DoubleFeatureNet::targetInit target_feature_base_ size = " << target_feature_base_.size() << std::endl;
    std::cout << "DoubleFeatureNet::targetInit target_feature_base_ size = " << target_feature_base_.size() << std::endl;
    std::cout << "DoubleFeatureNet::targetInit target_feature_list_full_ size = " << target_feature_list_full_.size() << std::endl;
    std::cout << "DoubleFeatureNet::targetInit target_feature_list_full_[0] size = " << target_feature_list_full_[0].size() << std::endl;
    std::cout << "DoubleFeatureNet::targetInit target_feature_list_full_[1] size = " << target_feature_list_full_[1].size() << std::endl;
}

/**
 * @brief Reid单目标追踪
 * @param id 表示第几个目标 0 表示 第一个目标 1 表示第二个目标
 * @return int 跟踪结果的索引，如果未找到匹配的目标则返回-1
 */
int DoubleFeatureNet::reidTrack(int id)
{
    std::cout << "DoubleFeatureNet::reidTrack id = " << id << std::endl;
    // 调用API获取当前帧所有人的特征
    current_frame_features_ = g_xbotgoSingleStrategyCallbacks->onGetAllFeatures();
    // std::cout << "DoubleFeatureNet::reidTrack, current_frame_features_="<<current_frame_features_.size()<<std::endl;

    // 分别从基础、全身、半身特征队列中查找
    int index = searchTarget(target_feature_base_, id, reid_base_);
    if (index != -1)
        return index;

    index = searchTarget(target_feature_list_full_[id], id, reid_full_);
    if (index != -1)
        return index;

    // --------------- Debug Log ---------------
    debug_log_state_ = reid_none_;
    // --------------- Debug Log ---------------
    return -1;
}

/**
 * @brief 在当前帧中搜索和目标特征列表匹配的低于Reid阈值的目标，且若成功匹配更新上一帧的目标特征
 *
 * @param feature_list 目标特征列表
 * @param id 表示第几个目标 0 表示 第一个目标 1 表示第二个目标
 * @param debug_log_state 调试日志变量
 * @return int 匹配目标的索引，如果未找到匹配的目标则返回-1
 */
int DoubleFeatureNet::searchTarget(const std::vector<std::vector<double>>& feature_list, const int id, int debug_log_state)
{

    std::cout << "DoubleFeatureNet::search target debug_log_state" << debug_log_state << std::endl;

    for (size_t i = 0; i < feature_list.size(); i++) {
        std::cout << "DoubleFeatureNet::search target i=" << i << ",size=" << feature_list[i].size() << std::endl;
    }

    std::cout << "DoubleFeatureNet::search target feature_list =" << feature_list.size() << std::endl;
    std::cout << "DoubleFeatureNet::search target current_frame_features_ =" << current_frame_features_.size() << std::endl;

    auto distances = calculateMinimalDistances(feature_list, current_frame_features_);
    auto min_distance = std::min_element(distances.begin(), distances.end());
    int min_index = std::distance(distances.begin(), min_distance);

    // ------------------------ Debug Log ------------------------
    debug_log_state_ = debug_log_state;
    if (debug_log_state == reid_base_)
        debug_log_distance_full_ = *min_distance;
    if (debug_log_state == reid_full_)
        debug_log_distance_full_ = *min_distance;
    // ------------------------ Debug Log ------------------------

    if (*min_distance < reid_threshold_) {
        // ------------------------ Debug Log ------------------------
        debug_log_distance_full_ = (debug_log_state == reid_full_ || debug_log_state == reid_base_) ? *min_distance : 0.0;
        // ------------------------ Debug Log ------------------------

        previous_target_feature_[id] = current_frame_features_[min_index];
        return min_index;
    }
    return -1;
}

/**
 * @brief 调用API获取指定索引的目标特征
 * @param id 表示第几个目标 0 表示 第一个目标 1 表示第二个目标
 * @param index 目标的索引
 */
void DoubleFeatureNet::getFeature(const int id, const int index)
{
    current_target_feature_[id] = g_xbotgoSingleStrategyCallbacks->onGetFeature(index);
    // std::cout << "DoubleFeatureNet::getFeature ="<<current_target_feature_.size()<< std::endl;
}

/**
 * @brief 更新上一帧的目标特征
 */
void DoubleFeatureNet::updatePreviousTargetFeature()
{
    previous_target_feature_ = current_target_feature_;
}

/**
 * @brief Reid局部验证——
 * @brief 将当前成功追踪的目标特征和保存的上一帧追踪目标特征进行比较，如果距离大于设定的欧式距离阈值，验证失败，否则验证成功
 *
 * @return true 验证成功
 * @return false 验证失败
 */
bool DoubleFeatureNet::reidLocalVerification(const int id)
{
    double distance = calculateDistance(current_target_feature_[id], previous_target_feature_[id]);

    // ------------------------ Debug Log ------------------------
    debug_log_distance_pre_ = distance;
    // ------------------------ Debug Log ------------------------

    if (distance > local_verification_threshold_) {
        return false;
    }
    return true;
};

/**
 * @brief 更新目标特征列表——
 * @brief 将当前追踪到的对象特征与目标特征列表中所有特征进行比较，所有欧式距离都大于阈值，视为更新成功，否则视为更新失败
 * @param id 表示第几个目标 0 表示 第一个目标 1 表示第二个目标
 * @param target_feature_list 目标特征列表
 * @param debug_log_distance 调试日志变量
 * @param threshold 两特征向量之间欧式距离阈值
 * @return true 更新成功
 * @return false 更新失败
 */
bool DoubleFeatureNet::updateFeatureList(const int id, std::vector<std::vector<double>>& target_feature_list, double& debug_log_distance, const double threshold)
{
    double distance = 0.0;
    double distance_min = 999.0;
    for (size_t k = 0; k < target_feature_list.size(); ++k) {
        distance = calculateDistance(current_target_feature_[id], target_feature_list[k]);

        // ------------------------ Debug Log ------------------------
        distance_min = std::min(distance_min, distance);
        debug_log_distance = distance_min;
        // ------------------------ Debug Log ------------------------

        if (distance < threshold) {
            // ------------------------ Debug Log ------------------------
            debug_log_distance = distance;
            // ------------------------ Debug Log ------------------------

            return false;
        }
    }
    target_feature_list.push_back(current_target_feature_[id]);
    return true;
}

/**
 * @brief 更新追踪目标的特征队列——
 * @brief 根据当前追踪到的对象姿态是全身还是半身，若为全身特征则同时更新全身与半身特征队列，若为半身特征则仅更新半身特征队列。全身和半身特征队列有数量上限list_limit_
 * @param id 表示第几个目标 0 表示 第一个目标 1 表示第二个目标
 * @param index 当前追踪到的对象的索引
 */
void DoubleFeatureNet::updateTargetFeatureList(const int id, const int index)
{
    // 调用API获取姿态pose
    pose_ = g_xbotgoSingleStrategyCallbacks->onGetPose(index);

    // ------------------------ Debug Log ------------------------
    debug_log_pose_ = static_cast<int>(pose_);
    debug_log_distance_full_ = 0.0;
    debug_log_distance_half_ = 0.0;
    // ------------------------ Debug Log ------------------------

    if (pose_ == pose_full_) {
        updateFeatureList(id, target_feature_list_full_[id], debug_log_distance_full_, feature_list_full_threshold_);
        // 更新全身特征队列targetFeatureList_full
        // if (updateFeatureList(target_feature_list_full_, debug_log_distance_full_, feature_list_full_threshold_))
        // {
        //     // 调用API获取半身特征，更新半身特征队列targetFeatureList_half
        //     current_target_feature_ = g_xbotgoSingleStrategyCallbacks->onGetHalfFeature(index);
        //     std::cout << "DoubleFeatureNet::updateTargetFeatureList onGetHalfFeature="<<current_target_feature_.size()<<std::endl;
        //     updateFeatureList(target_feature_list_half_, debug_log_distance_half_, feature_list_half_threshold_);
        // }
    }

    // ------------------------ Debug Log ------------------------
    debug_log_list_full_ = target_feature_list_full_.size();
    // debug_log_list_half_ = target_feature_list_half_.size();
    // ------------------------ Debug Log ------------------------

    // 若超出list_limit，删除最早的一个
    if (static_cast<int>(target_feature_list_full_.size()) > list_limit_) {
        target_feature_list_full_.erase(target_feature_list_full_.begin());
    }
    // if (static_cast<int>(target_feature_list_half_.size()) > list_limit_)
    // {
    //     target_feature_list_half_.erase(target_feature_list_half_.begin());
    // }
};

/**
 * @brief 计算两个特征之间的欧氏距离
 *
 * @param feature1 特征向量1
 * @param feature2 特征向量2
 * @return double 欧氏距离
 */
double DoubleFeatureNet::calculateDistance(const std::vector<double> feature1, const std::vector<double> feature2)
{
    if (feature1.size() != feature2.size()) {
        return -1.0; // 返回错误值或抛出异常
    }

    double distance = 0.0;
    for (size_t i = 0; i < feature1.size(); ++i) {
        double diff = feature1[i] - feature2[i];
        distance += diff * diff;
    }
    return distance;
}

/**
 * @brief 计算两个特征列表之间的最小欧氏距离
 *
 * @param feature_list1 特征列表1
 * @param feature_list2 特征列表2
 * @return std::vector<double> 包含每个feature_list2中特征与feature_list1中特征的最小距离
 */
std::vector<double> DoubleFeatureNet::calculateMinimalDistances(const std::vector<std::vector<double>> feature_list1, const std::vector<std::vector<double>> feature_list2)
{
    std::vector<double> distances;
    double distance = 0.0;
    for (size_t i = 0; i < feature_list2.size(); i++) {
        distances.push_back(999);
        for (size_t k = 0; k < feature_list1.size(); k++) {
            distance = calculateDistance(feature_list2[i], feature_list1[k]);
            if (distance < distances[i]) {
                distances[i] = distance;
            }
        }
    }
    return distances;
}

/**
 * @brief 判断追踪目标检测结果框是否重叠或即将出现重叠
 *
 * @param yolo_detection_results 当前帧图片的YOLO检测结果
 * @param index 追踪目标检测结果的索引
 * @param scale 扩大检测结果框的尺度，默认为1.0倍
 * @return true 如果存在重叠
 * @return false 如果不重叠
 */
bool isOverLapping_(const std::vector<YOLODetectionResult>& yolo_detection_results, int index, float scale)
{
    // 扩大检测结果框的长宽至原来的scale倍，默认为1.0倍
    // float width = yolo_detection_results[index].width * scale;
    // float height = yolo_detection_results[index].height * scale;
    for (int i = 0; i < static_cast<int>(yolo_detection_results.size()); i++) {
        if (i != index) {
            int x_overlap = std::max(0.0,
                                     std::min(yolo_detection_results[index].x + 0.5 * yolo_detection_results[index].width,
                                              yolo_detection_results[i].x + 0.5 * yolo_detection_results[i].width) -
                                         std::max(yolo_detection_results[index].x - 0.5 * yolo_detection_results[index].width,
                                                  yolo_detection_results[i].x - 0.5 * yolo_detection_results[i].width));

            if (x_overlap != 0.0) {
                int y_overlap = std::max(0.0,
                                         std::min(yolo_detection_results[index].y + 0.5 * yolo_detection_results[index].height,
                                                  yolo_detection_results[i].y + 0.5 * yolo_detection_results[i].height) -
                                             std::max(yolo_detection_results[index].y - 0.5 * yolo_detection_results[index].height,
                                                      yolo_detection_results[i].y - 0.5 * yolo_detection_results[i].height));

                int intersection_area = x_overlap * y_overlap;
                int target_area = yolo_detection_results[index].width * yolo_detection_results[index].height;

                // 如果重叠部分面积大于目标框的五分之一，则认为重叠
                if (intersection_area > target_area / 5) {
                    return true;
                }
            }
        }
    }
    return false;
}
} // namespace XbotgoSDK
