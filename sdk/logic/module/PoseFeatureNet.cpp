#include "PoseFeatureNet.h"
#include "XbotgoSingleStrategyCallback.h"

namespace XbotgoSDK
{

extern XbotgoSingleStrategyCallback*  g_xbotgoSingleStrategyCallbacks;

/**
 * @brief 构造函数，FeatureExtractor
 *
 * @param feature_extractor_ FeatureExtractor对象指针
 */
PoseFeatureNet::PoseFeatureNet(const double &screen_width, const double &screen_height)
    : screen_width_(screen_width), screen_height_(screen_height) {}

/**
 * @brief 初始化目标特征列表和阈值
 *
 * @param targetFeatures 要追踪目标的特征
 * @param reid_threshold reid阈值
 */
void PoseFeatureNet::targetInit(YOLODetectionResult target_body_coord)
{
    YOLODetectionResult padding_rect = paddingRect(target_body_coord, screen_width_, screen_height_);
    std::tuple<int, std::array<float, 4>, std::array<float, 4>> target_pose = analysisPose(target_body_coord, padding_rect, postProcess(g_xbotgoSingleStrategyCallbacks->onPoseDetect(padding_rect)));
    pose_ = std::get<0>(target_pose);

    if (pose_ == _pose_full_)
    {
        // fullbody
        std::array<float, 4> fullbody_rect = std::get<1>(target_pose);
        YOLODetectionResult target_coord = {0, 1, padding_rect.x + (fullbody_rect[0] * padding_rect.width),
                                            padding_rect.y + (fullbody_rect[1] * padding_rect.height),
                                            padding_rect.width * (fullbody_rect[2] - fullbody_rect[0]),
                                            padding_rect.height * (fullbody_rect[3] - fullbody_rect[1])};
        previous_target_feature_ = g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord);
        target_feature_base_full_.push_back(previous_target_feature_);
        target_feature_list_full_.push_back(previous_target_feature_);
        // halfbody
        std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
        target_coord = {0, 1, padding_rect.x + (halfbody_rect[0] * padding_rect.width),
                        padding_rect.y + (halfbody_rect[1] * padding_rect.height),
                        padding_rect.width * (halfbody_rect[2] - halfbody_rect[0]),
                        padding_rect.height * (halfbody_rect[3] - halfbody_rect[1])};
        target_feature_base_half_.push_back(g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord));
        target_feature_list_half_ = target_feature_base_half_;

        full_isInit = true;
        half_isInit = true;
    }
    else if (pose_ == _pose_half_)
    {
        // halfbody
        std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
        YOLODetectionResult target_coord = {0, 1, padding_rect.x + (halfbody_rect[0] * padding_rect.width), padding_rect.y + (halfbody_rect[1] * padding_rect.height),
                                            padding_rect.width * (halfbody_rect[2] - halfbody_rect[0]), padding_rect.height * (halfbody_rect[3] - halfbody_rect[1])};
        std::vector<double> target_feature = g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord);
        target_feature_list_half_.push_back(target_feature);
        target_feature_base_half_.push_back(target_feature);
        previous_target_feature_ = target_feature;

        half_isInit = true;
    }

    debug_log_pose_state_ = pose_;
    debug_log_list_full_ = target_feature_list_full_.size();
    debug_log_list_half_ = target_feature_list_half_.size();
}

/**
 * @brief Reid单目标追踪
 *
 * @return int 跟踪结果的索引，如果未找到匹配的目标则返回-1
 */
int PoseFeatureNet::reidTrack(std::vector<YOLODetectionResult> &body_detection_results)
{
    // 调用API获取当前帧所有人的特征
    std::vector<std::vector<double>> current_frame_full_features_;
    std::vector<std::vector<double>> current_frame_half_features_;
    for (size_t i = 0; i < body_detection_results.size(); i++)
    {
        YOLODetectionResult padding_rect = paddingRect(body_detection_results[i], screen_width_, screen_height_);
        std::tuple<int, std::array<float, 4>, std::array<float, 4>> target_pose = analysisPose(body_detection_results[i], padding_rect, postProcess(g_xbotgoSingleStrategyCallbacks->onPoseDetect(padding_rect)));
        pose_ = std::get<0>(target_pose);
        if (pose_ == _pose_full_)
        {
            std::array<float, 4> fullbody_rect = std::get<1>(target_pose);
            YOLODetectionResult target_coord = {0, 1, padding_rect.x + (fullbody_rect[0] * padding_rect.width),
                                                padding_rect.y + (fullbody_rect[1] * padding_rect.height),
                                                padding_rect.width * (fullbody_rect[2] - fullbody_rect[0]),
                                                padding_rect.height * (fullbody_rect[3] - fullbody_rect[1])};
            current_frame_full_features_.push_back(g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord));
            if (full_isInit)
            {
                current_frame_half_features_.push_back(std::vector<double>());
            }
            else
            {
                std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
                target_coord = {0, 1, padding_rect.x + (halfbody_rect[0] * padding_rect.width), padding_rect.y + (halfbody_rect[1] * padding_rect.height),
                                padding_rect.width * (halfbody_rect[2] - halfbody_rect[0]), padding_rect.height * (halfbody_rect[3] - halfbody_rect[1])};
                current_frame_half_features_.push_back(g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord));
            }
        }
        else if (pose_ == _pose_half_)
        {
            std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
            YOLODetectionResult target_coord = {0, 1, padding_rect.x + (halfbody_rect[0] * padding_rect.width), padding_rect.y + (halfbody_rect[1] * padding_rect.height),
                                                padding_rect.width * (halfbody_rect[2] - halfbody_rect[0]), padding_rect.height * (halfbody_rect[3] - halfbody_rect[1])};
            current_frame_full_features_.push_back(std::vector<double>());
            current_frame_half_features_.push_back(g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord));
        }
        else if (pose_ == _pose_shoulder_)
        {
            std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
            YOLODetectionResult target_coord = {0, 1, padding_rect.x + (halfbody_rect[0] * padding_rect.width), padding_rect.y + (halfbody_rect[1] * padding_rect.height),
                                                padding_rect.width * (halfbody_rect[2] - halfbody_rect[0]), padding_rect.height * (halfbody_rect[3] - halfbody_rect[1])};
            current_frame_full_features_.push_back(std::vector<double>());
            current_frame_half_features_.push_back(g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord));
        }
        else
        {
            current_frame_full_features_.push_back(std::vector<double>());
            current_frame_half_features_.push_back(std::vector<double>());
        }
    }

    // --------------- Debug Log ---------------
    debug_log_distance_full_ = 0;
    debug_log_distance_half_ = 0;
    // --------------- Debug Log ---------------

    // 分别从基础、全身、半身特征队列中查找
    int index = searchTarget(target_feature_base_full_, current_frame_full_features_, _reid_full_base_, reid_full_threshold_);
    if (index != -1)
    {
        debug_log_pose_state_ = _pose_full_;
        return index;
    }

    index = searchTarget(target_feature_base_half_, current_frame_half_features_, _reid_half_base_, reid_half_threshold_);
    if (index != -1)
    {
        debug_log_pose_state_ = _pose_half_;
        return index;
    }

    index = searchTarget(target_feature_list_full_, current_frame_full_features_, _reid_full_, reid_full_threshold_);
    if (index != -1)
    {
        debug_log_pose_state_ = _pose_full_;
        return index;
    }

    index = searchTarget(target_feature_list_half_, current_frame_half_features_, _reid_half_, reid_half_threshold_);
    if (index != -1)
    {
        debug_log_pose_state_ = _pose_half_;
        return index;
    }

    debug_log_reid_state_ = _reid_lose_;

    return -1;
}

/**
 * @brief 在当前帧中搜索和目标特征列表匹配的低于Reid阈值的目标，且若成功匹配更新上一帧的目标特征
 *
 * @param FeatureList 目标特征列表
 * @param debug_log_state 调试日志变量
 * @return int 匹配目标的索引，如果未找到匹配的目标则返回-1
 */
int PoseFeatureNet::searchTarget(const std::vector<std::vector<double>> &feature_list, const std::vector<std::vector<double>> &frame_features, int debug_log_state, double reid_threshold)
{
    if (feature_list.empty() || frame_features.empty())
    {
        if (debug_log_state == _reid_full_)
        {
            debug_log_distance_full_ = -1.0;
        }
        if (debug_log_state == _reid_half_)
        {
            debug_log_distance_full_ = -1.0;
        }
        return -1;
    }

    auto distances = calculateMinimalDistances(feature_list, frame_features);
    auto min_distance = std::min_element(distances.begin(), distances.end());
    int min_index = std::distance(distances.begin(), min_distance);

    // ------------------------ Debug Log ------------------------
    debug_log_reid_state_ = debug_log_state;
    if (debug_log_state == _reid_full_ || debug_log_state == _reid_full_base_)
    {
        debug_log_distance_full_ = *min_distance;
    }
    if (debug_log_state == _reid_half_ || debug_log_state == _reid_half_base_)
    {
        debug_log_distance_half_ = *min_distance;
    }
    // ------------------------ Debug Log ------------------------

    if (*min_distance < reid_threshold)
    {
        // ------------------------ Debug Log ------------------------
        // debug_log_distance_full_ = (debug_log_state == reid_full_ || debug_log_state == reid_full_base_) ? *min_distance : 0.0;
        // debug_log_distance_half_ = (debug_log_state == reid_half_ || debug_log_state == reid_half_base_) ? *min_distance : 0.0;
        // ------------------------ Debug Log ------------------------

        previous_target_feature_ = frame_features[min_index];
        return min_index;
    }
    return -1;
}

/**
 * @brief 调用API获取指定索引的目标特征
 *
 * @param index 目标的索引
 */
void PoseFeatureNet::getFeature(YOLODetectionResult target_body_coord)
{
    current_target_feature_ = g_xbotgoSingleStrategyCallbacks->onGetFeature(target_body_coord);
}

/**
 * @brief Reid局部验证——
 * @brief 将当前成功追踪的目标特征和保存的上一帧追踪目标特征进行比较，如果距离大于设定的欧式距离阈值，验证失败，否则验证成功
 *
 * @return true 验证成功
 * @return false 验证失败
 */
bool PoseFeatureNet::reidLocalVerification(const YOLODetectionResult &target_body_coord, int index, const std::vector<YOLODetectionResult> &body_detection_results)
{
    // 提取当前目标的特征
    current_target_feature_ = g_xbotgoSingleStrategyCallbacks->onGetFeature(target_body_coord);

    // 计算当前目标特征与之前目标特征之间的距离
    double distance = calculateDistance(current_target_feature_, previous_target_feature_);

    // ------------------------ Debug Log ------------------------
    debug_log_distance_pre_ = distance;
    // ------------------------ Debug Log ------------------------

    // 根据目标是否在前面，选择不同的验证阈值
    // double threshold = isTargetInFront(target_body_coord, index, body_detection_results) ? front_verification_threshold_ : back_verification_threshold_;
    // 判断距离是否超过阈值
    // return distance <= threshold;
    if (isTargetInFront(index, body_detection_results))
    {
        if (distance <= front_verification_threshold_)
        {
            debug_log_byte_state_ = _byte_success_overlap_front_;
            return true;
        }
        debug_log_byte_state_ = _byte_wrong_overlap_front_;
    }
    else
    {
        if (distance <= back_verification_threshold_)
        {
            debug_log_byte_state_ = _byte_success_overlap_back_;
            return true;
        }
        debug_log_byte_state_ = _byte_wrong_overlap_back_;
    }
    return false;
};

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
bool PoseFeatureNet::updateFeatureList(std::vector<std::vector<double>> &target_feature_list, double &debug_log_distance, const double threshold)
{
    double distance = 0.0;
    double distance_min = 999.0;
    for (size_t k = 0; k < target_feature_list.size(); ++k)
    {
        distance = calculateDistance(current_target_feature_, target_feature_list[k]);

        // ------------------------ Debug Log ------------------------
        distance_min = std::min(distance_min, distance);
        debug_log_distance = distance_min;
        // ------------------------ Debug Log ------------------------

        if (distance < threshold)
        {
            // ------------------------ Debug Log ------------------------
            debug_log_distance = distance;
            // ------------------------ Debug Log ------------------------

            return false;
        }
    }
    target_feature_list.push_back(current_target_feature_);
    return true;
}

/**
 * @brief 更新追踪目标的特征队列——
 * @brief 根据当前追踪到的对象姿态是全身还是半身，若为全身特征则同时更新全身与半身特征队列，若为半身特征则仅更新半身特征队列。全身和半身特征队列有数量上限list_limit_
 *
 * @param index 当前追踪到的对象的索引
 */
void PoseFeatureNet::updateTargetFeatureList(YOLODetectionResult target_body_coord, std::tuple<int, std::array<float, 4>, std::array<float, 4>> target_pose)
{
    pose_ = std::get<0>(target_pose);

    // ------------------------ Debug Log ------------------------
    debug_log_pose_state_ = pose_;
    debug_log_distance_full_ = 0.0;
    debug_log_distance_half_ = 0.0;
    // ------------------------ Debug Log ------------------------

    if (pose_ == _pose_full_)
    {
        // 更新全身特征队列targetFeatureList_full
        std::array<float, 4> fullbody_rect = std::get<1>(target_pose);
        YOLODetectionResult target_coord = {0, 1, target_body_coord.x + (fullbody_rect[0] * target_body_coord.width),
                                            target_body_coord.y + (fullbody_rect[1] * target_body_coord.height),
                                            target_body_coord.width * (fullbody_rect[2] - fullbody_rect[0]),
                                            target_body_coord.height * (fullbody_rect[3] - fullbody_rect[1])};
        current_target_feature_ = g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord);
        previous_target_feature_ = current_target_feature_;
        if (!full_isInit)
        {
            // fullbody
            target_feature_list_full_.push_back(current_target_feature_);
            target_feature_base_full_.push_back(current_target_feature_);
            full_isInit = true;

            // halfbody
            std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
            target_coord = {0, 1, target_body_coord.x + (halfbody_rect[0] * target_body_coord.width),
                            target_body_coord.y + (halfbody_rect[1] * target_body_coord.height),
                            target_body_coord.width * (halfbody_rect[2] - halfbody_rect[0]),
                            target_body_coord.height * (halfbody_rect[3] - halfbody_rect[1])};
            if (!half_isInit)
            {
                target_feature_base_half_.push_back(g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord));
                target_feature_list_half_ = target_feature_base_half_;
                half_isInit = true;
            }
            else
            {
                updateFeatureList(target_feature_list_half_, debug_log_distance_half_, feature_list_half_threshold_);
            }
        }
        else
        {
            if (updateFeatureList(target_feature_list_full_, debug_log_distance_full_, feature_list_full_threshold_))
            {
                // 调用API获取半身特征，更新半身特征队列targetFeatureList_half
                std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
                target_coord = {0,1, target_body_coord.x + (halfbody_rect[0] * target_body_coord.width),
                                target_body_coord.y + (halfbody_rect[1] * target_body_coord.height),
                                target_body_coord.width * (halfbody_rect[2] - halfbody_rect[0]),
                                target_body_coord.height * (halfbody_rect[3] - halfbody_rect[1])};
                current_target_feature_ = g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord);
                updateFeatureList(target_feature_list_half_, debug_log_distance_half_, feature_list_half_threshold_);
            }
        }
    }
    else if (pose_ == _pose_half_)
    {
        // 更新半身特征队列targetFeatureList_half
        std::array<float, 4> halfbody_rect = std::get<2>(target_pose);
        YOLODetectionResult target_coord = {0, 1, target_body_coord.x + (halfbody_rect[0] * target_body_coord.width),
                                            target_body_coord.y + (halfbody_rect[1] * target_body_coord.height),
                                            target_body_coord.width * (halfbody_rect[2] - halfbody_rect[0]),
                                            target_body_coord.height * (halfbody_rect[3] - halfbody_rect[1])};
        current_target_feature_ = g_xbotgoSingleStrategyCallbacks->onGetFeature(target_coord);
        previous_target_feature_ = current_target_feature_;
        if (!half_isInit)
        {
            // halfbody
            target_feature_list_half_.push_back(current_target_feature_);
            target_feature_base_half_.push_back(current_target_feature_);
            half_isInit = true;
        }
        else
        {
            updateFeatureList(target_feature_list_half_, debug_log_distance_half_, feature_list_half_threshold_);
        }
    }

    // ------------------------ Debug Log ------------------------
    debug_log_list_full_ = target_feature_list_full_.size();
    debug_log_list_half_ = target_feature_list_half_.size();
    // ------------------------ Debug Log ------------------------

    // 若超出list_limit，删除最早的一个
    if (static_cast<int>(target_feature_list_full_.size()) > list_limit_)
    {
        target_feature_list_full_.erase(target_feature_list_full_.begin());
    }
    if (static_cast<int>(target_feature_list_half_.size()) > list_limit_)
    {
        target_feature_list_half_.erase(target_feature_list_half_.begin());
    }
};

/**
 * @brief 计算两个特征之间的欧氏距离
 *
 * @param feature1 特征向量1
 * @param feature2 特征向量2
 * @return double 欧氏距离
 */
double PoseFeatureNet::calculateDistance(const std::vector<double> feature1, const std::vector<double> feature2)
{
    if (feature1.size() != feature2.size())
    {
        return 999.0; // 返回错误值或抛出异常
    }

    double distance = 0.0;
    for (size_t i = 0; i < feature1.size(); ++i)
    {
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
std::vector<double> PoseFeatureNet::calculateMinimalDistances(const std::vector<std::vector<double>> feature_list1, const std::vector<std::vector<double>> feature_list2)
{
    std::vector<double> distances;
    double distance = 0.0;
    for (size_t i = 0; i < feature_list2.size(); i++)
    {
        distances.push_back(999);
        for (size_t k = 0; k < feature_list1.size(); k++)
        {
            distance = calculateDistance(feature_list2[i], feature_list1[k]);
            if (distance < distances[i])
            {
                distances[i] = distance;
            }
        }
    }
    return distances;
}

YOLODetectionResult paddingRect(YOLODetectionResult target_coord, double screen_width, double screen_height)
{
    YOLODetectionResult padding_result = target_coord;
    double x1 = padding_result.x;
    double y1 = padding_result.y;
    double x2 = padding_result.x + padding_result.width;
    double y2 = padding_result.y + padding_result.height;
    double x = padding_result.x + 0.5 * padding_result.width;
    double y = padding_result.y + 0.5 * padding_result.height;

    if (padding_result.height < 2 * padding_result.width)
    {
        y1 = std::max(0.0, y - padding_result.width);
        y2 = std::min(screen_height, y + padding_result.width);
    }
    else
    {
        x1 = std::max(0.0, x - padding_result.height / 4);
        x2 = std::min(screen_width, x + padding_result.height / 4);
    }

    padding_result.x = x1;
    padding_result.y = y1;
    padding_result.width = x2 - x1;
    padding_result.height = y2 - y1;

    return padding_result;
}

bool yoloOverLap(const std::vector<YOLODetectionResult> &yolo_detection_results, int index)
{
    for (size_t i = 0; i < yolo_detection_results.size(); i++)
    {
        if (static_cast<int>(i) == index)
        {
            continue;
        }
        int x_overlap = std::max(0.0f,
                                 std::min(yolo_detection_results[index].x + yolo_detection_results[index].width, yolo_detection_results[i].x + yolo_detection_results[i].width) -
                                     std::max(yolo_detection_results[index].x, yolo_detection_results[i].x));
        if (x_overlap != 0.0)
        {
            int y_overlap = std::max(0.0f,
                                     std::min(yolo_detection_results[index].y + yolo_detection_results[index].height, yolo_detection_results[i].y + yolo_detection_results[i].height) -
                                         std::max(yolo_detection_results[index].y, yolo_detection_results[i].y));

            int intersection_area = x_overlap * y_overlap;
            int target_area = yolo_detection_results[index].width * yolo_detection_results[index].height;

            // 如果重叠部分面积大于目标框的五分之一，则认为重叠
            if (intersection_area > target_area / 5)
            {
                return true;
            }
        }
    }
    return false;
}

bool poseOverLap(const std::vector<Box> &pose_boxes)
{
    if (pose_boxes.empty())
    {
        return true; // 如果为空则认为重叠了，让外层策略做验证
    }

    int index = 0;
    for (size_t i = 0; i < pose_boxes.size(); i++)
    {
        if (areaBox(pose_boxes[i]) > areaBox(pose_boxes[index]))
        {
            index = static_cast<int>(i);
        }
    }

    for (size_t i = 0; i < pose_boxes.size(); i++)
    {
        if (static_cast<int>(i) == index)
        {
            continue;
        }
        // 计算水平重叠
        int x_overlap = std::max(0.0f, std::min(pose_boxes[index].right, pose_boxes[i].right) - std::max(pose_boxes[index].left, pose_boxes[i].left));
        if (x_overlap > 0)
        {
            // 计算垂直重叠
            int y_overlap = std::max(0.0f, std::min(pose_boxes[index].bottom, pose_boxes[i].bottom) - std::max(pose_boxes[index].top, pose_boxes[i].top));
            // 如果水平和垂直方向上都有重叠，则认为有重叠
            if (y_overlap > 0)
            {
                return true;
            }
        }
    }
    return false;
}

bool PoseFeatureNet::isOverLap(const std::vector<YOLODetectionResult> &yolo_detection_results, int index, const std::vector<Box> &pose_boxes)
{
    if (yoloOverLap(yolo_detection_results, index) == false)
    {
        return poseOverLap(pose_boxes);
    }
    return true;
}

bool isTargetInFront(int index, const std::vector<YOLODetectionResult> &body_detection_results)
{
    float target_area = body_detection_results[index].width * body_detection_results[index].height;
    float overlap_threshold = target_area / 5.0f;

    for (size_t i = 0; i < body_detection_results.size(); i++)
    {
        if (static_cast<int>(i) == index)
        {
            continue;
        }

        float x_overlap = std::max(0.0f,
                                   std::min(body_detection_results[index].x + body_detection_results[index].width, body_detection_results[i].x + body_detection_results[i].width) -
                                       std::max(body_detection_results[index].x, body_detection_results[i].x));
        float y_overlap = std::max(0.0f,
                                   std::min(body_detection_results[index].y + body_detection_results[index].height, body_detection_results[i].y + body_detection_results[i].height) -
                                       std::max(body_detection_results[index].y, body_detection_results[i].y));

        float overlap_area = x_overlap * y_overlap;

        if (overlap_area > overlap_threshold && body_detection_results[index].y + body_detection_results[index].height < body_detection_results[i].y + body_detection_results[i].height)
        {
            return false; // body_detection_results[index] 不在前面
        }
    }
    return true; // body_detection_results[index] 在所有重叠对象中都在前面
}

}