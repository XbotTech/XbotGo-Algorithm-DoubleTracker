#include "PosePostProcess.h"

namespace XbotgoSDK
{
    float areaBox(const Box &box)
    {
        return (box.right - box.left) * (box.bottom - box.top);
    }

    float iou(const Box &box1, const Box &box2)
    {
        float left = std::max(box1.left, box2.left);
        float top = std::max(box1.top, box2.top);
        float right = std::min(box1.right, box2.right);
        float bottom = std::min(box1.bottom, box2.bottom);

        float cross = std::max(0.0f, right - left) * std::max(0.0f, bottom - top);
        float unionArea = areaBox(box1) + areaBox(box2) - cross;

        if (cross == 0 || unionArea == 0)
            return 0.0f;
        return cross / unionArea;
    }

    std::vector<Box> NMS(std::vector<Box> &boxes, float iou_thres)
    {
        std::vector<bool> remove_flags(boxes.size(), false);
        std::vector<Box> keep_boxes;

        for (size_t i = 0; i < boxes.size(); ++i)
        {
            if (remove_flags[i])
                continue;

            keep_boxes.push_back(boxes[i]);
            for (size_t j = i + 1; j < boxes.size(); ++j)
            {
                if (remove_flags[j])
                    continue;

                if (iou(boxes[i], boxes[j]) > iou_thres)
                {
                    remove_flags[j] = true;
                }
            }
        }
        return keep_boxes;
    }

    std::vector<Box> postProcess(const std::vector<std::vector<float>> &pred, const float &conf_thres, const float &iou_thres)
    {
        std::vector<Box> boxes;

        for (const auto &item : pred)
        {
            if (item[4] <= conf_thres)
                continue;

            float cx = item[0];
            float cy = item[1];
            float w = item[2];
            float h = item[3];
            float conf = item[4];

            float left = std::max(0.0f, cx - w * 0.5f);
            float top = std::max(0.0f, cy - h * 0.5f);
            float right = std::max(0.0f, cx + w * 0.5f);
            float bottom = std::max(0.0f, cy + h * 0.5f);

            // left *= 128;
            // right *= 128;
            // top *= 256;
            // bottom *= 256;

            std::vector<float> keypoints(item.begin() + 5, item.end());

            boxes.push_back({left, top, right, bottom, conf, keypoints});
        }

        sort(boxes.begin(), boxes.end(), [](const Box &a, const Box &b)
             { return a.conf > b.conf; });

        return NMS(boxes, iou_thres);
    }

    std::tuple<int, std::array<float, 4>, std::array<float, 4>> halfOrFull(std::array<float, 4> rect, std::vector<float> points)
    {

        // 铺平的数组转成17*3
        std::vector<std::array<float, 3>> keypoints;
        for (size_t i = 0; i < points.size(); i += 3)
        {
            std::array<float, 3> kp = {points[i], points[i + 1], points[i + 2]};
            keypoints.push_back(kp);
        }

        // 判断骨骼点是否在框内，若不在，则忽略
        float left = rect[0], top = rect[1], right = rect[2], bottom = rect[3];

        for (int i = 0; i < 17; i++)
        {
            float x = keypoints[i][0], y = keypoints[i][1];

            // 判断点是否在框外
            if ((x < left || x > right) || (y < top || y > bottom))
            {
                keypoints[i][2] = 0; // 将置信度置0，忽略它
            }
        }

        // left-ankle和right-ankle有一个存在在框内即判定为全身，全身框返回完整框（0，0），（128，256）。半身框返回腰点位置。
        // if (keypoints[15][2] > 0.5 || keypoints[16][2] > 0.5)
        // {
        //     // x1, y1, x2, y2
        //     std::array<float, 4> full_rect = {0, 0, 1, 1};
        //     // 腰点的位置
        //     std::array<float, 4> half_rect = {left / 128, top / 256, right / 128, std::max(keypoints[11][1], keypoints[12][1]) / 256};
        //     return {pose_full_, full_rect, half_rect};
        // }

        // left-knee和right-knee有一个在框内，判定为全身，全身框返回rect位置，半身框返回腰点位置
        if (keypoints[13][2] > 0.5 || keypoints[14][2] > 0.5)
        {
            std::array<float, 4> full_rect = {left / 128, top / 256, right / 128, bottom / 256};
            std::array<float, 4> half_rect = {left / 128, top / 256, right / 128, std::max(keypoints[11][1], keypoints[12][1]) / 256}; // 腰点的位置
            return {_pose_full_, full_rect, half_rect};
        }

        // left-hip和right-hip有一个在框内，判定为半身，全身框返回0，半身框返回腰点位置
        if (keypoints[11][2] > 0.5 || keypoints[12][2] > 0.5)
        {
            std::array<float, 4> full_rect = {0, 0, 0, 0};
            std::array<float, 4> half_rect = {left / 128, top / 256, right / 128, std::max(keypoints[11][1], keypoints[12][1]) / 256};
            return {_pose_half_, full_rect, half_rect};
        }

        // left-hip和right-hip有一个在框内，判定为半身，全身框返回0，半身框返回rect位置
        if (keypoints[5][2] > 0.5 || keypoints[6][2] > 0.5)
        {
            std::array<float, 4> full_rect = {0, 0, 0, 0};
            std::array<float, 4> half_rect = {left / 128, top / 256, right / 128, bottom / 256};
            return {_pose_shoulder_, full_rect, half_rect};
        }

        return {_pose_none_, {0, 0, 0, 0}, {0, 0, 0, 0}};
    }

    std::tuple<int, std::array<float, 4>, std::array<float, 4>> analysisPose(YOLODetectionResult target_body_coord, YOLODetectionResult padding_rect, std::vector<Box> postProcess_result)
    {
        // 判断postProcess_result是否为空
        if (postProcess_result.empty())
        {
            return std::make_tuple(_pose_none_, std::array<float, 4>{0, 0, 0, 0}, std::array<float, 4>{0, 0, 0, 0});
        }

        std::vector<YOLODetectionResult> postProcess_yolo_result;
        for (auto &box : postProcess_result)
        {
            YOLODetectionResult yolo_result = {0, 1, padding_rect.x + ((box.left / 128) * padding_rect.width),
                                               padding_rect.y + ((box.top / 256) * padding_rect.height),
                                               padding_rect.width * ((box.right / 128) - (box.left / 128)),
                                               padding_rect.height * ((box.bottom / 256) - (box.top / 256))};
            postProcess_yolo_result.push_back(yolo_result);
        }

        // 在postProcess_yolo_result中找到和target_body_coord重叠面积最大的框
        double max_overlap_area = 0;
        int best_match_index = -1;

        for (size_t i = 0; i < postProcess_yolo_result.size(); i++)
        {
            // 计算重叠区域的坐标
            double overlap_x1 = std::max(target_body_coord.x, postProcess_yolo_result[i].x);
            double overlap_y1 = std::max(target_body_coord.y, postProcess_yolo_result[i].y);
            double overlap_x2 = std::min(target_body_coord.x + target_body_coord.width, postProcess_yolo_result[i].x + postProcess_yolo_result[i].width);
            double overlap_y2 = std::min(target_body_coord.y + target_body_coord.height, postProcess_yolo_result[i].y + postProcess_yolo_result[i].height);

            // 计算重叠区域的宽度和高度
            double overlap_width = std::max(0.0, overlap_x2 - overlap_x1);
            double overlap_height = std::max(0.0, overlap_y2 - overlap_y1);

            // 计算重叠面积
            double overlap_area = overlap_width * overlap_height;

            // 更新最大重叠面积和最佳匹配索引
            if (overlap_area > max_overlap_area)
            {
                max_overlap_area = overlap_area;
                best_match_index = static_cast<int>(i);
            }
        }

        if (best_match_index == -1)
        {
            return std::make_tuple(_pose_none_, std::array<float, 4>{0, 0, 0, 0}, std::array<float, 4>{0, 0, 0, 0});
        }

        Box target_box = postProcess_result[best_match_index];

        std::array<float, 4> target_rect = {target_box.left, target_box.top, target_box.right, target_box.bottom};
        std::vector<float> target_points = target_box.keypoints;
        return halfOrFull(target_rect, target_points);
    }
}