#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

#include "DetectionResult.h"

namespace XbotgoSDK
{

#define _pose_none_ -1
#define _pose_full_ 0
#define _pose_half_ 1
#define _pose_shoulder_ 2

struct Box {
    float left, top, right, bottom, conf;
    std::vector<float> keypoints;
};

float areaBox(const Box& box);

float iou(const Box& box1, const Box& box2);

std::vector<Box> NMS(std::vector<Box>& boxes, float iou_thres);

std::vector<Box> postProcess(const std::vector<std::vector<float>>& pred, const float& conf_thres = 0.25, const float& iou_thres = 0.45);

std::tuple<int, std::array<float, 4>, std::array<float, 4>> halfOrFull(std::array<float, 4> rect, std::vector<float> points);

std::tuple<int, std::array<float, 4>, std::array<float, 4>> analysisPose(YOLODetectionResult target_body_coord, YOLODetectionResult padding_rect, std::vector<Box> postProcess_result);
}