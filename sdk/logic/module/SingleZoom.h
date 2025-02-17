#pragma once
#include <iostream>
#include <vector>

#include "DetectionResult.h"

struct ZoomResult {
    double zoom_value;
    double zoom_time;
    double zoom_rate;
};

struct RectBox {
    double x;
    double y;
    double width;
    double height;
};

class SingleZoom
{
private:
    double max_zoom_value_; // 可变焦的最大值
    double min_zoom_value_; // 可变焦的最小值
    double screen_width_;   // 画面的宽
    double screen_height_;  // 画面的高

    double target_height_;            // 目标高度
    double zoom_increment_ = 0.01;    // 变焦增量
    double zoom_out_increment_ = 0.2; // 缩小增量

    double zoom_in_proportion_;
    double zoom_out_proportion_;

    ZoomResult zoom_result_;               // 变焦输出结果
    std::pair<double, double> last_coord_; // 目标上一帧坐标

public:
    SingleZoom(double width, double height, double min_zoom, double max_zoom, double zoom_in_proportion, double zoom_out_proportion, double target_zoom_factory);
    ZoomResult zoom(const int& index, const std::vector<RectBox>& relative_coords, const double& current_zoom_value);
    ZoomResult zoom(const int& index, const std::vector<YOLODetectionResult>& yolo_detection_results, const double& current_zoom_value);
    ZoomResult zoomOut(const double& current_zoom_value);

    void writeMessage(double zoom_value, double proportion);
};
