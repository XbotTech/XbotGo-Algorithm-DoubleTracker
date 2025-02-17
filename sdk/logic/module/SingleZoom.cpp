#include "SingleZoom.h"
#include <cmath>

double easeInOut(double x)
{
    if (x < 0.5 && x > 0)
    {
        return 4 * x * x;
    }
    else
    {
        return 1;
    }
}

SingleZoom::SingleZoom(double width, double height, double min_zoom, double max_zoom, double zoom_in_proportion, double zoom_out_proportion, double target_zoom_factory)
{
    screen_width_ = width;
    screen_height_ = height;
    min_zoom_value_ = min_zoom;
    max_zoom_value_ = max_zoom;
    zoom_in_proportion_ = zoom_in_proportion;
    zoom_out_proportion_ = zoom_out_proportion;
    target_height_ = height * target_zoom_factory;
    last_coord_ = std::make_pair(0.0, 0.0);
}

ZoomResult SingleZoom::zoom(const int &index, const std::vector<RectBox> &relative_coords, const double &current_zoom_value)
{
    double proportion;
    proportion = relative_coords[index].height / screen_height_;

    if (proportion > zoom_out_proportion_) // 0.6
    {
        // zoom out
        zoom_result_.zoom_value = std::max((target_height_ / relative_coords[index].height) * current_zoom_value, min_zoom_value_);
        zoom_result_.zoom_rate = easeInOut(std::abs(zoom_result_.zoom_value - current_zoom_value));
        zoom_result_.zoom_time = zoom_increment_ / zoom_result_.zoom_rate;
    }
    else if (proportion < zoom_in_proportion_) // 0.4
    {
        // zoom in
        zoom_result_.zoom_value = std::min((target_height_ / relative_coords[index].height) * current_zoom_value, max_zoom_value_);
        zoom_result_.zoom_rate = easeInOut(std::abs(zoom_result_.zoom_value - current_zoom_value));
        zoom_result_.zoom_time = zoom_increment_ / zoom_result_.zoom_rate;
    }
    else
    {
        // no zoom
        zoom_result_.zoom_value = 0;
        zoom_result_.zoom_rate = 0;
        zoom_result_.zoom_time = 0;
    }

    std::cout << "SingleZoom::zoom proportion="<<proportion<<std::endl;
    std::cout << "SingleZoom::zoom current_zoom_value="<<current_zoom_value<<std::endl;
    std::cout << "SingleZoom::zoom relative_coords[index].height="<<relative_coords[index].height<<std::endl;
    std::cout << "SingleZoom::zoom zoom_value="<<zoom_result_.zoom_value<<std::endl;
    std::cout << "SingleZoom::zoom zoom_rate="<<zoom_result_.zoom_rate<<std::endl;
    std::cout << "SingleZoom::zoom sub="<<zoom_result_.zoom_value - current_zoom_value<<std::endl;

    writeMessage(zoom_result_.zoom_value, proportion);

    return zoom_result_;
}

ZoomResult SingleZoom::zoom(const int &index, const std::vector<YOLODetectionResult> &yolo_detection_results, const double &current_zoom_value)
{
    double proportion;
    proportion = yolo_detection_results[index].height / screen_height_;

    if (proportion > zoom_out_proportion_)
    {
        // zoom out
        zoom_result_.zoom_value = std::max((target_height_ / yolo_detection_results[index].height) * current_zoom_value, min_zoom_value_);
        zoom_result_.zoom_rate = easeInOut(std::abs(zoom_result_.zoom_value - current_zoom_value));
        zoom_result_.zoom_time = zoom_increment_ / zoom_result_.zoom_rate;
    }
    else if (proportion < zoom_in_proportion_)
    {
        // zoom in
        zoom_result_.zoom_value = std::min((target_height_ / yolo_detection_results[index].height) * current_zoom_value, max_zoom_value_);
        zoom_result_.zoom_rate = easeInOut(std::abs(zoom_result_.zoom_value - current_zoom_value));
        zoom_result_.zoom_time = zoom_increment_ / zoom_result_.zoom_rate;
    }
    else
    {
        // no zoom
        zoom_result_.zoom_value = 0;
        zoom_result_.zoom_rate = 0;
        zoom_result_.zoom_time = 0;
    }
    writeMessage(zoom_result_.zoom_value, proportion);

    return zoom_result_;
}

ZoomResult SingleZoom::zoomOut(const double &current_zoom_value)
{
    zoom_result_.zoom_value = std::max(current_zoom_value - zoom_out_increment_, min_zoom_value_);
    zoom_result_.zoom_rate = easeInOut(zoom_out_increment_);
    zoom_result_.zoom_time = zoom_increment_ / zoom_result_.zoom_rate;
    writeMessage(zoom_result_.zoom_value, 0.0);
    return zoom_result_;
}

void SingleZoom::writeMessage(double zoom_value, double proportion)
{
    // debug_log_manager_->setZoomValue(zoom_value);
    // debug_log_manager_->setProportion(proportion);
    // debug_log_manager_->uploadZoomLogs();
}