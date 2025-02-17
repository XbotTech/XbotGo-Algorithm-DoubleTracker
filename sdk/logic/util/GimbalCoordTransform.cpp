#include <iostream>

#include "GimbalCoordTransform.h"

namespace XbotgoSDK
{

GimbalCoordTransform::GimbalCoordTransform(int img_width, int img_height, float f_px_, float f_py_)
    : center_x(img_width / 2.0), center_y(img_height / 2.0), f_px(f_px_), f_py(f_py_)
{
}

float GimbalCoordTransform::radians(float angle_deg)
{
    float angle_rad = angle_deg * M_PI / 180.0;
    return angle_rad;
}

YOLODetectionResult GimbalCoordTransform::real2virXY(const YOLODetectionResult& detection, float angle_yaw, float angle_pitch)
{
    //angle_yaw = -angle_yaw;

    float delta_x = detection.x - center_x; // 带正负: 左负右正
    float delta_y = detection.y - center_y; // 带正负: 上负下正

    float delta_yaw = atan(delta_x / f_px); // 弧度
    float Ry = sqrt(delta_x * delta_x + delta_y * delta_y + f_py * f_py);
    float delta_pitch = asin(delta_y / Ry);

    float total_yaw = radians(angle_yaw) + delta_yaw; // 转换为弧度
    float total_pitch = radians(angle_pitch) + delta_pitch;

    float virtual_x = total_yaw * f_px;
    float virtual_y = total_pitch * f_py;

    YOLODetectionResult virtualDetectionResult = {detection.lable, detection.prob, virtual_x, virtual_y, detection.width, detection.height};
    return virtualDetectionResult;
}

TrackPoint GimbalCoordTransform::real2virXY(float real_x, float real_y, float angle_yaw, float angle_pitch)
{
    //angle_yaw = -angle_yaw;

    float delta_x = real_x - center_x; // 带正负: 左负右正
    float delta_y = real_y - center_y; // 带正负: 上负下正

    float delta_yaw = atan(delta_x / f_px); // 弧度
    float Ry = sqrt(delta_x * delta_x + delta_y * delta_y + f_py * f_py);
    float delta_pitch = asin(delta_y / Ry);

    float total_yaw = radians(angle_yaw) + delta_yaw; // 转换为弧度
    float total_pitch = radians(angle_pitch) + delta_pitch;

    float virtual_x = total_yaw * f_px;
    float virtual_y = total_pitch * f_py;

    TrackPoint point = {virtual_x, virtual_y};
    return point;
}

TrackPoint GimbalCoordTransform::vir2realXY(float vir_x, float vir_y, float angle_yaw, float angle_pitch)
{
    //angle_yaw = -angle_yaw;

    float delta_yaw_rad = vir_x / f_px - radians(angle_yaw); // 转换为弧度
    float delta_pitch_rad = vir_y / f_py - radians(angle_pitch);

    float delta_x = tan(delta_yaw_rad) * f_px;
    float delta_y = sqrt(delta_x * delta_x + f_py * f_py) * tan(delta_pitch_rad);

    float real_x = center_x + delta_x;
    float real_y = center_y + delta_y;
    if (real_x <= 0)
        real_x = 0;
    if (real_y <= 0)
        real_y = 0;
    if (real_x >= center_x * 2)
        real_x = center_x * 2 - 1;
    if (real_y >= center_y * 2)
        real_y = center_y * 2 - 1;

    TrackPoint point = {real_x, real_y};
    return point;
}

YOLODetectionResult GimbalCoordTransform::vir2realXY(const YOLODetectionResult& detection, float angle_yaw, float angle_pitch)
{
    //angle_yaw = -angle_yaw;

    float delta_yaw_rad = detection.x / f_px - radians(angle_yaw); // 转换为弧度
    float delta_pitch_rad = detection.y / f_py - radians(angle_pitch);

    float delta_x = tan(delta_yaw_rad) * f_px;
    float delta_y = sqrt(delta_x * delta_x + f_py * f_py) * tan(delta_pitch_rad);

    float real_x = center_x + delta_x;
    float real_y = center_y + delta_y;
    if (real_x <= 0)
        real_x = 0;
    if (real_y <= 0)
        real_y = 0;
    if (real_x >= center_x * 2)
        real_x = center_x * 2 - 1;
    if (real_y >= center_y * 2)
        real_y = center_y * 2 - 1;

    YOLODetectionResult realDetectionResult = {detection.lable, detection.prob, real_x, real_y, detection.width, detection.height};
    return realDetectionResult;
}

}