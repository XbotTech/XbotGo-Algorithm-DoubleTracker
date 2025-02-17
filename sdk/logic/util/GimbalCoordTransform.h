#ifndef GIMBALCOORDTRANSFORM_H
#define GIMBALCOORDTRANSFORM_H

#include "XbotgoStrategy.h"
#include <cmath>
#include <vector>

namespace XbotgoSDK
{

class GimbalCoordTransform
{
public:
    GimbalCoordTransform(int img_width, int img_height, float f_px_, float f_py_);

    YOLODetectionResult real2virXY(const YOLODetectionResult& detection, float angle_yaw, float angle_pitch);
    YOLODetectionResult vir2realXY(const YOLODetectionResult& detection, float angle_yaw, float angle_pitch);
    TrackPoint real2virXY(float real_x, float real_y, float angle_yaw, float angle_pitch);
    TrackPoint vir2realXY(float vir_x, float vir_y, float angle_yaw, float angle_pitch);

private:
    float center_x;
    float center_y;
    float f_px;
    float f_py;
    std::vector<float> Y_real;
    std::vector<float> Z_ref;

    float radians(float angle_deg);
};

}

#endif // GIMBALCOORDTRANSFORM_H
