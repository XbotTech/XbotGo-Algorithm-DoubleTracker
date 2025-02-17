#ifndef CHAMCALIBRATEXY_H
#define CHAMCALIBRATEXY_H

#include "XbotgoStrategy.h"
#include <cmath>
#include <vector>

namespace XbotgoSDK
{

class ChamCalibrateXY
{
public:
    ChamCalibrateXY(int img_width = 2560, int img_height = 1440);

    YOLODetectionResult real2virXY(const YOLODetectionResult& detection, float angle_yaw, float angle_pitch);
    YOLODetectionResult vir2realXY(const YOLODetectionResult& detection, float angle_yaw, float angle_pitch);
    TrackPoint real2virXY(float real_x, float real_y, float angle_yaw, float angle_pitch);
    TrackPoint vir2realXY(float vir_x, float vir_y, float angle_yaw, float angle_pitch);
    std::vector<float> calibrateXY(float real_x, float real_y);

private:
    float center_x;
    float center_y;
    float dm;
    float fm;
    std::vector<float> Y_real;
    std::vector<float> Z_ref;

    float radians(float angle_deg);
    std::vector<float> lookupDeltaXY(float delta_x, float delta_y);
    void initializeTables();
};

} // namespace goddistance

#endif // CHAMCALIBRATEXY_H
