#ifndef __XBOTGO_SDK_CORE_LOGIC_COORDINATE_H__
#define __XBOTGO_SDK_CORE_LOGIC_COORDINATE_H__

#include <iostream>
#include <memory>

#include "DetectionResult.h"
#include "MOT.h"
#include "XbotgoStrategy.h"

namespace XbotgoSDK
{

/**
 * @brief  传感器数据
 *
 */
struct Gyroscope {
    float yawAngle;
    float pitchAngle;
    Gyroscope()
    {
    }
    Gyroscope(float yawAngle, float pitchAngle)
        : yawAngle(yawAngle), pitchAngle(pitchAngle)
    {
    }
};

class CoordinateTransform
{
public:
    static YOLODetectionResult coordinateToVirtual(const YOLODetectionResult& detection, const Gyroscope& gyroscope, const int bufferWidth, const int bufferHeight);
    static TrackPoint coordinateToVirtual(const TrackPoint& trackPoint, const Gyroscope& gyroscope, const int bufferWidth, const int bufferHeight);
    static TrackPoint coordinateToBuffer(const TrackPoint& trackPoint, const Gyroscope& gyroscope, const int bufferWidth, const int bufferHeight);
    static TrackPoint coordinateToDeviceBuffer(const TrackPoint& trackPoint, const Gyroscope& gyroscope, const int bufferWidth, const int bufferHeight);
};

}

#endif
