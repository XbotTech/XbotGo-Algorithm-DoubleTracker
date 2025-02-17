#include "CoordinateTransform.h"

namespace XbotgoSDK
{

YOLODetectionResult CoordinateTransform::coordinateToVirtual(const YOLODetectionResult& detection, const Gyroscope& gyroscope ,const int bufferWidth, const int bufferHeight)
{   
    float currentYawAngle = gyroscope.yawAngle;  
    float currentPitchAngle = gyroscope.pitchAngle;  
    float xOffset = -currentYawAngle * 24.9 - bufferWidth/2;  
    float yOffset = -currentPitchAngle * 24.9 - bufferHeight/2;  
  
    YOLODetectionResult  virtualDetectionResult = {detection.lable, detection.prob, detection.x+xOffset, detection.y+yOffset, detection.width,detection.height}; 
    return virtualDetectionResult;  
}

TrackPoint CoordinateTransform::coordinateToVirtual(const TrackPoint& trackPoint, const Gyroscope& gyroscope,const int bufferWidth, const int bufferHeight)
{   
    float currentYawAngle = gyroscope.yawAngle;  
    float currentPitchAngle = gyroscope.pitchAngle;  
    float xOffset = -currentYawAngle * 24.9 - bufferWidth/2;  
    float yOffset = -currentPitchAngle * 24.9 - bufferHeight/2;  
    
    float finalPointX = trackPoint.x + xOffset;
    float finalPointY = trackPoint.y + yOffset;

    TrackPoint point = {finalPointX, finalPointY};
    return point;  
}


TrackPoint CoordinateTransform::coordinateToBuffer(const TrackPoint& trackPoint, const Gyroscope& gyroscope, const int bufferWidth, const int bufferHeight)
{   
    float x = trackPoint.x;
    float gimbalCenterPointX = gyroscope.yawAngle * 24.9;
    float gimbalCenterPointY =  gyroscope.pitchAngle * 24.9;
    float finalPointX = (x + gimbalCenterPointX) /  2560 * float(bufferWidth);

    float y = trackPoint.y;
    float finalPointY =  (y - gimbalCenterPointY) / 1440 * float(bufferHeight);

    TrackPoint point = {finalPointX, finalPointY};
    return point;
}

TrackPoint CoordinateTransform::coordinateToDeviceBuffer(const TrackPoint& trackPoint, const Gyroscope& gyroscope, const int bufferWidth, const int bufferHeight)
{
    float xOffset = gyroscope.yawAngle * 24.9 + bufferWidth/2;
    float yOffset = gyroscope.pitchAngle * 24.9 + bufferHeight/2;  
    float finalPointX = trackPoint.x + xOffset;
    float finalPointY = trackPoint.y + yOffset;

    TrackPoint point = {finalPointX, finalPointY};
    return point;
}

}