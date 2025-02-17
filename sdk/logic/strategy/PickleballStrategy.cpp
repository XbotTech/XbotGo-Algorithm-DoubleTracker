#include "PickleballStrategy.h"
#include "XbotgoControlCallback.h"
#include <vector>

namespace XbotgoSDK
{
    extern XbotgoControlCallback* g_xbotgoControlCallback;

    void PickleballStrategy::setAngleInfo(double maxAngle, double pixelPerAngle, int maxTurnDownTime) {
        _maxXAngle = maxAngle;
        _pixelXPerAngle = pixelPerAngle;
        _maxTurnDownTime = maxTurnDownTime;
    }

    int PickleballStrategy::track(const std::vector<YOLODetectionResult>& detectResult, double currentXAngle, int unused_flag) {

        if (_isAngleSet == false) {
            _originalXAngle = -currentXAngle;

            _noPersonDetectTime = 0;

            _isAngleSet = true;
        }

        switch (type)
        {
        case Single:
            singleTrack(detectResult, -currentXAngle);
            break;
        case Double:
            doubleTrack(detectResult, -currentXAngle);
            break;
        default:
            break;
        }

        return 0;
    }

    void PickleballStrategy::singleTrack(const std::vector<YOLODetectionResult>& detectResult, double currentXAngle) {

        std::vector<YOLODetectionResult> filterPersons = filterFeetUnder(detectResult, deviceVideoBufferHeight * 0.55);

        double targetX = deviceVideoBufferWidth/2.0;
        double targetY = deviceVideoBufferHeight/2.0;

        if (_turnDownTime < _maxTurnDownTime) {
            targetY = deviceVideoBufferHeight * 0.7;
            _turnDownTime++;
        }

        double xOffset = currentXAngle - _originalXAngle;

        if (filterPersons.size() == 0) {
            // 如果没有人通过筛选
            // 判断是否有偏移，且是否大于多少帧没有检测到人，如果有，就回中。否则不动

            _noPersonDetectTime += 1;

            if(_noPersonDetectTime > _maxNoPersonDetectTime) {
                // TODO: 这里是不是不应该写0？写比如10，或者20，留一些空间
                if (xOffset < -(_maxXAngle/2)) {
                    targetX = deviceVideoBufferWidth*0.5 + deviceVideoBufferWidth*0.1;
                } else if (xOffset > (_maxXAngle/2)) {
                    targetX = deviceVideoBufferWidth*0.5 - deviceVideoBufferWidth*0.1;
                }
            }

        }  else {

            _noPersonDetectTime = 0;

            std::vector<YOLODetectionResult> sortedByCenterPersons = sortByCenter(filterPersons, xOffset * _pixelXPerAngle, deviceVideoBufferWidth);

            YOLODetectionResult targetPerson = sortedByCenterPersons[0];

            if (targetPerson.x < deviceVideoBufferWidth * 0.2) {

                if (xOffset > -_maxXAngle) {
                    // 如果目标在左边缘，判断是否已经到达左侧最大角度，如果还没，就向左转动
                    targetX = deviceVideoBufferWidth*0.5 - deviceVideoBufferWidth*0.2;
                } 

            } else if (targetPerson.x + targetPerson.width > deviceVideoBufferWidth * 0.8) {

                if (xOffset < _maxXAngle) {
                    // 如果目标在右边缘，判断是否已经到达右侧最大角度，如果还没，就向右移动
                    targetX = deviceVideoBufferWidth*0.5 + deviceVideoBufferWidth*0.2;
                }

            } else if (targetPerson.x > deviceVideoBufferWidth * 0.3 && (targetPerson.x + targetPerson.width) < deviceVideoBufferWidth * 0.7) {
                // 留一点buffer，不要那么急着转回到

                 if (xOffset < -(_maxXAngle/2)) {
                     targetX = deviceVideoBufferWidth*0.5 + deviceVideoBufferWidth*0.1;
                 } else if (xOffset > (_maxXAngle/2)) {
                     targetX = deviceVideoBufferWidth*0.5 - deviceVideoBufferWidth*0.1;
                 }
            }
        }

        g_xbotgoControlCallback->onSetTarget(targetX, targetY);
    }


    void PickleballStrategy::doubleTrack(const std::vector<YOLODetectionResult>& detectResult, double currentXAngle) {

        std::vector<YOLODetectionResult> filterPersons = filterFeetUnder(detectResult, deviceVideoBufferHeight * 0.55);

        double targetX = deviceVideoBufferWidth/2.0;
        double targetY = deviceVideoBufferHeight/2.0;

        if (_turnDownTime < _maxTurnDownTime) {
            targetY = deviceVideoBufferHeight * 0.7;
            _turnDownTime++;
        }

        double xOffset = currentXAngle - _originalXAngle;

        if (filterPersons.size() == 0) {
            // 如果没有人通过筛选
            // 判断是否有偏移，且是否大于多少帧没有检测到人，如果有，就回中。否则不动

            _noPersonDetectTime += 1;

            if(_noPersonDetectTime > _maxNoPersonDetectTime) {
                // TODO: 这里是不是不应该写0？写比如10，或者20，留一些空间
                if (xOffset < -(_maxXAngle/2)) {
                    targetX = deviceVideoBufferWidth*0.5 + deviceVideoBufferWidth*0.1;
                } else if (xOffset > (_maxXAngle/2)) {
                    targetX = deviceVideoBufferWidth*0.5 - deviceVideoBufferWidth*0.1;
                }
            }

            // TODO: 是否需要考虑一个人的情况？

        } else if (filterPersons.size() >=2) {

            _noPersonDetectTime = 0;

            std::vector<YOLODetectionResult> sortedByCenterPersons = sortByCenter(filterPersons, xOffset * _pixelXPerAngle, deviceVideoBufferWidth);

            YOLODetectionResult targetPerson1 = sortedByCenterPersons[0];
            YOLODetectionResult targetPerson2 = sortedByCenterPersons[1];

            double targetPerson1CenterX = targetPerson1.x + targetPerson1.width/2.0;
            double targetPerson2CenterX = targetPerson2.x + targetPerson2.width/2.0;
            double centerX = (targetPerson1CenterX + targetPerson2CenterX)/2.0;

            if (centerX < deviceVideoBufferWidth * 0.4) {

                if (xOffset > -_maxXAngle) {

                    // 如果目标在左边缘，判断是否已经到达左侧最大角度，如果还没，就向左转动
                    targetX = deviceVideoBufferWidth*0.5 - deviceVideoBufferWidth*0.1;
                }

            } else if (centerX > deviceVideoBufferWidth * 0.6) {

                if (xOffset < _maxXAngle) {
                    // 如果目标在右边缘，判断是否已经到达右侧最大角度，如果还没，就向右移动
                    targetX = deviceVideoBufferWidth*0.5 + deviceVideoBufferWidth*0.1;
                }

            } else if (centerX > deviceVideoBufferWidth * 0.45 && centerX < deviceVideoBufferWidth * 0.55) {
                // 留一点buffer，不要那么急着转回到

                 if (xOffset < -(_maxXAngle/2)) {
                     targetX = deviceVideoBufferWidth*0.5 + deviceVideoBufferWidth*0.1;
                 } else if (xOffset > (_maxXAngle/2)) {
                     targetX = deviceVideoBufferWidth*0.5 - deviceVideoBufferWidth*0.1;
                 }
            }
        }

        g_xbotgoControlCallback->onSetTarget(targetX, targetY);
    }

    std::vector<YOLODetectionResult> PickleballStrategy::filterFeetUnder(const std::vector<YOLODetectionResult>& allDetectResults, float y) {
        std::vector<YOLODetectionResult> filtered;
        std::copy_if(allDetectResults.begin(), allDetectResults.end(), std::back_inserter(filtered), [y](const YOLODetectionResult& result) {
            return (result.y + result.height) > y;
        });
        return filtered;
    }

    std::vector<YOLODetectionResult> PickleballStrategy::sortByClose(std::vector<YOLODetectionResult> allDetectResults) {
        std::sort(allDetectResults.begin(), allDetectResults.end(), [](const YOLODetectionResult& a, const YOLODetectionResult& b) {

            double aMaxY = a.y + a.height;
            double bMaxY = b.y + b.height;

            return aMaxY > bMaxY;
        });
        return allDetectResults;
    }

    std::vector<YOLODetectionResult> PickleballStrategy::sortByCenter(std::vector<YOLODetectionResult> allDectionResults, double xOffset, double deviceVideoBufferWidth) {

        std::sort(allDectionResults.begin(), allDectionResults.end(), [xOffset, deviceVideoBufferWidth](const YOLODetectionResult& a, const YOLODetectionResult& b) {

            double aCenter = a.x + a.width/2.0;
            double bCenter = b.x + b.width/2.0;

            double aCenterOffset = aCenter + xOffset;
            double bCenterOffset = bCenter + xOffset;

            double aOffset = aCenterOffset - deviceVideoBufferWidth/2.0;
            double bOffset = bCenterOffset - deviceVideoBufferWidth/2.0;

            return  abs(aOffset) < abs(bOffset);
        });
        return allDectionResults;
    }

    std::tuple<double, double> PickleballStrategy::getTheCenterOfTwoDetectionResult(YOLODetectionResult firstDetectionResult, YOLODetectionResult secondDetectionResult) {

        double firstCenterX = firstDetectionResult.x + firstDetectionResult.width/2.0;
        double secondCenterX = secondDetectionResult.x + secondDetectionResult.width/2.0;

        double firstCenterY = firstDetectionResult.y + firstDetectionResult.height/2.0;
        double secondCenterY = secondDetectionResult.y + secondDetectionResult.height/2.0;

        double centerX = (firstCenterX + secondCenterX)/2.0;
        double centerY = (firstCenterY + secondCenterY)/2.0;

        return std::tuple<double, double>(centerX, centerY);

    }
}
