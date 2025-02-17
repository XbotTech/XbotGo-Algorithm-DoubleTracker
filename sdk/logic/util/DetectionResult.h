#pragma once
#include <iostream>
#include <string>
#include <vector>

/**
 * @brief YOLO目标检测结果结构体
 */
struct YOLODetectionResult
{
    YOLODetectionResult(){}
    YOLODetectionResult(int lable, float prob, float x, float y, float w, float h):
        lable(lable),prob(prob), x(x),y(y), width(w),height(h){}
    int lable; //0:人 1:球 2:球筐
    float prob;
    float x;
    float y;
    float width;
    float height;
};

struct DetectionResult {
    YOLODetectionResult personResult;
    float xOffset;
    float yOffset;

    DetectionResult(YOLODetectionResult personResult, float xOffset, float yOffset)
         : personResult(personResult), xOffset(xOffset), yOffset(yOffset) {}
    virtual ~DetectionResult() = default; // 声明虚析构函数以使其成为多态基类
    
    bool operator==(const DetectionResult& other) const {
        return personResult.prob == other.personResult.prob &&
               personResult.x == other.personResult.x &&
               personResult.y == other.personResult.y &&
               personResult.width == other.personResult.width &&
               personResult.height == other.personResult.height &&
               xOffset == other.xOffset &&
               yOffset == other.yOffset;
    }

};

struct JerseyNumberDetectionResult: DetectionResult {
    std::string jerseyNumber;

    JerseyNumberDetectionResult(YOLODetectionResult personResult, float xOffset, float yOffset, const std::string& jerseyNumber)
     : DetectionResult(personResult, xOffset, yOffset), jerseyNumber(jerseyNumber) {}

    bool operator==(const JerseyNumberDetectionResult& other) const {
        return DetectionResult::operator==(other) && jerseyNumber == other.jerseyNumber;
    }

};

struct NewPoseDetectionResult: DetectionResult {
    int matchedIndex = -1;

    NewPoseDetectionResult(YOLODetectionResult personResult, float xOffset, float yOffset)
     : DetectionResult(personResult, xOffset, yOffset){}

    bool operator==(const NewPoseDetectionResult& other) const {
        return DetectionResult::operator==(other) && matchedIndex == other.matchedIndex;
    }

};

struct HeadAndBodyIndex
{
    int head_index;
    int body_index;
};