#include <fstream>
#include <iostream>

#include "Global.h"
#include "XbotgoStrategy.h"
#include "XbotgoStrategyCallback.h"
#include "json.hpp"

using namespace std;
using namespace XbotgoSDK;
using json = nlohmann::json;

class MyXbotgoStrategyCallback : public XbotgoStrategyCallback
{
public:
    void onTrackingPoint(TrackResult point) override
    {
        std::cout << "XbotgoStrategyCallback onTrackingPoint x: " << point.x << " y: " << point.y << " directon:" << point.direction << std::endl;
    }
    void onTrackingZoom(float scaleFactor) override
    {
        std::cout << "XbotgoStrategyCallback onTrackingZoom:" << scaleFactor << std::endl;
    }
    virtual void onControlSpeed(int yawSpeed, int pitchSpeed) override
    {
        std::cout << "XbotgoStrategyCallback onControlSpeed" << std::endl;
    }
};

struct YOLODetectionResultExt {
    std::vector<YOLODetectionResult> rst;
    double time;
    float yaw;
    float pitch;
};

std::vector<YOLODetectionResultExt> loadJson(string filepath)
{
    // 从文件中读取 JSON 数据
    std::ifstream file(filepath);
    json j;
    file >> j;
    std::vector<YOLODetectionResultExt> totalYoloFrames;
    for (const auto& element : j) {
        YOLODetectionResultExt yolo_detection_results_ext;
        try {
            yolo_detection_results_ext.time = element["time"];
            yolo_detection_results_ext.yaw = element["currentYawAngle"];
            yolo_detection_results_ext.pitch = element["currentPitchAngle"];

            for (const auto& bufferResult : element["yolo"]) {
                YOLODetectionResult r;

                r.lable = bufferResult["label"];
                r.x = bufferResult["x"];
                r.y = bufferResult["y"];
                r.prob = bufferResult["confidence"];
                r.width = bufferResult["width"];
                r.height = bufferResult["height"];
                yolo_detection_results_ext.rst.push_back(r);
            }
        } catch (...) {
            std::cout << "error in reading json" << endl;
            continue;
        }
        totalYoloFrames.push_back(yolo_detection_results_ext);
    }
    return totalYoloFrames;
}

int main(int argc, char** argv)
{
    std::cout << "<<< xbotgo sdk >>>" << std::endl;
    std::vector<YOLODetectionResultExt> totalYoloFrames = loadJson("../docs/IceHockeyTrack.json");
    std::cout << totalYoloFrames.size() << std::endl;

    // 设置是否开启log打印
    Xbotgo_Set_Printf_Enabled(true);
    // 选择并注册策略
    int enableLog = 1;
    InitStrategyType type = InitStrategyType::INIT_STRATEGY_TYPE_FOLLOWMEPOSE;
    Xbotgo_Strategy_init(type, "./logs/", enableLog);
    // 注册回调
    XbotgoStrategyCallback* callback = new MyXbotgoStrategyCallback();
    Xbotgo_Strategy_registerCallbacks(callback);
    // 设置初始参数
    Xbotgo_Strategy_setVideoBufferSize(1920, 1080);        // 设置拍摄视频分辨率
    Xbotgo_Strategy_device_setVideoBufferSize(2560, 1440); // 设置AI检测视频分辨率
    Xbotgo_Strategy_set_yawLimitAngle(130);                // 设置yaw轴极限角
    Xbotgo_Strategy_setZoomParam(1.0, 1.4);                // 设置变焦范围
    Xbotgo_Strategy_set_pitch_initialAngle(10);            // 设置pitch轴初始角度
    // 激活策略
    Xbotgo_Strategy_activate();

    for (size_t i = 0; i < totalYoloFrames.size(); i++) {
        YOLODetectionResultExt yolo_detection_results_ext = totalYoloFrames[i];
        std::vector<YOLODetectionResult> yolo_detection_results = yolo_detection_results_ext.rst;
        // 发送陀螺仪数据
        Xbotgo_Strategy_setGyroscope(yolo_detection_results_ext.yaw, yolo_detection_results_ext.pitch);
        // 发送AI检测结果并进行跟踪
        Xbotgo_Strategy_track_for_test(yolo_detection_results, yolo_detection_results_ext.time);
        // usleep(300000);
    }

    // 关闭策略
    Xbotgo_Strategy_deactivate();
    // 删除策略，释放资源
    Xbotgo_Strategy_uninit();
    return 0;
}
