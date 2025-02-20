#define DEBUG
#include <fstream>
#include <iostream>

#include "Global.h"
#include "XbotgoStrategy.h"
#include "XbotgoSingleStrategyCallback.h"
#include "XbotgoSingleStrategy.h"
#include "DoubleFeatureNet.h"
#include "XbotgoStrategyCommon.h"
#include "json.hpp"

using namespace std;
using namespace XbotgoSDK;
using json = nlohmann::json;
// extern XbotgoSingleStrategyCallback* g_xbotgoSingleStrategyCallbacks;
// 递归终止条件：打印单个元素
template <typename T>
void printVector_(const T& elem, std::ostream& os = std::cout)
{
    os << elem;
}

// 递归函数：处理多维 vector
template <typename T>
void printVector_(const std::vector<T>& vec, std::ostream& os = std::cout)
{
    os << "[";
    bool first = true;
    for (const auto& elem : vec) {
        if (!first) {
            os << ", ";
        }
        else {
            first = false;
        }
        printVector_(elem, os); // 递归调用
    }
    os << "]";
}
template <typename T>
void printVector(const std::vector<T>& vec, const std::string str = "", std::ostream& os = std::cout)
{
    std::cout << str;
    printVector_(vec, cout);
    std::cout << std::endl;
}

class MyXbotgoSingleStrategyCallback : public XbotgoSingleStrategyCallback
{
private:
    std::vector<std::vector<double>> featureList = std::vector<std::vector<double>>(2, std::vector<double>(10));

public:
    MyXbotgoSingleStrategyCallback()
    {
        // 填充数据
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 10; ++j) {
                featureList[i][j] = static_cast<double>(i * 10 + j);
                // std::cout << featureList.at(i).at(j) << " ";
            }
            // std::cout << " | ";
        }
        // std::cout << std::endl;
    };
    virtual ~MyXbotgoSingleStrategyCallback() {};

    virtual std::vector<std::vector<double>> onGetAllFeatures() override
    {
        return featureList;
    }

    virtual std::vector<double> onGetFeature(int index) override
    {
        return featureList[index];
    }

    virtual std::vector<double> onGetHalfFeature(int index) override
    {
        return {1.0, 2.0};
    }

    virtual int onGetPose(int index) override
    {
        return 0;
    }

    virtual std::vector<double> onGetFeature(YOLODetectionResult target_coord) override
    {
        return {1.0, 2.0};
    }

    virtual std::vector<std::vector<float>> onPoseDetect(YOLODetectionResult target_coord) override
    {
        return {{1.0f, 2.0f}, {3.0f, 4.0f}};
    }

    virtual double onGetCurrentYaw() override
    {
        return 0.0;
    }

    virtual double onGetCurrentPitch() override
    {
        return 0.0;
    }

    virtual void onSetGimbal(double yaw, double pitch) override
    {
        // 假设这里有一些设置云台角度的代码
    }

    virtual double onGetZoomFactor() override
    {
        return 1.0;
    }

    virtual void onSetZoomFactor(double factor, double time, double rate) override
    {
        // 假设这里有一些设置Zoom系数的代码
    }

    virtual void onDebugFeatureNetInfo(int state, int pose, int listFull, int listHalf, double distanceFull, double distanceHalf, double distancePre) override
    {
        // 假设这里有一些打印Debug信息的代码
    }

    virtual void onDebugFollowMeInfo(int byteState, int reidState, int poseState, int listFull, int listHalf, double distanceFull, double distanceHalf, double distancePre) override
    {
        // 假设这里有一些打印Debug信息的代码
    }

    virtual void onDebugJerseyNumberTrackState(int state) override
    {
        // 假设这里有一些打印Debug信息的代码
    }
};

int main()
{
    XbotgoSingleStrategyCallback* myCallback = new MyXbotgoSingleStrategyCallback();
    g_xbotgoSingleStrategyCallbacks = myCallback;
    DoubleFeatureNet* myDoubleFeatureNet = new DoubleFeatureNet();
    cout << "----------debug myCallback----------" << endl;
    auto feature = myCallback->onGetFeature(0);
    std::cout << "onGetFeature(0): ";
    printVector(feature);

    feature = myCallback->onGetFeature(1);
    std::cout << "onGetFeature(1): ";
    printVector(feature);

    auto feature1 = myCallback->onGetAllFeatures();
    std::cout << "onGetAllFeatures(): ";
    printVector(feature1);
    cout << "----------debug myCallback----------" << endl
         << endl;

    cout << "----------debug DoubleDeatureNet::targetInit----------" << endl;

    myDoubleFeatureNet->targetInit({0, 1}, 2.1);

    printVector(myDoubleFeatureNet->target_feature_base_, "\ntarget_feature_base_: ");
    printVector(myDoubleFeatureNet->target_feature_list_full_, "target_feature_list_full_: ");
    printVector(myDoubleFeatureNet->previous_target_feature_, "previous_target_feature_: ");
    printVector(myDoubleFeatureNet->current_target_feature_, "current_target_feature_: ");
    cout
        << "DoubleFeatureNet::calculateDistance : " << myDoubleFeatureNet->calculateDistance({1, 2, 3}, {1, 2, 5}) << endl;

    cout << "\n----------debug DoubleDeatureNet::getFeature----------" << endl;
    myDoubleFeatureNet->getFeature(0, 1);
    printVector(myDoubleFeatureNet->current_target_feature_, "current_target_feature_");
    return 0;
}
