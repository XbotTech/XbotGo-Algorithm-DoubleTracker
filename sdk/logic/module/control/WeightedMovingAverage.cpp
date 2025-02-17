#include "WeightedMovingAverage.h"

namespace XbotgoSDK
{

double WeightedMovingAverage::update(double new_value)
{
    // 维持窗口大小和权重相同
    if (window.size() == weights.size()) {
        window.erase(window.begin()); // 移除窗口中的第一个元素
    }
    window.push_back(new_value); // 添加新的值

    // 计算加权和
    double weighted_sum = 0.0;
    for (std::size_t i = 0; i < window.size(); ++i) {
        weighted_sum += weights[i] * window[i];
    }

    // 计算总权重（只计算当前窗口大小的权重）
    double total_weight = std::accumulate(weights.begin(), weights.begin() + window.size(), 0.0);

    return weighted_sum / total_weight; // 返回加权平均值
}

}