
#ifndef WEIGHTEDMOVINGAVERAGE_H
#define WEIGHTEDMOVINGAVERAGE_H

#include <numeric>
#include <vector>

namespace XbotgoSDK
{

class WeightedMovingAverage
{
public:
    // 构造函数：初始化权重
    WeightedMovingAverage(const std::vector<double>& weights)
        : weights(weights)
    {
    }

    // 更新函数：添加新值并计算加权移动平均
    double update(double new_value);

private:
    std::vector<double> weights; // 权重
    std::vector<double> window;  // 当前窗口数据
};

}

#endif // WEIGHTEDMOVINGAVERAGE_H