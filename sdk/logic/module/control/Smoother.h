#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <iostream>
#include <string>

#include "Global.h"
#include "WeightedMovingAverage.h"

namespace XbotgoSDK
{

class Smoother
{
public:
    Smoother(const double& _max_decrement, const double& _speed_limit, const double& _speed_limit_init, const int& _speed_limit_increase, const std::string& _logPrefix = "")
    {
        max_decrement = _max_decrement;
        speed_limit = _speed_limit;
        speed_limit_init = _speed_limit_init;
        speed_limit_increase = _speed_limit_increase;

        logPrefix_ = _logPrefix;
    }

    double smooth(const double& input);

private:
    int max_decrement = 120;
    double speed_limit = 40;
    double speed_limit_init = 40;
    double speed_limit_increase = 80;

    int direction = 1;
    double last_output = 0.0;

    std::string logPrefix_;
};

}

#endif // SMOOTHER_H
