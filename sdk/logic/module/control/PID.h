#ifndef PID_H
#define PID_H

#include <iostream>
#include <deque>
#include <numeric>
#include <string>

#include "Global.h"
#include "WeightedMovingAverage.h"

namespace XbotgoSDK
{

class PID
{
public:
    PID(const double& _kp, const double& _ki, const double& _kd,
        const int& _resolution, const double& _fps, const double& _maxOut, const std::string& _logPrefix = "")
    {
        kp_ = _kp;
        ki_ = _ki;
        kd_ = _kd;
        dt_ = 1 / _fps;
        pError_ = iError_ = dError_ = 0.0;
        maxOut_ = _maxOut;
        targetPosition_ = _resolution / 2;

        wms = new WeightedMovingAverage({1, 2, 3, 3, 3});

        logPrefix_ = _logPrefix;
    }

    void setMaxOut(const double& _maxOut)
    {
        maxOut_ = std::abs(_maxOut);
    }

    double getMaxOut()
    {
        return maxOut_;
    }

    double update(const double& _trackPosition);

    double Calculate(const double& _error);

    double Compute(const double& _trackPosition);

private:
    double kp_, ki_, kd_, dt_;
    double pError_, iError_, dError_, lastError_;
    double maxOut_;
    double targetPosition_;
    std::string logPrefix_;

    std::deque<double> iDeque_;
    WeightedMovingAverage* wms;
};

}

#endif // PID_H
