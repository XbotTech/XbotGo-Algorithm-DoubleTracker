#include "PID.h"

#define LimitMax(input, max)       \
    {                              \
        if (input > max) {         \
            input = max;           \
        } else if (input < -max) { \
            input = -max;          \
        }                          \
    }

namespace XbotgoSDK
{

double PID::update(const double& _trackPosition)
{
    double error = _trackPosition - targetPosition_;
    double error_wms = wms->update(error);
    double output = Calculate(error_wms);

    printfXbotGo("Control %s input_error:%.1f wms_error:%.1f output_pid:%.1f\n", logPrefix_.c_str(), error, error_wms, output);
    return output;
}

double PID::Calculate(const double& _error)
{
    pError_ = _error;
    iDeque_.push_back(pError_ * dt_);
    if (iDeque_.size() > 3) {
        iDeque_.pop_front();
    }
    iError_ = std::accumulate(iDeque_.begin(), iDeque_.end(), 0);
    dError_ = (pError_ - lastError_) / dt_;
    lastError_ = pError_;

    double output = kp_ * pError_ + ki_ * iError_ + kd_ * dError_;
    LimitMax(output, maxOut_);
    return output;
}

double PID::Compute(const double& _trackPosition)
{
    double error = _trackPosition - targetPosition_;
    pError_ = error;
    iDeque_.push_back(pError_ * dt_);
    if (iDeque_.size() > 3) {
        iDeque_.pop_front();
    }
    iError_ = std::accumulate(iDeque_.begin(), iDeque_.end(), 0);
    dError_ = (pError_ - lastError_) / dt_;
    lastError_ = pError_;

    double output = kp_ * pError_ + ki_ * iError_ + kd_ * dError_;
    LimitMax(output, maxOut_);

    printfXbotGo("Control %s input_error:%.1f output_pid:%.1f\n", logPrefix_.c_str(), error, output);
    return output;
}

}
