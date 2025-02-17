#include "Smoother.h"

namespace XbotgoSDK
{

double Smoother::smooth(const double& input)
{
    double output = input;

    if (output > 0) {
        if (direction == 0) {
            speed_limit = speed_limit_init;
            direction = 1;
        }
    } else if (output < 0) {
        if (direction == 1) {
            speed_limit = speed_limit_init;
            direction = 0;
        }
        output = -output;
    }

    if (output >= speed_limit) {
        output = speed_limit;
        if (speed_limit < 150) {
            speed_limit += 40;
        } else if (speed_limit < 400) {
            speed_limit += speed_limit_increase;
        } else {
            speed_limit += speed_limit_increase * 2;
        }

    } else if (output < speed_limit_init) {
        speed_limit = speed_limit_init;
    } else if (output < speed_limit) {
        if (speed_limit < 150) {
            speed_limit = output + 40;
        } else if (speed_limit < 400) {
            speed_limit = output + speed_limit_increase;
        } else {
            speed_limit = output + speed_limit_increase * 2;
        }
    }

    if (direction == 0) {
        output = 0 - output;
    }

    // 方向向右
    if (direction == 1) {
        if (last_output < -max_decrement) {
            output = last_output + max_decrement;
        } else {
            if (last_output <= max_decrement * 1.5) {
                if (last_output - output > max_decrement / 4.0) {
                    output = last_output - max_decrement / 4.0;
                }
                output = std::max(output, 0.0);
            } else if (last_output > 400) {
                if (last_output - output > max_decrement * 2) {
                    output = last_output - max_decrement * 2;
                }
            } else {
                if (last_output - output > max_decrement) {
                    output = last_output - max_decrement;
                    if (output <= max_decrement * 1.5) {
                        output = max_decrement * 1.5;
                    }
                }
            }
        }
    }
    // 方向向左
    else {
        if (last_output > max_decrement) {
            output = last_output - max_decrement;
        } else {
            if (last_output >= -max_decrement * 1.5) {
                if (last_output - output < -max_decrement / 4.0) {
                    output = last_output + max_decrement / 4.0;
                }
                output = std::min(output, 0.0);
            } else if (last_output < -400) {
                if (last_output - output < -max_decrement * 2) {
                    output = last_output + max_decrement * 2;
                }
            } else {
                if (last_output - output < -max_decrement) {
                    output = last_output + max_decrement;
                    if (output >= -max_decrement * 1.5) {
                        output = -max_decrement * 1.5;
                    }
                }
            }
        }
    }

    last_output = output;

    printfXbotGo("Governor %s speed_limit:%.1f final_output:%.1f\n", logPrefix_.c_str(), speed_limit, output);

    return output;
}

}
