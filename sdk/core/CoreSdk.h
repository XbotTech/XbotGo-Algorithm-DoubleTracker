
#ifndef __XBOTGO_SDK_CORE_H__
#define __XBOTGO_SDK_CORE_H__

#include <iostream>
#include <memory>

#include "Global.h"

#include "ControllerManger.h"
#include "StrategyManger.h"

namespace XbotgoSDK
{

class CoreSdk
{
public:
    CoreSdk()
        : controllerManger(std::make_shared<ControllerManger>()),
          strategyManger(std::make_shared<StrategyManger>()){}
    ~CoreSdk(){printfXbotGo("CoreSdk::~\n");}

    std::shared_ptr<ControllerManger> getControllerManger();
    std::shared_ptr<StrategyManger> getStrategyManger();

    int setLogfilePath(std::string url, int enableLog);

private:
    std::shared_ptr<ControllerManger> controllerManger;
    std::shared_ptr<StrategyManger> strategyManger;
};

}

#endif