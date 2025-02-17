
#ifndef __XBOTGO_SDK_CORE_STRATEGYMANGER_H__
#define __XBOTGO_SDK_CORE_STRATEGYMANGER_H__

#include <iostream>
#include <memory>

#include "Global.h"
#include "Strategy.h"
#include "StrategyFactory.h"

namespace XbotgoSDK
{

class StrategyManger
{
public:
    StrategyManger()
        : strategy(nullptr){}
    ~StrategyManger()
    {
        if (nullptr != strategy) {
            delete strategy;
            strategy = nullptr;
        }
        printfXbotGo("StrategyManger::~\n");
    }

    void setContext(InitStrategyType type);
    IStrategy* getContext();

private:
    IStrategy* strategy;
};

}

#endif
