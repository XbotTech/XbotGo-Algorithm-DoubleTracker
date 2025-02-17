
#ifndef __XBOTGO_SDK_CORE_STRATEGYFACTORY_H__
#define __XBOTGO_SDK_CORE_STRATEGYFACTORY_H__

#include <iostream>
#include <memory>

#include "Global.h"
#include "IStrategy.h"
#include "XbotgoStrategy.h"

namespace XbotgoSDK
{

class StrategyFactory
{
public:
    StrategyFactory(){printfXbotGo("StrategyFactory\n");}
    ~StrategyFactory(){printfXbotGo("StrategyFactory::~\n");}

    static IStrategy* createProduct(InitStrategyType type);

private:
};

}

#endif
