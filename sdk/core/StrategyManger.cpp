#include "StrategyManger.h"

namespace XbotgoSDK
{

void StrategyManger::setContext(InitStrategyType type)
{
    strategy = StrategyFactory::createProduct(type);
}

IStrategy*  StrategyManger::getContext()
{
    return strategy;
}

}
