#include "StrategyFactory.h"
#include "Strategy.h"

namespace XbotgoSDK
{

IStrategy* StrategyFactory::createProduct(InitStrategyType type)
{
    IStrategy* strategy = nullptr;
    StrategyParam param = {10, 10, 10, 50, 2000};
    switch (type) {
    case InitStrategyType::INIT_STRATEGY_TYPE_TEAM: // 团队模式
        strategy = new TeamStrategy();
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_BASKETBALL_WHOLE_OVER14_CHAMELEON:  // 篮球  全场  十四岁以上  chameleon
    case InitStrategyType::INIT_STRATEGY_TYPE_BASKETBALL_WHOLE_UNDER14_CHAMELEON: // 篮球  全场  十四岁以下  chameleon
    case InitStrategyType::INIT_STRATEGY_TYPE_BASKETBALL_HALF_OVER14_CHAMELEON:   // 篮球  半场  十四岁以上  chameleon
    case InitStrategyType::INIT_STRATEGY_TYPE_BASKETBALL_HALF_UNDER14_CHAMELEON:  // 篮球  半场  十四岁以下  chameleon
        param.motFPS = 16;
        param.FastPeopleThreshold = 105;
        strategy = new BasketballTeamStrategy(param);
        break;

    // 套壳篮球
    case InitStrategyType::INIT_STRATEGY_TYPE_HANDBALL_WHOLE_OVER14:  // 手球  全场  十四岁以上
    case InitStrategyType::INIT_STRATEGY_TYPE_HANDBALL_WHOLE_UNDER14: // 手球  全场  十四岁以下
    case InitStrategyType::INIT_STRATEGY_TYPE_HANDBALL_HALF_OVER14:   // 手球  半场  十四岁以上
    case InitStrategyType::INIT_STRATEGY_TYPE_HANDBALL_HALF_UNDER14:  // 手球  半场  十四岁以下
        param.motFPS = 16;
        param.FastPeopleThreshold = 105;
        strategy = new BasketballTeamStrategy(param);
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_5V5_OVER14_CHAMELEON:  // 足球  5v5  十四岁以上  chameleon
    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_5V5_UNDER14_CHAMELEON: // 足球  5v5  十四岁以下  chameleon
        param.motFPS = 16;
        param.FastPeopleThreshold = 105;
        strategy = new BasketballTeamStrategy(param);
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_7V7_OVER14_CHAMELEON:  // 足球    7v7    十四岁以上  chameleon
    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_7V7_UNDER14_CHAMELEON: // 足球    7v7    十四岁以下  chameleon
        param.motFPS = 8;
        param.FastPeopleThreshold = 50;
        strategy = new FootballTeamStrategy(param);
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_11V11_OVER14_CHAMELEON:  // 足球    11v11  十四岁以上  chameleon
    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_11V11_UNDER14_CHAMELEON: // 足球    11v11  十四岁以下  chameleon
        param.motFPS = 8;
        param.FastPeopleThreshold = 50;
        param.yoloAreaThreshold = 0;
        strategy = new FootballTeamStrategy(param);
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_ICE_HOCKEY_WHOLE_OVER14:  // 冰球     全场    十四岁以上
    case InitStrategyType::INIT_STRATEGY_TYPE_ICE_HOCKEY_WHOLE_UNDER14: // 冰球     全场    十四岁以下
        param.motFPS = 8;
        param.FastPeopleThreshold = 40;
        strategy = new IceHockeyTeamStrategy(param);
        break;

    // 套壳足球
    case InitStrategyType::INIT_STRATEGY_TYPE_WHEELCHAIR_SOCCER:        // 轮椅足球
    case InitStrategyType::INIT_STRATEGY_TYPE_RUGBY_WHOLE_OVER14:       // 橄榄球   全场    十四岁以上
    case InitStrategyType::INIT_STRATEGY_TYPE_RUGBY_WHOLE_UNDER14:      // 橄榄球   全场    十四岁以下
    case InitStrategyType::INIT_STRATEGY_TYPE_HOCKEY_WHOLE_OVER14:      // 曲棍球   全场    十四岁以上
    case InitStrategyType::INIT_STRATEGY_TYPE_HOCKEY_WHOLE_UNDER14:     // 曲棍球   全场    十四岁以下
    case InitStrategyType::INIT_STRATEGY_TYPE_BROOM_BALL_WHOLE_OVER14:  // 扫帚球   全场    十四岁以上
    case InitStrategyType::INIT_STRATEGY_TYPE_BROOM_BALL_WHOLE_UNDER14: // 扫帚球   全场    十四岁以下
        param.motFPS = 8;
        param.FastPeopleThreshold = 30;
        strategy = new FootballTeamStrategy(param);
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_TENNIS_SINGLE: // 网球    单打
        strategy = new TennisStrategy();
        break;
    case InitStrategyType::INIT_STRATEGY_TYPE_TENNIS_DOUBLE: // 网球    双打
        strategy = new TennisStrategy();
        break;
    case InitStrategyType::INIT_STRATEGY_TYPE_PICKLEBALL_SINGLE: // 匹克球    单打
        strategy = new PickleballStrategy(PickleballStrategy::Single);
        break;
    case InitStrategyType::INIT_STRATEGY_TYPE_PICKLEBALL_DOUBLE: // 匹克球    双打
        strategy = new PickleballStrategy(PickleballStrategy::Double);
        break;
    case InitStrategyType::INIT_STRATEGY_TYPE_JERSEYNUMBER: // 球衣号码
        strategy = new JerseyNumberSingleStrategy();
        break;
    case InitStrategyType::INIT_STRATEGY_TYPE_FOLLOWMEPOSE: // 单人新Pose模式
        strategy = new SingleStrategyPose();
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_BASKETBALL_WHOLE_OVER14_GIMBAL:  // 篮球  全场  十四岁以上  gimbal
    case InitStrategyType::INIT_STRATEGY_TYPE_BASKETBALL_WHOLE_UNDER14_GIMBAL: // 篮球  全场  十四岁以下  gimbal
    case InitStrategyType::INIT_STRATEGY_TYPE_BASKETBALL_HALF_OVER14_GIMBAL:   // 篮球  半场  十四岁以上  gimbal
    case InitStrategyType::INIT_STRATEGY_TYPE_BASKETBALL_HALF_UNDER14_GIMBAL:  // 篮球  半场  十四岁以下  gimbal
        param.motFPS = 30;
        param.frameInterval = 25;
        param.caculateInterval = 25;
        param.FastPeopleThreshold = 70;
        param.yoloAreaThreshold = 1000;
        strategy = new BasketballTeamStrategyGimbal(param);
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_5V5_OVER14_GIMBAL:  // 足球  5v5  十四岁以上  gimbal
    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_5V5_UNDER14_GIMBAL: // 足球  5v5  十四岁以下  gimbal
        param.motFPS = 30;
        param.frameInterval = 20;
        param.caculateInterval = 20;
        param.FastPeopleThreshold = 100;
        param.yoloAreaThreshold = 600;
        strategy = new FootballTeamStrategyGimbal(param);
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_7V7_OVER14_GIMBAL:  // 足球    7v7    十四岁以上  gimbal
    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_7V7_UNDER14_GIMBAL: // 足球    7v7    十四岁以下  gimbal
        param.motFPS = 30;
        param.frameInterval = 25;
        param.caculateInterval = 25;
        param.FastPeopleThreshold = 100;
        param.yoloAreaThreshold = 600;
        strategy = new FootballTeamStrategyGimbal(param);
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_11V11_OVER14_GIMBAL:  // 足球    11v11  十四岁以上  gimbal
    case InitStrategyType::INIT_STRATEGY_TYPE_SOCCER_11V11_UNDER14_GIMBAL: // 足球    11v11  十四岁以下  gimbal
        param.motFPS = 30;
        param.frameInterval = 25;
        param.caculateInterval = 25;
        param.FastPeopleThreshold = 100;
        param.yoloAreaThreshold = 600;
        strategy = new FootballTeamStrategyGimbal(param);
        break;

    case InitStrategyType::INIT_STRATEGY_TYPE_ICE_HOCKEY_WHOLE_OVER14_GIMBAL:  // 冰球     全场    十四岁以上
    case InitStrategyType::INIT_STRATEGY_TYPE_ICE_HOCKEY_WHOLE_UNDER14_GIMBAL: // 冰球     全场    十四岁以下
        param.motFPS = 30;
        param.frameInterval = 25;
        param.caculateInterval = 25;
        param.FastPeopleThreshold = 100;
        param.yoloAreaThreshold = 0;
        strategy = new FootballTeamStrategyGimbal(param);
        break;

    default:
        break;
    }

    return strategy;
}

}
