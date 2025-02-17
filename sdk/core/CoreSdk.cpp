#include "CoreSdk.h"
// #include <dirent.h>
// #include <glog/logging.h>

namespace XbotgoSDK
{

std::shared_ptr<StrategyManger> CoreSdk::getStrategyManger()
{
    return strategyManger;
}

std::shared_ptr<ControllerManger> CoreSdk::getControllerManger()
{
    return controllerManger;
}

int CoreSdk::setLogfilePath(std::string url, int enableLog)
{
    // LOG(INFO) << "CoreSdk::setLogfilePath "<< url << " enableLog="<<enableLog;

    // google::InitGoogleLogging("xbotgo");

    // google::SetLogDestination(google::GLOG_FATAL, (url + "log_fatal_").c_str()); // 设置 google::FATAL 级别的日志存储路径和文件名前缀
    // google::SetLogDestination(google::GLOG_ERROR, (url + "log_error_").c_str()); //设置 google::ERROR 级别的日志存储路径和文件名前缀
    // google::SetLogDestination(google::GLOG_WARNING, (url + "log_warning_").c_str()); //设置 google::WARNING 级别的日志存储路径和文件名前缀
    // google::SetLogDestination(google::GLOG_INFO, (url + "log_info_").c_str()); //设置 google::INFO 级别的日志存储路径和文件名前缀
    // FLAGS_logbufsecs = 0; //缓冲日志输出，默认为30秒，此处改为立即输出
    // FLAGS_max_log_size = 100; //最大日志大小为 100MB
    // FLAGS_stop_logging_if_full_disk = true; //当磁盘被写满时，停止日志输出
    // if(enableLog==0)
    // {
    //     FLAGS_logtostderr = true;     //如果FLAGS_logtostderr设置为true，日志将输出到标准输出；如果设置为false，则日志将输出到指定的文件中
    // }
    // else
    // {
    //     FLAGS_alsologtostderr = true; // 设置日志消息除了日志文件之外还可显示到屏幕上
    // }
    // FLAGS_colorlogtostderr = true; // 彩色打印输出

    // LOG(INFO) << "SDK version 0.0.2 ";
    // LOG(INFO) << "CoreSdk::setLogfilePath "<< url << " enableLog="<<enableLog;

    return 0;
}

}