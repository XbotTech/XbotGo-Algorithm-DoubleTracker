#include "IStrategy.h"

namespace XbotgoSDK
{

void IStrategy::setGyroscopeController(std::shared_ptr<ControllerManger>& controllerManger)
{
   this->controllerManger = controllerManger;  
}

void IStrategy::setZoomInfo(double minZoom, double maxZoom)
{
    this->minZoom = minZoom;
    this->maxZoom = maxZoom;
}

void IStrategy::setVideoBufferSize(int width, int height) 
{
    this->deviceVideoBufferWidth = width;
    this->deviceVideoBufferHeight = height;
}

void IStrategy::setFps(int fps)
{
    this->fps = fps;
}

}
