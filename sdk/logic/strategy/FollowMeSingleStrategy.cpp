#include "FollowMeSingleStrategy.h"
#include "XbotgoSingleStrategyCallback.h"


namespace XbotgoSDK
{

extern XbotgoSingleStrategyCallback*  g_xbotgoSingleStrategyCallbacks;

void FollowMeSingleStrategy::targetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target) {

    // TODO: 需要传入fps, deviceVideoBufferWidth, deviceVideoBufferHeight, minZoom, maxZoom
    std::cout << "FollowMeSingleStrategy::target init"<<detectResults.size()<<std::endl;

    // TODO: 这里理论上应该一定能找到targetIndex，但是如果万一没找到应该怎么办？
    int targetIndex = findDetectionResultIndex(detectResults, target);

    std::cout << "FollowMeSingleStrategy::target index"<<targetIndex<<std::endl;


    _sot = std::make_unique<SOT>();
    _featureNet = std::make_unique<FeatureNet>();
    _singleZoom = std::make_unique<SingleZoom>(deviceVideoBufferWidth, deviceVideoBufferHeight, minZoom, maxZoom, 0.4, 0.6, 0.5);



    std::vector<YOLODetectionResult> virtualResults;
    convertToVirtualResults(detectResults, virtualResults);

    _sot->targetInit(virtualResults, targetIndex);
    _featureNet->targetInit(targetIndex, 2.7);
    _isSOTInit = true;
}

int FollowMeSingleStrategy::doFeatureProcessing(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int index) {

    std::cout << "FollowMeSingleStrategy::doFeatureProcessing"<<detectResults.size()<< "target index =" <<index<<std::endl;
    
    if (index != -1) {
        // SOT追踪成功
        int targetIndexWhenSOTSuccess = handleSOTSuccess(detectResults, yoloResults, index);
        return targetIndexWhenSOTSuccess;
    } else {
        // SOT追踪失败
        int targetIndexWhenSOTFailure = handleSOTFailure(detectResults, index);
        return targetIndexWhenSOTFailure;
    }
}

int FollowMeSingleStrategy::handleSOTSuccess(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int index) {
    
    std::cout << "FollowMeSingleStrategy::handleSOTSuccess"<<detectResults.size()<< "target index =" <<index<<std::endl;

    _SOTFailureCount = 0; // 连续失败次数

    // 将ByteTrack成功追踪到的目标的YOLO框放大1.1倍，判断是否存在重叠，应对即将或已经出现的多人重叠的情况
    if (isOverLapping(yoloResults, index) == false) {

        if (_skipReIDVerifyCount != 0) {
            /// 前几帧存在重叠情况，需要进行上一帧特征Reid验证
            _skipReIDVerifyCount --;
            return featureNetVerification(detectResults, index);
        } else {
            updateFeatureNet(index);
            return index;
        }

    } else {
        /// 如果出现多人重叠的情况，进行上一帧特征Reid验证
        _skipReIDVerifyCount = 3;
        return featureNetVerification(detectResults, index);
    }
}

int FollowMeSingleStrategy::handleSOTFailure(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int index) {

    std::cout << "FollowMeSingleStrategy::handleSOTFailure"<<detectResults.size()<< "target index =" <<index<<std::endl;

    _SOTFailureCount++;

    if (_SOTFailureCount <= 2) {

        // ByteTrack追踪失败，且连续失败次数小于3(允许ByteTrack有追踪失败)
        return -1;
    }

    _SOTFailureCount--;
    _isSkipSOT = true;

    return doFullFeatureNetFound(detectResults, index);
}

int FollowMeSingleStrategy::doFullFeatureNetFound(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int index) {
    
    int featureNetIndex = _featureNet->reidTrack();

    _SOTFailureCount++;
    _isSkipSOT = true;

    std::cout << "FollowMeSingleStrategy::doFullFeatureNetFound"<< "featureNet index =" <<index<<std::endl;

    if (featureNetIndex != -1) {
        std::vector<YOLODetectionResult> virtualResults;
        convertToVirtualResults(detectResults, virtualResults);
        _sot->restart(virtualResults, featureNetIndex);
        _SOTFailureCount = 0;
        _isSkipSOT = false;
    }

    return featureNetIndex;
}

void FollowMeSingleStrategy::updateFeatureNet(int index) {

    std::cout << "FollowMeSingleStrategy::updateFeatureNet"<< "featureNet index =" <<index<<std::endl;

    _featureNet->getFeature(index);                 // 调用API获取ByteTrack成功追踪到的目标的特征
    _featureNet->updateTargetFeatureList(index);    // 更新追踪目标的特征队列
    _featureNet->updatePreviousTargetFeature();     // 将当前成功追踪的目标特征信息更新至上一帧追踪目标特征
}

int FollowMeSingleStrategy::featureNetVerification(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int index) {

    std::cout << "FollowMeSingleStrategy::featureNetVerification"<< "featureNet index =" <<index<<std::endl;

    _featureNet->getFeature(index);

    if(_featureNet->reidLocalVerification() == true) {
        return index;
    } else {
        return doFullFeatureNetFound(detectResults, index);   
    }
}

void FollowMeSingleStrategy::zoom(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue) {

    ZoomResult zoomResult{0,0,0};

    if (index == -1) {

        _zoomLostCount++;

        if (_zoomLostCount > 15) {
            zoomResult = _singleZoom->zoomOut(currentZoomValue);
        }

    } else {

        _zoomLostCount = 0;

        std::vector<RectBox> rectBoxes = extractYOLODetectionResults(detectResults);
        zoomResult = _singleZoom->zoom(index, rectBoxes, currentZoomValue);
    }

    g_xbotgoSingleStrategyCallbacks->onSetZoomFactor(zoomResult.zoom_value, zoomResult.zoom_time, zoomResult.zoom_rate);
}

void FollowMeSingleStrategy::control(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue) {

    double targetYaw = deviceVideoBufferWidth/2.0; // 其实是X
    double targetPitch = deviceVideoBufferHeight/2.0; // 其实是Y

    if (index != -1) {
        YOLODetectionResult targetDetectResult = detectResults[index];

        if (targetDetectResult.x <= 0.15 * deviceVideoBufferWidth) {
            targetYaw = 0.5 * (deviceVideoBufferWidth - (deviceVideoBufferWidth / currentZoomValue));
        } else if (targetDetectResult.x >= 0.85 * deviceVideoBufferWidth) {
            targetYaw = 0.5 * (deviceVideoBufferWidth + (deviceVideoBufferWidth / currentZoomValue));
        } else if (targetDetectResult.x > 0.15 * deviceVideoBufferWidth && targetDetectResult.x <= 0.3 * deviceVideoBufferWidth) {
            targetYaw = ((targetDetectResult.x - 0.5 * targetDetectResult.width - 0.5 * deviceVideoBufferWidth) / currentZoomValue) + 0.5 * deviceVideoBufferWidth;
        } else if (targetDetectResult.x >= 0.7 * deviceVideoBufferWidth && targetDetectResult.x < 0.85 * deviceVideoBufferWidth) {
            targetYaw = ((targetDetectResult.x + 0.5 * targetDetectResult.width - 0.5 * deviceVideoBufferWidth) / currentZoomValue) + 0.5 * deviceVideoBufferWidth;
        } else { 
            targetYaw = ((targetDetectResult.x - 0.5 * deviceVideoBufferWidth) / currentZoomValue) + 0.5 * deviceVideoBufferWidth;
        }

        if (targetDetectResult.y - 0.5 * targetDetectResult.height < 0.3 * deviceVideoBufferHeight) {
            targetPitch = ((2 * targetDetectResult.y - 0.5 * targetDetectResult.height - 0.8 * deviceVideoBufferHeight) / currentZoomValue) + 0.5 * deviceVideoBufferHeight;
        } else {
            targetPitch = ((targetDetectResult.y - 0.1 * targetDetectResult.height - 0.5 * deviceVideoBufferHeight) / currentZoomValue) + 0.5 * deviceVideoBufferHeight;
        }
    }

    g_xbotgoSingleStrategyCallbacks->onSetGimbal(targetYaw, targetPitch);
}

int FollowMeSingleStrategy::findDetectionResultIndex(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target) {
    for (size_t i = 0; i < detectResults.size(); ++i) {
        if (*detectResults[i] == *target) {
            return static_cast<int>(i);
        }
    }
    return -1; // 如果未找到则返回 -1
}

std::vector<RectBox> FollowMeSingleStrategy::extractYOLODetectionResults(const std::vector<YOLODetectionResult>& results) {
    std::vector<RectBox> rectBoxes;
    // 使用迭代器遍历输入的 JerseyNumberResult 结果
    for (const auto& result : results) {
        // 将每个 JerseyNumberResult 中的 personYOLODetectionResult 添加到结果中
        RectBox box = {result.x, result.y, result.width, result.height};
        rectBoxes.push_back(box);
    }
    return rectBoxes;
}

}
