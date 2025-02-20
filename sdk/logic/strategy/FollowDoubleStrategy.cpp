#include "FollowDoubleStrategy.h"
#include "XbotgoSingleStrategyCallback.h"

namespace XbotgoSDK
{

extern XbotgoSingleStrategyCallback* g_xbotgoSingleStrategyCallbacks;

/// @brief 通过下标初始化目标,注意此时为两个目标
/// @param detectResults yolo的检测结果
/// @param targets 目标的信息 指针
void FollowDoubleSingleStrategy::myTargetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<std::shared_ptr<DetectionResult>>& targets)
{
    std::cout << "FollowDoubleSingleStrategy::targetInit, deteResults.size() : " << detectResults.size() << std::endl;
    std::cout << "FollowDoubleSingleStrategy::targetInit, targets.size() : " << targets.size() << std::endl;

    std::vector<int> targetsIndex(2, -1);
    targetsIndex[0] = findDetectionResultIndex(detectResults, targets[0]);
    targetsIndex[1] = findDetectionResultIndex(detectResults, targets[1]);

    std::cout << "FollowDoubleSingleStrategy::targetIndex[0] : " << targetsIndex[0] << std::endl;
    std::cout << "FollowDoubleSingleStrategy::targetIndex[1] : " << targetsIndex[1] << std::endl;
    _doubleSOT = std::make_unique<DoubleSOT>();
    _doubleFeatureNet = std::make_unique<DoubleFeatureNet>();
    _singleZoom = std::make_unique<SingleZoom>(deviceVideoBufferWidth, deviceVideoBufferHeight, minZoom, maxZoom, 0.4, 0.6, 0.5);

    std::vector<YOLODetectionResult> virtualResults;
    convertToVirtualResults(detectResults, virtualResults);

    _doubleSOT->targetInit(virtualResults, targetsIndex);
    _doubleFeatureNet->targetInit(targetsIndex, 2.7);

    _isSOTInit = true;
}

void FollowDoubleSingleStrategy::targetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target)
{

    // // TODO: 需要传入fps, deviceVideoBufferWidth, deviceVideoBufferHeight, minZoom, maxZoom
    // std::cout << "FollowDoubleSingleStrategy::target init" << detectResults.size() << std::endl;

    // // TODO: 这里理论上应该一定能找到targetIndex，但是如果万一没找到应该怎么办？
    // int targetIndex = findDetectionResultIndex(detectResults, target);

    // std::cout << "FollowDoubleSingleStrategy::target index" << targetIndex << std::endl;

    // _sot = std::make_unique<SOT>();
    // _featureNet = std::make_unique<FeatureNet>();
    // _singleZoom = std::make_unique<SingleZoom>(deviceVideoBufferWidth, deviceVideoBufferHeight, minZoom, maxZoom, 0.4, 0.6, 0.5);

    // std::vector<YOLODetectionResult> virtualResults;
    // convertToVirtualResults(detectResults, virtualResults);

    // _sot->targetInit(virtualResults, targetIndex);
    // _featureNet->targetInit(targetIndex, 2.7);
    // _isSOTInit = true;
}

int FollowDoubleSingleStrategy::mydoFeatureProcessing(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int id, int index)
{

    std::cout << "FollowDoubleSingleStrategy::doFeatureProcessing" << detectResults.size() << "target index =" << index << std::endl;

    if (index != -1) {
        // SOT追踪成功
        int targetIndexWhenSOTSuccess = handleSOTSuccess(detectResults, yoloResults, id, index);
        return targetIndexWhenSOTSuccess;
    }
    else {
        // SOT追踪失败
        int targetIndexWhenSOTFailure = handleSOTFailure(detectResults, id, index);
        return targetIndexWhenSOTFailure;
    }
}

int FollowDoubleSingleStrategy::handleSOTSuccess(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int id, int index)
{

    std::cout << "FollowDoubleSingleStrategy::handleSOTSuccess" << detectResults.size() << "target index =" << index << std::endl;

    _SOTFailureCount = 0; // 连续失败次数

    // 将ByteTrack成功追踪到的目标的YOLO框放大1.1倍，判断是否存在重叠，应对即将或已经出现的多人重叠的情况
    if (isOverLapping(yoloResults, index) == false) {

        if (_skipReIDVerifyCount != 0) {
            /// 前几帧存在重叠情况，需要进行上一帧特征Reid验证
            _skipReIDVerifyCount--;
            return featureNetVerification(detectResults, id, index);
        }
        else {
            updateFeatureNet(id, index);
            return index;
        }
    }
    else {
        /// 如果出现多人重叠的情况，进行上一帧特征Reid验证
        _skipReIDVerifyCount = 3;
        return featureNetVerification(detectResults, id, index);
    }
}

int FollowDoubleSingleStrategy::handleSOTFailure(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int id, int index)
{

    std::cout << "FollowDoubleSingleStrategy::handleSOTFailure" << detectResults.size() << "target index =" << index << std::endl;

    _SOTFailureCount++;

    if (_SOTFailureCount <= 2) {

        // ByteTrack追踪失败，且连续失败次数小于3(允许ByteTrack有追踪失败)
        return -1;
    }

    _SOTFailureCount--;
    _isSkipSOT = true;

    return doFullFeatureNetFound(detectResults, id, index);
}

int FollowDoubleSingleStrategy::doFullFeatureNetFound(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int id, int index)
{

    int featureNetIndex = _doubleFeatureNet->reidTrack(id);

    _SOTFailureCount++;
    _isSkipSOT = true;

    std::cout << "FollowDoubleSingleStrategy::doFullFeatureNetFound" << "featureNet index =" << index << std::endl;

    if (featureNetIndex != -1) {
        std::vector<YOLODetectionResult> virtualResults;
        convertToVirtualResults(detectResults, virtualResults);
        // TODO 需要重启 SOT 需要将当前两个 Index都传入进去
        // _doubleSOT->restart(virtualResults, featureNetIndex);
        _SOTFailureCount = 0;
        _isSkipSOT = false;
    }
    return featureNetIndex;
}

void FollowDoubleSingleStrategy::updateFeatureNet(int id, int index)
{

    std::cout << "FollowDoubleSingleStrategy::updateFeatureNet" << "featureNet index =" << index << std::endl;

    _doubleFeatureNet->getFeature(id, index);              // 调用API获取ByteTrack成功追踪到的目标的特征
    _doubleFeatureNet->updateTargetFeatureList(id, index); // 更新追踪目标的特征队列
    _doubleFeatureNet->updatePreviousTargetFeature();      // 将当前成功追踪的目标特征信息更新至上一帧追踪目标特征
}

int FollowDoubleSingleStrategy::featureNetVerification(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, int id, int index)
{

    std::cout << "FollowDoubleSingleStrategy::featureNetVerification" << "featureNet index =" << index << std::endl;

    _doubleFeatureNet->getFeature(id, index);

    if (_doubleFeatureNet->reidLocalVerification(id) == true) {
        return index;
    }
    else {
        return doFullFeatureNetFound(detectResults, id, index);
    }
}

void FollowDoubleSingleStrategy::zoom(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue)
{

    ZoomResult zoomResult{0, 0, 0};

    if (index == -1) {

        _zoomLostCount++;

        if (_zoomLostCount > 15) {
            zoomResult = _singleZoom->zoomOut(currentZoomValue);
        }
    }
    else {

        _zoomLostCount = 0;

        std::vector<RectBox> rectBoxes = extractYOLODetectionResults(detectResults);
        zoomResult = _singleZoom->zoom(index, rectBoxes, currentZoomValue);
    }

    g_xbotgoSingleStrategyCallbacks->onSetZoomFactor(zoomResult.zoom_value, zoomResult.zoom_time, zoomResult.zoom_rate);
}

void FollowDoubleSingleStrategy::control(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue)
{

    double targetYaw = deviceVideoBufferWidth / 2.0;    // 其实是X
    double targetPitch = deviceVideoBufferHeight / 2.0; // 其实是Y

    if (index != -1) {
        YOLODetectionResult targetDetectResult = detectResults[index];

        if (targetDetectResult.x <= 0.15 * deviceVideoBufferWidth) {
            targetYaw = 0.5 * (deviceVideoBufferWidth - (deviceVideoBufferWidth / currentZoomValue));
        }
        else if (targetDetectResult.x >= 0.85 * deviceVideoBufferWidth) {
            targetYaw = 0.5 * (deviceVideoBufferWidth + (deviceVideoBufferWidth / currentZoomValue));
        }
        else if (targetDetectResult.x > 0.15 * deviceVideoBufferWidth && targetDetectResult.x <= 0.3 * deviceVideoBufferWidth) {
            targetYaw = ((targetDetectResult.x - 0.5 * targetDetectResult.width - 0.5 * deviceVideoBufferWidth) / currentZoomValue) + 0.5 * deviceVideoBufferWidth;
        }
        else if (targetDetectResult.x >= 0.7 * deviceVideoBufferWidth && targetDetectResult.x < 0.85 * deviceVideoBufferWidth) {
            targetYaw = ((targetDetectResult.x + 0.5 * targetDetectResult.width - 0.5 * deviceVideoBufferWidth) / currentZoomValue) + 0.5 * deviceVideoBufferWidth;
        }
        else {
            targetYaw = ((targetDetectResult.x - 0.5 * deviceVideoBufferWidth) / currentZoomValue) + 0.5 * deviceVideoBufferWidth;
        }

        if (targetDetectResult.y - 0.5 * targetDetectResult.height < 0.3 * deviceVideoBufferHeight) {
            targetPitch = ((2 * targetDetectResult.y - 0.5 * targetDetectResult.height - 0.8 * deviceVideoBufferHeight) / currentZoomValue) + 0.5 * deviceVideoBufferHeight;
        }
        else {
            targetPitch = ((targetDetectResult.y - 0.1 * targetDetectResult.height - 0.5 * deviceVideoBufferHeight) / currentZoomValue) + 0.5 * deviceVideoBufferHeight;
        }
    }

    g_xbotgoSingleStrategyCallbacks->onSetGimbal(targetYaw, targetPitch);
}

int FollowDoubleSingleStrategy::findDetectionResultIndex(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target)
{
    for (size_t i = 0; i < detectResults.size(); ++i) {
        if (*detectResults[i] == *target) {
            return static_cast<int>(i);
        }
    }
    return -1; // 如果未找到则返回 -1
}
std::vector<RectBox> FollowDoubleSingleStrategy::extractYOLODetectionResults(const std::vector<YOLODetectionResult>& results)
{
    std::vector<RectBox> rectBoxes;
    // 使用迭代器遍历输入的 JerseyNumberResult 结果
    for (const auto& result : results) {
        // 将每个 JerseyNumberResult 中的 personYOLODetectionResult 添加到结果中
        RectBox box = {result.x, result.y, result.width, result.height};
        rectBoxes.push_back(box);
    }
    return rectBoxes;
}

} // namespace XbotgoSDK
