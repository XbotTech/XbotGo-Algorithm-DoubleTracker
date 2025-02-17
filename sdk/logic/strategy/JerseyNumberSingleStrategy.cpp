#include "JerseyNumberSingleStrategy.h"
#include "XbotgoSingleStrategyCallback.h"

namespace XbotgoSDK
{

extern XbotgoSingleStrategyCallback*  g_xbotgoSingleStrategyCallbacks;

void JerseyNumberSingleStrategy::targetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target) {

    std::cout << "JerseyNumberSingleStrategy::target init"<<detectResults.size()<<std::endl;

    // 使用 std::dynamic_pointer_cast 来进行智能指针的动态类型转换
    std::shared_ptr<JerseyNumberDetectionResult> jerseyTarget = std::dynamic_pointer_cast<JerseyNumberDetectionResult>(target);

    if(jerseyTarget==nullptr){
        return ;
    }

    _targetJerseyNumber = jerseyTarget->jerseyNumber;

    std::cout << "JerseyNumberSingleStrategy::target init targetJerseyNumber is"<<_targetJerseyNumber<<std::endl;

    _isSOTInit = false;
    tryToDoTragetInit(detectResults);
}

int JerseyNumberSingleStrategy::tryToDoTragetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults) {

    int targetIndex = findTargetJerseyNumber(detectResults, _targetJerseyNumber);

    if (targetIndex != -1) {

        std::cout << "JerseyNumberSingleStrategy::target init targetIndex is"<<targetIndex<<std::endl;

        _sot = std::make_unique<SOT>();
        _featureNet = std::make_unique<FeatureNet>();
        _singleZoom = std::make_unique<SingleZoom>(deviceVideoBufferWidth, deviceVideoBufferHeight, minZoom, maxZoom, 0.5, 0.3, 0.35);


        std::vector<YOLODetectionResult> virtualResults;
        convertToVirtualResults(detectResults, virtualResults);

        _sot->targetInit(virtualResults, targetIndex);
        _featureNet->targetInit(targetIndex, 2.7);
        _isSOTInit = true;
    }

    return targetIndex;
}

int JerseyNumberSingleStrategy::doFeatureProcessing(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int index) {

    std::vector<JerseyNumberDetectionResult> jerseyNumberResults;

    for (const auto& detectResult : detectResults) {
        auto jerseyResult = std::dynamic_pointer_cast<JerseyNumberDetectionResult>(detectResult);
        if (jerseyResult) {
            jerseyNumberResults.push_back(*jerseyResult); // 解引用智能指针以获取对象
        } else {
            // TODO: 处理没有找到的情况
        }
    }
    
    if (index != -1) {
        // SOT追踪成功
        int targetIndexWhenSOTSuccess = handleSOTSuccess(jerseyNumberResults, yoloResults, index);
        return targetIndexWhenSOTSuccess;
    } else {
        // SOT追踪失败
        int targetIndexWhenSOTFailure = handleSOTFailure(jerseyNumberResults, yoloResults, index);
        return targetIndexWhenSOTFailure;
    }
}

int JerseyNumberSingleStrategy::handleSOTSuccess(std::vector<JerseyNumberDetectionResult>& jerseyNumberResults, const std::vector<YOLODetectionResult>& yoloResults, int index) {

    int jerseyNumberTargetId = findTargetJerseyNumber(jerseyNumberResults, _targetJerseyNumber);

    if (jerseyNumberTargetId != -1) {
        // 如果找到了球衣号码

        // TODO: 这里应该判断一下找到的人和队列中的人的featureNet Distance 如果太大意味着不同颜色的号码

        if (isOverLapping(yoloResults, index, 1.1) == false) {
            _canAddFeatureNet = true;
            updateFeatureNet(jerseyNumberTargetId);
        } else {
            _canAddFeatureNet = false;
        }

        if (jerseyNumberTargetId != index) {
            // 如果找到球衣号码的目标，和SOT得出的下标不符
            restartSOT(jerseyNumberResults, index);
        }

        g_xbotgoSingleStrategyCallbacks->onDebugJerseyNumberTrackState(1);

        return jerseyNumberTargetId;

    } else {

        if (jerseyNumberResults[index].jerseyNumber == "") {
            // 如果没有找到号码，信任Byte
            if (isOverLapping(yoloResults, index, 1.1) == false) {
                if (_skipReIDVerifyCount != 0) {
                    /// 前几帧存在重叠情况，需要进行上一帧特征Reid验证
                    _skipReIDVerifyCount --;

                    return featureNetVerification(jerseyNumberResults, index);
                } else {
                    if(_canAddFeatureNet == true) {
                        updateFeatureNet(index);
                    }
                    g_xbotgoSingleStrategyCallbacks->onDebugJerseyNumberTrackState(2);
                    return index;
                }
            } else {
                _skipReIDVerifyCount = 3;
                _canAddFeatureNet = false;

                return featureNetVerification(jerseyNumberResults, index);
            }
        } else {
            // 如果是有球衣号码，说明找到的不是目标的球衣号码，也就是说SOT跟踪失败了
            return handleSOTFailure(jerseyNumberResults, yoloResults, index);
        }
    }
}

int JerseyNumberSingleStrategy::handleSOTFailure(std::vector<JerseyNumberDetectionResult>& jerseyNumberResults, const std::vector<YOLODetectionResult>& yoloResults, int index) {

    _canAddFeatureNet = false;
    
    int targetIndex = findTargetJerseyNumber(jerseyNumberResults, _targetJerseyNumber);

    if (targetIndex != -1) {

        if (isOverLapping(yoloResults, targetIndex, 1.1) == false) {
            // 如果通过JerseyNumber找到目标
            updateFeatureNet(targetIndex);
            _canAddFeatureNet = true;
        } else {
            _canAddFeatureNet = false;
        }

        restartSOT(jerseyNumberResults, targetIndex);
        
        g_xbotgoSingleStrategyCallbacks->onDebugJerseyNumberTrackState(1);
        return targetIndex;
    }

    // TODO: 是否需要考虑使用FeatureNet找回

    _SOTFailureCount++;

    // if (_SOTFailureCount <= 1) {

    //     // ByteTrack追踪失败，且连续失败次数小于3(允许ByteTrack有追踪失败)
    //     return -1;
    // }

    // _SOTFailureCount--;
    _isSkipSOT = true;

    int featureNetIndex = doFullFeatureNetFound(jerseyNumberResults, index);

    // 如果全局FeatureNet找回来的下标中没有号码，则信任FeatureNet，否则认为找回错误
    if (featureNetIndex != -1 && jerseyNumberResults[featureNetIndex].jerseyNumber == "") {
        restartSOT(jerseyNumberResults, featureNetIndex);
        _isSkipSOT = false;
        g_xbotgoSingleStrategyCallbacks->onDebugJerseyNumberTrackState(5);
        return featureNetIndex;
    }

    g_xbotgoSingleStrategyCallbacks->onDebugJerseyNumberTrackState(6);

    return - 1;
}

int JerseyNumberSingleStrategy::doFullFeatureNetFound(const std::vector<JerseyNumberDetectionResult>& jerseyNumberResults, int index) {
    int featureNetIndex = _featureNet->reidTrack();

    std::cout << "JerseyNumberSingleStrategy::doFullFeatureNetFound"<< "featureNet index =" <<index<<std::endl;

    if (featureNetIndex != -1) {
        g_xbotgoSingleStrategyCallbacks->onDebugJerseyNumberTrackState(4);
        restartSOT(jerseyNumberResults, featureNetIndex);
        _SOTFailureCount = 0;
        _isSkipSOT = false;
    } else {
        _isSkipSOT = true;
    }

    return featureNetIndex;
}

void JerseyNumberSingleStrategy::updateFeatureNet(int index) {

    std::cout << "JerseyNumberSingleStrategy::updateFeatureNet"<< "featureNet index =" <<index<<std::endl;

    _featureNet->getFeature(index);                 // 调用API获取ByteTrack成功追踪到的目标的特征
    _featureNet->updateTargetFeatureList(index);    // 更新追踪目标的特征队列
    _featureNet->updatePreviousTargetFeature();     // 将当前成功追踪的目标特征信息更新至上一帧追踪目标特征
}

int JerseyNumberSingleStrategy::featureNetVerification(const std::vector<JerseyNumberDetectionResult>& jerseyNumberDetectionResults, int index) {

    std::cout << "JerseyNumberSingleStrategy::featureNetVerification"<< "featureNet index =" <<index<<std::endl;

    _featureNet->getFeature(index);

    if(_featureNet->reidLocalVerification() == true) {
        g_xbotgoSingleStrategyCallbacks->onDebugJerseyNumberTrackState(3);
        return index;
    } else {
        return doFullFeatureNetFound(jerseyNumberDetectionResults, index);   
    }
}

void JerseyNumberSingleStrategy::zoom(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue) {

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

void JerseyNumberSingleStrategy::control(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue) {

    double targetYaw = deviceVideoBufferWidth/2.0; // 其实是X
    double targetPitch = deviceVideoBufferHeight/2.0; // 其实是Y

    if (index != -1) {
        YOLODetectionResult targetDetectResult = detectResults[index];

        if (targetDetectResult.x + 0.5 * targetDetectResult.width < 0.45 * deviceVideoBufferWidth || targetDetectResult.x + 0.5 * targetDetectResult.width > 0.55 * deviceVideoBufferWidth) {
            targetYaw = ((targetDetectResult.x + 0.5 * targetDetectResult.width - 0.5 * deviceVideoBufferWidth) / currentZoomValue) + 0.5 * deviceVideoBufferWidth;
        }

        targetPitch = ((targetDetectResult.y + 0.2 * targetDetectResult.height - 0.5 * deviceVideoBufferHeight) / currentZoomValue) + 0.5 * deviceVideoBufferHeight;
    }

    g_xbotgoSingleStrategyCallbacks->onSetGimbal(targetYaw, targetPitch);
}


void JerseyNumberSingleStrategy::restartSOT(const std::vector<JerseyNumberDetectionResult>& jerseyNumberResults, int targetIndex) {

    std::vector<YOLODetectionResult> virtualResults;
    convertJerseyNumberResultsToVirtualResults(jerseyNumberResults, virtualResults);
    _sot->restart(virtualResults, targetIndex);
}

void JerseyNumberSingleStrategy::convertJerseyNumberResultsToVirtualResults(const std::vector<JerseyNumberDetectionResult> &jerseyNumberResults, std::vector<YOLODetectionResult> &virtualResults)
{
    virtualResults.clear();
    for (int i = 0; i < static_cast<int>(jerseyNumberResults.size()); i++)
    {
        YOLODetectionResult virtualResult = YOLODetectionResult(0, jerseyNumberResults[i].personResult.prob,
                                                                jerseyNumberResults[i].personResult.x + jerseyNumberResults[i].xOffset,
                                                                jerseyNumberResults[i].personResult.y + jerseyNumberResults[i].yOffset,
                                                                jerseyNumberResults[i].personResult.width,
                                                                jerseyNumberResults[i].personResult.height);
        virtualResults.push_back(virtualResult);
    }
}

/// 查找目标 jerseyNumber 的下标
/// @param detectResults 搜索到的JerseyNumber的下标
/// @param targetJerseyNumber 目标JerseyNumber
int JerseyNumberSingleStrategy::findTargetJerseyNumber(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::string& targetJerseyNumber) {
    for (size_t i = 0; i < detectResults.size(); ++i) {
        // 尝试将基类引用转换为派生类引用
        const auto jerseyResult = std::dynamic_pointer_cast<JerseyNumberDetectionResult>(detectResults[i]);
        if (jerseyResult) {
            // 比较球衣号码
            if (jerseyResult->jerseyNumber == targetJerseyNumber) {
                return static_cast<int>(i);
            }
        }
    }
    return -1; // 未找到匹配项
}

/// 查找目标 jerseyNumber 的下标
/// @param detectResults 搜索到的JerseyNumber的下标
/// @param targetJerseyNumber 目标JerseyNumber
int JerseyNumberSingleStrategy::findTargetJerseyNumber(const std::vector<JerseyNumberDetectionResult>& jerseyNumberDetectionResults, const std::string& targetJerseyNumber) {
    for (size_t i = 0; i < jerseyNumberDetectionResults.size(); ++i) {
        if (jerseyNumberDetectionResults[i].jerseyNumber == targetJerseyNumber) {
            return static_cast<int>(i);
        }
    }
    return -1; // 未找到匹配项
}

// TODO: zoom的时机应该要改改(可能不要把人放的太大)

std::vector<RectBox> JerseyNumberSingleStrategy::extractYOLODetectionResults(const std::vector<YOLODetectionResult>& results) {
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
