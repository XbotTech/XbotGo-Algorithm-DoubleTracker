#include "SingleStrategy.h"

namespace XbotgoSDK
{

class JerseyNumberSingleStrategy: public SingleStrategy 
{

public:
    JerseyNumberSingleStrategy(){
        std::cout << "JerseyNumberSingleStrategy is being created." << std::endl;
    };
    ~JerseyNumberSingleStrategy(){
        std::cout << "JerseyNumberSingleStrategy is being destroyed." << std::endl;  
    };

    void targetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::shared_ptr<DetectionResult>& target)override;

protected:

    int tryToDoTragetInit(const std::vector<std::shared_ptr<DetectionResult>>& detectResults)override;

    int doFeatureProcessing(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::vector<YOLODetectionResult>& yoloResults, int index)override;

    void zoom(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue)override;

    void control(const std::vector<YOLODetectionResult>& detectResults, int index, double currentZoomValue)override;

private:

    int handleSOTSuccess(std::vector<JerseyNumberDetectionResult>& jerseyNumberResults, const std::vector<YOLODetectionResult>& yoloResults, int index);
    int handleSOTFailure(std::vector<JerseyNumberDetectionResult>& jerseyNumberResults, const std::vector<YOLODetectionResult>& yoloResults, int index);

    int doFullFeatureNetFound(const std::vector<JerseyNumberDetectionResult>& jerseyNumberResults, int index);

    /// @brief 根据下标更新FeatureNet特征队列
    /// @param index 
    void updateFeatureNet(int index);

    /// @brief 重置SOT
    /// @param jerseyNumberResults 检测数据
    /// @param targetIndex 目标数组下标
    void restartSOT(const std::vector<JerseyNumberDetectionResult>& jerseyNumberResults, int targetIndex);

    /// @brief 将Buffer中的坐标位置通过X,Y的偏移量转换到虚拟坐标中
    /// @param jerseyNumberResults 检测出的结果
    /// @param virtualResults 虚拟坐标结果
    void convertJerseyNumberResultsToVirtualResults(const std::vector<JerseyNumberDetectionResult> &jerseyNumberResults, std::vector<YOLODetectionResult> &virtualResults);

    /// @brief 
    /// @param index 
    /// @return 
    int featureNetVerification(const std::vector<JerseyNumberDetectionResult>& jerseyNumberDetectionResults, int index);

    int findTargetJerseyNumber(const std::vector<std::shared_ptr<DetectionResult>>& detectResults, const std::string& targetJerseyNumber);

    int findTargetJerseyNumber(const std::vector<JerseyNumberDetectionResult>& jerseyNumberDetectionResults, const std::string& targetJerseyNumber);

    /// @brief 转换YOLO数组到RectBox数组
    /// @param results 
    /// @return 
    std::vector<RectBox> extractYOLODetectionResults(const std::vector<YOLODetectionResult>& results);

private:

    std::string _targetJerseyNumber;

    bool _canAddFeatureNet = false;
    
};

}
