#ifndef __XBOTGO_SDK_CORE_LOGIC_BALLTRACKER_H__
#define __XBOTGO_SDK_CORE_LOGIC_BALLTRACKER_H__

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>

#include "Global.h"
#include "XbotgoStrategyCommon.h"

namespace XbotgoSDK
{

#define REAL_BALL 1      // 真实的球
#define PREDICTED_BALL 0 // 预测的球
#define EMPTY_BALL -1    // 空的球

#define STOP 0               // 停止指令
#define TURN_LEFT -1         // 向左转指令
#define TURN_RIGHT 1         // 向右转指令
#define TURN_LEFT_QUICKLY -6 // 向左快速转指令
#define TURN_RIGHT_QUICKLY 6 // 向右快速转指令

class BallTracker
{
private:
    bool initialized = false;     // 是否完成初始化标识符
    int predict_count_ = 0;       // 预测计数
    int initialized_count_ = 0;   // 初始化计数器
    float min_search_range_ = 50; // 最小搜索范围

    int fps_;
    int buffer_size_;
    int initialized_size_;
    int variance_threshold_x_;
    int variance_threshold_y_;
    int rotate_compensation_threshold_;

    // 轨迹池，每个deque代表一条轨迹，轨迹中存储的信息含义为 (球的类型, 球的位置)球的类型分别是-1代表空的球，0表示预测的球，1表示真实的球
    std::vector<std::deque<std::pair<int, BallDetectionResult>>> trajectory_pool_;

    inline int calculateRealBallCount(const std::deque<std::pair<int, BallDetectionResult>>& trajectory) const
    {
        int count = 0;
        for (const auto& ball : trajectory) {
            if (ball.first == REAL_BALL) {
                count++;
            }
        }
        return count;
    }

    inline float calculateCenterX(const BallDetectionResult& ball) const
    {
        return ball.x + ball.width / 2.0;
    }

    inline float calculateCenterY(const BallDetectionResult& ball) const
    {
        return ball.y + ball.height / 2.0;
    }

    inline float calculateEuclideanDistanceSquared(const BallDetectionResult& ball_a, const BallDetectionResult& ball_b) const
    {
        float dx = calculateCenterX(ball_a) - calculateCenterX(ball_b);
        float dy = calculateCenterY(ball_a) - calculateCenterY(ball_b);
        return dx * dx + dy * dy;
    }

    void ballTrajectoryAssociation(std::vector<std::deque<std::pair<int, BallDetectionResult>>>& trajectory_pool, const std::vector<BallDetectionResult>& ball_detection_results);

    void cleanUpTrajectoryPool(std::vector<std::deque<std::pair<int, BallDetectionResult>>>& trajectory_pool, int buffer_size);

    std::pair<float, float> calculateVariance(const std::deque<std::pair<int, BallDetectionResult>>& trajectory) const;

    float calculateTotalMovementX(const std::deque<std::pair<int, BallDetectionResult>>& trajectory) const;

    float calculateTotalMovementY(const std::deque<std::pair<int, BallDetectionResult>>& trajectory) const;

    std::deque<std::pair<int, BallDetectionResult>> selectTrajectory(std::vector<std::deque<std::pair<int, BallDetectionResult>>>& trajectory_pool) const;

    std::pair<int, BallDetectionResult> direct(std::deque<std::pair<int, BallDetectionResult>>& trajectory);

public:
    BallTracker(const int& fps = 20, const int& initialized_size = 5, const int& variance_threshold_x = 1600, const int& variance_threshold_y = 1600, const int& rotate_compensation_threshold = 200)
    {
        this->fps_ = fps;                                                     // 帧率
        this->buffer_size_ = fps + 10;                                        // 轨迹长度
        this->initialized_size_ = initialized_size;                           // 初始化所需的帧数
        this->variance_threshold_x_ = variance_threshold_x;                   // 过滤静止轨迹的x方差阈值
        this->variance_threshold_y_ = variance_threshold_y;                   // 过滤静止轨迹的y方差阈值
        this->rotate_compensation_threshold_ = rotate_compensation_threshold; // 旋转补偿阈值

        printfXbotGo("BallTracker:Initial parameter\n");
        printfXbotGo("BallTracker:fps = %d\n", fps_);
        printfXbotGo("BallTracker:buffer size = %d\n", buffer_size_);
        printfXbotGo("BallTracker:x variance threshold = %d\n", variance_threshold_x_);
        printfXbotGo("BallTracker:y variance threshold = %d\n", variance_threshold_y_);
    }

    ~BallTracker()
    {
    }

    std::pair<int, BallDetectionResult> update(const std::vector<BallDetectionResult>& ball_detection_results);
};

}

#endif