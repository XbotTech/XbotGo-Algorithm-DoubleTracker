#include "BallTracker.h"

namespace XbotgoSDK
{

void BallTracker::ballTrajectoryAssociation(std::vector<std::deque<std::pair<int, BallDetectionResult>>>& trajectory_pool, const std::vector<BallDetectionResult>& ball_detection_results)
{
    ////////////////// Step 1: 提取各个轨迹搜索所基于的球 //////////////////
    std::vector<BallDetectionResult> last_ball_of_each_queue;
    // std::vector<int> search_range_of_each_queue;
    for (const auto& trajectory : trajectory_pool) {
        // 从后往前遍历轨迹，找到第一个真实的球
        for (auto ball = trajectory.rbegin(); ball != trajectory.rend(); ++ball) {
            if (ball->first == REAL_BALL) {
                last_ball_of_each_queue.push_back(ball->second);
                break;
            }
        }
    }

    ////////////////// Step 2: 计算所有轨迹和所有球之间的距离 //////////////////
    // 计算last_ball_of_each_queue中所有的球和当前帧中所有的球之间的欧式距离并保存
    std::vector<std::tuple<int, int, float>> distance_data_frame;
    for (size_t i = 0; i < last_ball_of_each_queue.size(); ++i) {
        for (size_t j = 0; j < ball_detection_results.size(); ++j) {
            // 加入面积筛选，如果未通过筛选，则将distance设置为无限大
            float last_ball_area = last_ball_of_each_queue[i].width * last_ball_of_each_queue[i].height;
            float current_ball_area = ball_detection_results[j].width * ball_detection_results[j].height;
            // float diff_area = std::abs(last_ball_of_each_queue[i].width * last_ball_of_each_queue[i].height - ball_detection_results[j].width * ball_detection_results[j].height);
            if (current_ball_area <= 0.5 * last_ball_area || current_ball_area >= 2 * last_ball_area) {
                distance_data_frame.push_back(std::make_tuple(i, j, std::numeric_limits<float>::infinity()));
                continue;
            }

            float dx = (calculateCenterX(last_ball_of_each_queue[i]) - calculateCenterX(ball_detection_results[j]));
            float dy = (calculateCenterY(last_ball_of_each_queue[i]) - calculateCenterY(ball_detection_results[j]));

            int search_range = std::max(std::max(last_ball_of_each_queue[i].width, last_ball_of_each_queue[i].height) * (30 / fps_ + 1), min_search_range_);

            // 如果dx或dy大于搜索范围，则将distance设置为无限大
            if (std::abs(dx) > search_range || std::abs(dy) > search_range) {
                distance_data_frame.push_back(std::make_tuple(i, j, std::numeric_limits<float>::infinity()));
                continue;
            }

            float distance_squared = dx * dx + dy * dy;
            if (distance_squared > search_range * search_range) {
                distance_data_frame.push_back(std::make_tuple(i, j, std::numeric_limits<float>::infinity()));
            } else {
                distance_data_frame.push_back(std::make_tuple(i, j, std::sqrt(distance_squared)));
            }
        }
    }

    ////////////////// Step 3: 利用距离对轨迹和球进行关联 //////////////////
    // 按distance从小到大排序
    std::sort(distance_data_frame.begin(), distance_data_frame.end(), [](const auto& a, const auto& b) { return std::get<2>(a) < std::get<2>(b); });

    // 创建两个标记数组，分别用于标记每个轨迹和球是否已经被关联
    std::vector<bool> trajectory_associated(last_ball_of_each_queue.size(), false);
    std::vector<bool> ball_associated(ball_detection_results.size(), false);

    // 遍历distance_data_frame，将每个球关联到最近的轨迹上,每个轨迹只能被一个球关联
    for (const auto& data : distance_data_frame) {
        int trajectory_index = std::get<0>(data);
        int ball_index = std::get<1>(data);
        float distance = std::get<2>(data);

        if (distance == std::numeric_limits<float>::infinity())
            break;

        if (trajectory_associated[trajectory_index])
            continue;

        if (ball_associated[ball_index])
            continue;

        trajectory_associated[trajectory_index] = true;
        ball_associated[ball_index] = true;
        trajectory_pool[trajectory_index].push_back({REAL_BALL, ball_detection_results[ball_index]});

        printfXbotGo("BallTracker: The ball (%f, %f, %f, %f) in trajectory %d is associated with the input ball (%f, %f, %f, %f)\n", last_ball_of_each_queue[trajectory_index].x, last_ball_of_each_queue[trajectory_index].y, last_ball_of_each_queue[trajectory_index].width, last_ball_of_each_queue[trajectory_index].height, trajectory_index, ball_detection_results[ball_index].x, ball_detection_results[ball_index].y, ball_detection_results[ball_index].width, ball_detection_results[ball_index].height);
    }

    // 为没有关联到轨迹的球创建新的轨迹
    for (size_t i = 0; i < ball_detection_results.size(); ++i) {
        if (!ball_associated[i]) {
            trajectory_pool.push_back({{REAL_BALL, ball_detection_results[i]}});
            printfXbotGo("BallTracker: Create a new trajectory for the ball (%.2f, %.2f, %.2f, %.2f)\n", ball_detection_results[i].x, ball_detection_results[i].y, ball_detection_results[i].width, ball_detection_results[i].height);
        }
    }

    // 为没有关联到的轨迹存入一个空的球
    for (size_t i = 0; i < trajectory_pool.size(); ++i) {
        if (!trajectory_associated[i]) {
            trajectory_pool[i].push_back({EMPTY_BALL, BallDetectionResult()});
        }
    }
}

void BallTracker::cleanUpTrajectoryPool(std::vector<std::deque<std::pair<int, BallDetectionResult>>>& trajectory_pool, int buffer_size)
{
    for (auto trajectory = trajectory_pool.begin(); trajectory != trajectory_pool.end();) {
        // 将超过数量阈值的轨迹的第一个球删去
        if (static_cast<int>(trajectory->size()) > buffer_size) {
            trajectory->pop_front();
        }

        // 遍历单个轨迹中所有的球，若一个真实的球都没有，则删除该轨迹
        bool has_real_ball = false;
        for (const auto& ball : *trajectory) {
            if (ball.first == REAL_BALL) // 如果找到真实的球
            {
                has_real_ball = true;
                break;
            }
        }

        if (!has_real_ball) {
            trajectory = trajectory_pool.erase(trajectory); // 如果没有真实的球，则删除该轨迹
        } else {
            ++trajectory; // 否则，继续下一个轨迹
        }
    }
}

std::pair<float, float> BallTracker::calculateVariance(const std::deque<std::pair<int, BallDetectionResult>>& trajectory) const
{
    std::vector<float> real_ball_x_positions;
    std::vector<float> real_ball_y_positions;

    // 提取真实的球的 x 和 y 坐标
    for (const auto& ball : trajectory) {
        if (ball.first == REAL_BALL) {
            real_ball_x_positions.push_back(calculateCenterX(ball.second));
            real_ball_y_positions.push_back(calculateCenterY(ball.second));
        }
    }

    // 如果真实的球少于 2 个，无法计算方差，返回 0
    if (real_ball_x_positions.size() <= 1) {
        return {0.0, 0.0};
    }

    // 计算 x 和 y 坐标的均值
    float sum_x = 0.0, sum_y = 0.0;
    for (size_t i = 0; i < real_ball_x_positions.size(); ++i) {
        sum_x += real_ball_x_positions[i];
        sum_y += real_ball_y_positions[i];
    }
    float mean_x = sum_x / real_ball_x_positions.size();
    float mean_y = sum_y / real_ball_y_positions.size();

    // 计算 x 和 y 坐标的方差
    float variance_x = 0.0, variance_y = 0.0;
    for (size_t i = 0; i < real_ball_x_positions.size(); ++i) {
        variance_x += std::pow(real_ball_x_positions[i] - mean_x, 2);
        variance_y += std::pow(real_ball_y_positions[i] - mean_y, 2);
    }
    variance_x /= real_ball_x_positions.size();
    variance_y /= real_ball_y_positions.size();

    return {variance_x, variance_y}; // 返回方差
}

float BallTracker::calculateTotalMovementX(const std::deque<std::pair<int, BallDetectionResult>>& trajectory) const
{
    // 提取真实的球的 x 坐标
    std::vector<float> real_ball_x_positions;
    for (const auto& ball : trajectory) {
        if (ball.first == REAL_BALL) {
            real_ball_x_positions.push_back(calculateCenterX(ball.second));
        }
    }

    // 如果真实的球少于 2 个，无法计算移动距离，返回 0
    if (real_ball_x_positions.size() <= 1) {
        return 0.0;
    }

    // 计算总的 x 方向上的移动距离
    float total_movement_x = 0.0;
    for (size_t i = 1; i < real_ball_x_positions.size(); ++i) {
        total_movement_x += (real_ball_x_positions[i] - real_ball_x_positions[i - 1]);
    }

    return total_movement_x;
}

float BallTracker::calculateTotalMovementY(const std::deque<std::pair<int, BallDetectionResult>>& trajectory) const
{
    // 提取真实的球的 y 坐标
    std::vector<float> real_ball_y_positions;
    for (const auto& ball : trajectory) {
        if (ball.first == REAL_BALL) {
            real_ball_y_positions.push_back(calculateCenterY(ball.second));
        }
    }

    // 如果真实的球少于 2 个，无法计算移动距离，返回 0
    if (real_ball_y_positions.size() <= 1) {
        return 0.0;
    }

    // 计算总的 y 方向上的移动距离
    float total_movement_y = 0.0;
    for (size_t i = 1; i < real_ball_y_positions.size(); ++i) {
        total_movement_y += (real_ball_y_positions[i] - real_ball_y_positions[i - 1]);
    }

    return total_movement_y;
}

std::deque<std::pair<int, BallDetectionResult>> BallTracker::selectTrajectory(std::vector<std::deque<std::pair<int, BallDetectionResult>>>& trajectory_pool) const
{
    size_t max_movement_trajectory_index = trajectory_pool.size();
    float max_movement = 0.0;

    for (size_t i = 0; i < trajectory_pool.size(); ++i) {
        const auto& trajectory = trajectory_pool[i];

        printfXbotGo("BallTracker: Trajectory %zu", i);

        // 计算轨迹中真实的球的数量
        int real_ball_count = calculateRealBallCount(trajectory);
        printfXbotGo(" the number of true balls is %d", real_ball_count);

        // 如果轨迹中真实的球少于 buffer_size_/3 个，跳过该轨迹
        if (real_ball_count <= buffer_size_ / 4) {
            printfXbotGo("\n");
            continue;
        }

        auto variance = calculateVariance(trajectory);
        printfXbotGo(", x variance:%2.f, y variance:%2.f", variance.first, variance.second);
        if (variance.first < variance_threshold_x_ && variance.second < variance_threshold_y_) {
            printfXbotGo("\n");
            continue;
        }

        float movement_x = calculateTotalMovementX(trajectory);
        float movement_y = calculateTotalMovementY(trajectory);
        float movement = std::sqrt(movement_x * movement_x + movement_y * movement_y);
        printfXbotGo(", x movement:%2.f, y movement:%2.f, total movement:%2.f\n", movement_x, movement_y, movement);

        if (movement > max_movement) {
            max_movement = movement;
            max_movement_trajectory_index = i;
        }
    }

    if (max_movement_trajectory_index < trajectory_pool.size()) {
        printfXbotGo("BallTracker: The best trajectory is '%zu' \n", max_movement_trajectory_index);
        return trajectory_pool[max_movement_trajectory_index];
    } else {
        printfXbotGo("BallTracker: Not found the best trajectory\n");
        return std::deque<std::pair<int, BallDetectionResult>>(); // 如果没有找到，返回一个空的deque
    }
}

std::pair<int, BallDetectionResult> BallTracker::direct(std::deque<std::pair<int, BallDetectionResult>>& trajectory)
{
    if (!trajectory.empty()) {
        if (trajectory.back().first == REAL_BALL) // 如果轨迹的最后一个球是真实的球，则返回该球并计算移动方向
        {
            printfXbotGo("BallTracker: Tracking result  'Real ball':(%.2f, %.2f, %.2f, %.2f)", trajectory.back().second.x, trajectory.back().second.y, trajectory.back().second.width, trajectory.back().second.height);
            int direction = STOP;
            // float speed_x = calculateTotalMovementX(trajectory) / trajectory.size();
            float speed_x = calculateTotalMovementX(trajectory);
            if (abs(speed_x) <= rotate_compensation_threshold_) {
                if (speed_x > 0) {
                    direction = TURN_RIGHT;
                    printfXbotGo(", direction to right\n");
                } else {
                    direction = TURN_LEFT;
                    printfXbotGo(", direction to left\n");
                }
            } else if (speed_x > rotate_compensation_threshold_) {
                direction = TURN_RIGHT_QUICKLY;
                printfXbotGo(", direction fast to right\n");
            } else {
                direction = TURN_LEFT_QUICKLY;
                printfXbotGo(", direction fast to left\n");
            }
            predict_count_ = 0;
            return {direction, trajectory.back().second};
        }
        // 如果轨迹的最后一个球是空的或者是预测的球
        else {
            BallDetectionResult last_real_ball;
            BallDetectionResult second_last_real_ball;
            bool found_last = false;
            bool found_second_last = false;
            // 从后往前遍历轨迹，找到最后一个和倒数第二个真实的球
            for (auto ball = trajectory.rbegin(); ball != trajectory.rend(); ++ball) {
                if (ball->first == REAL_BALL) {
                    if (!found_last) {
                        last_real_ball = ball->second;
                        found_last = true;
                    } else if (!found_second_last) {
                        second_last_real_ball = ball->second;
                        found_second_last = true;
                        break; // 找到第二个真实球后可以退出循环
                    }
                }
            }

            predict_count_++;
            BallDetectionResult predicted_ball;

            float speed_x = calculateCenterX(last_real_ball) - calculateCenterX(second_last_real_ball);
            float speed_y = calculateCenterY(last_real_ball) - calculateCenterY(second_last_real_ball);

            float decay_sum = (1 - std::pow(0.7, predict_count_)) / (1 - 0.7);
            predicted_ball.x = last_real_ball.x + speed_x * decay_sum;
            predicted_ball.y = last_real_ball.y + speed_y * decay_sum;
            predicted_ball.prob = last_real_ball.prob;
            predicted_ball.width = last_real_ball.width;
            predicted_ball.height = last_real_ball.height;

            // 删除轨迹中最后一个球
            trajectory.pop_back();
            // 将预测的球添加到轨迹中
            trajectory.push_back({PREDICTED_BALL, predicted_ball});
            printfXbotGo("BallTracker: Tracking result  'Predicted ball':(%.2f, %.2f, %.2f, %.2f)", predicted_ball.x, predicted_ball.y, predicted_ball.width, predicted_ball.height);
            printfXbotGo(", the first two real balls x speed: %.2f, y speed: %.2f, prediction count: %d", speed_x, speed_y, predict_count_);

            int direction = STOP;
            if (speed_x > 0) {
                direction = TURN_RIGHT;
                printfXbotGo(", direction to right\n");
            } else {
                direction = TURN_LEFT;
                printfXbotGo(", direction to left\n");
            }
            return {direction, predicted_ball};
        }
    }
    printfXbotGo("BallTracker: Track result is empty\n");
    return {STOP, BallDetectionResult()};
}

std::pair<int, BallDetectionResult> BallTracker::update(const std::vector<BallDetectionResult>& ball_detection_results)
{
    if (initialized == false) {
        // 如果轨迹池为空，则直接为每一个检测到的球创建一个新的轨迹
        if (trajectory_pool_.empty()) {
            for (auto& ball : ball_detection_results) {
                trajectory_pool_.push_back(std::deque<std::pair<int, BallDetectionResult>>{std::make_pair(REAL_BALL, ball)});
            }
            initialized_count_ = 1;
            printfXbotGo("BallTracker:---Initializing, trajectory pool is empty---\n");
        } else {
            // 如果没有检测到球，则为轨迹池中的每个轨迹添加一个空的球
            if (ball_detection_results.empty()) {
                for (auto& trajectory : trajectory_pool_) {
                    trajectory.push_back({EMPTY_BALL, BallDetectionResult()});
                }
            }
            // 如果检测到球，则尝试为每个检测到的球找到最匹配的轨迹
            else {
                ballTrajectoryAssociation(trajectory_pool_, ball_detection_results);
            }
            initialized_count_++;
            if (initialized_count_ >= initialized_size_) {
                initialized = true;
                initialized_count_ = 0;
                predict_count_ = 0;
            }
            printfXbotGo("BallTracker:-----------Initializing counts:%d-----------\n", initialized_count_);
        }
    } else {
        cleanUpTrajectoryPool(trajectory_pool_, buffer_size_); // 清理轨迹池

        // 如果轨迹池为空，重新初始化
        if (trajectory_pool_.empty()) {
            initialized = false;
            printfXbotGo("BallTracker:-------------Reinitialization--------------\n");
        } else {
            printfXbotGo("BallTracker:-----------------Tracking------------------\n");
            // 如果没有检测到球，则为轨迹池中的每个轨迹添加一个空的球
            if (ball_detection_results.empty()) {
                for (auto& trajectory : trajectory_pool_) {
                    trajectory.push_back({EMPTY_BALL, BallDetectionResult()});
                }
            }
            // 如果检测到球，则尝试为每个检测到的球找到最匹配的轨迹
            else {
                ballTrajectoryAssociation(trajectory_pool_, ball_detection_results);
            }

            auto selected_trajectory = selectTrajectory(trajectory_pool_); // 选择最佳轨迹

            return direct(selected_trajectory); // 直接返回最佳轨迹
        }
    }
    printfXbotGo("BallTracker: Tracking result is empty\n");
    return {STOP, BallDetectionResult()};
}

}
