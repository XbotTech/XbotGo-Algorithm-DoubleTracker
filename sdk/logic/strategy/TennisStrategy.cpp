#include "TennisStrategy.h"
#include "XbotgoTennisStrategyCallback.h"

namespace XbotgoSDK
{
    extern XbotgoTennisStrategyCallback* g_xbotgoTennisStrategyCallback;
// 找到面积最大的矩形
    std::vector<BallDetectionResult> TennisStrategy::findLargestRectArea(const std::vector<BallDetectionResult>& list) {
        if (list.empty()) {
            return {};
        }

        auto getArea = [](const BallDetectionResult& rec) {
            return rec.width * rec.height;
        };

        auto largest = std::max_element(list.begin(), list.end(), [&getArea](const BallDetectionResult& a, const BallDetectionResult& b) {
            return getArea(a) < getArea(b);
        });

        return { *largest };
    }

    std::vector<YOLODetectionResult> TennisStrategy::findLastRect(const std::vector<YOLODetectionResult>& list) {
        if (list.empty()) {
            return {};
        }

        auto getBottom = [](const YOLODetectionResult& rec) {
            return rec.y + rec.height;
        };

        auto lastest = std::max_element(list.begin(), list.end(), [&getBottom](const YOLODetectionResult& a, const YOLODetectionResult& b) {
            return getBottom(a) < getBottom(b);
        });

        return { *lastest };
    }

    std::vector<YOLODetectionResult> TennisStrategy::sortByBottomDescending(std::vector<YOLODetectionResult>& list) {
        if (list.empty()) {
            return {};
        }

        std::sort(list.begin(), list.end(), [](const YOLODetectionResult& rec1, const YOLODetectionResult& rec2) {
            float bottom1 = rec1.y + rec1.height;
            float bottom2 = rec2.y + rec2.height;
            return bottom2 < bottom1;
        });

        return list;
    }

    std::vector<YOLODetectionResult> TennisStrategy::filterByBottomAndLeftRight(const BallDetectionResult& entity, const std::vector<YOLODetectionResult>& list) {
        std::vector<YOLODetectionResult> filteredList;

        float entityBottom = entity.y + entity.height;
        float entityLeft = entity.x;
        float entityRight = entity.x + entity.width;

        for (const auto& rec : list) {
            float recBottom = rec.y + rec.height;
            float recLeft = rec.x;
            float recRight = rec.x + rec.width;

            // 条件：rec的底部在entity的底部线以下
            // 并且rec的右边框在entity的左边框右面
            // 并且rec的左边框在entity的右边框左面
            if (recBottom > entityBottom && recRight > entityLeft && recLeft < entityRight) {
                filteredList.push_back(rec);
            }
        }

        return filteredList;
    }
    std::vector<YOLODetectionResult> TennisStrategy::filterBelowMiddleOfEntityBottom(const BallDetectionResult& entity, const std::vector<YOLODetectionResult>& list, float bufferHeight) {
        std::vector<YOLODetectionResult> filteredList;

        float entityBottom = entity.y + entity.height;
        float middlePosition = (entityBottom + bufferHeight) / 2.0f;
        for (const auto& rec : list) {
            float recBottom = rec.y + rec.height;
            // 条件：rec的底部在entity的底部和屏幕底部之间的中间位置以下
            if (recBottom > middlePosition ) {
                filteredList.push_back(rec);
            }
        }

        return filteredList;
    }
    std::vector<YOLODetectionResult> TennisStrategy::filterByBottom(const BallDetectionResult& entity, const std::vector<YOLODetectionResult>& list) {
        std::vector<YOLODetectionResult> filteredList;

        float entityBottom = entity.y + entity.height;

        for (const auto& rec : list) {
            float recBottom = rec.y + rec.height;
            // 条件：rec的底部在entity的底部线以下
            if (recBottom > entityBottom ) {
                filteredList.push_back(rec);
            }
        }

        return filteredList;
    }
    int TennisStrategy::track(const std::vector<YOLODetectionResult>& yoloResult, const std::vector<BallDetectionResult>& hoopResult,int type) {
        //单打
        if (type == 0) {
            //筛选出最大的球网为最终框
            std::vector<BallDetectionResult> largestHoopRect = findLargestRectArea(hoopResult);
            //获取屏幕宽高
            int bufferWidth = controllerManger->videoBufferWidth;
            int bufferHeight = controllerManger->videoBufferHight;

            //targetY默认为不转动
            float targetX = bufferWidth/2;
            //targetX默认为不转动
            float targetY = bufferHeight / 2;
            // int type = -1;
            if (!largestHoopRect.empty()) {
                //筛选出球网“下面”的人，即近侧的人
                auto filteredList = filterByBottom(largestHoopRect[0], yoloResult);
                float  hoopCenter = largestHoopRect[0].x + largestHoopRect[0].width/2;
                //筛选出最接近底线（镜头）的人
                auto lastPersonRect = findLastRect(filteredList);

                //targetY为球网下边线+120，保证球网显示在屏幕的中间偏上
                targetY = largestHoopRect[0].y + largestHoopRect[0].height + bufferHeight * 0.2;
                targetX = hoopCenter;
                if (!lastPersonRect.empty()) {
                    //获取人的中点坐标
                    float centerX = lastPersonRect[0].x + lastPersonRect[0].width / 2;

                    int leftLimit = bufferWidth * 2 / 15;
                    int rightLimit = bufferWidth * 13 / 15;
                    //如果人的位置在屏幕中间区域右测,向右转动
                    if (centerX > rightLimit) {
                        //向右转动,球网左边缘接近屏幕左测（左侧距离屏幕100像素以内），再想右转球网显示不全，不转动
                        if (largestHoopRect[0].x <= bufferWidth * 0.08) {
                            targetX = bufferWidth / 2;
                            type = 0;
                        }
                        //中点加上 人和边界的距离  为追踪点
                        else{
                            targetX = bufferWidth / 2 + (centerX - rightLimit)*3;
                            type = 1;
                        }
                    }
                    //如果人中点在屏幕中间区域左侧(中间偏左150以外)，向左转动
                    else if (centerX < leftLimit){
                        //向左转动,球网右边缘接近屏幕右测（右侧距离屏幕100像素以内），再想左转球网显示不全，不转动
                        if (largestHoopRect[0].x + largestHoopRect[0].width >= bufferWidth - bufferWidth * 0.08) {
                            targetX = bufferWidth / 2;
                            type = 2;
                        }
                            //中点减去 人和边界的距离  为追踪点
                        else {
                            targetX = bufferWidth / 2 - (leftLimit - centerX) *3;
                            type = 3;
                        }
                    }

                        //如果人中点在屏幕中间区域   球网中点为中心点
                    else {

                        if (hoopCenter > bufferWidth / 2 + bufferWidth * 0.02 || hoopCenter < bufferWidth / 2 - bufferWidth * 0.02) {
                            // 判断人是否接近屏幕边缘
                            if (centerX > bufferWidth  / 6 && centerX < bufferWidth * 5 /6) {
                                // 人不在屏幕边缘，才以球网中点为中心点
                                targetX = hoopCenter;
                                type = 4;
                            } else {
                                // 人在屏幕边缘，不转动
                                type = 5;
                                targetX = bufferWidth / 2;
                            }
                        } else{
                            type = 6;
                            targetX = bufferWidth/2;
                        }
                    }
                }
            }
            TrackResult point = TrackResult(targetX, targetY);
            // targetX,targetY即跟踪目标点
            g_xbotgoTennisStrategyCallback->onTrackingPoint(point);
            return 0;
        }
        //双打
        else{
            //筛选出最大的球网为最终框
            std::vector<BallDetectionResult> largestHoopRect = findLargestRectArea(hoopResult);
            //获取屏幕宽高
            int bufferWidth = controllerManger->videoBufferWidth;
            int buferHeight = controllerManger->videoBufferHight;

            //targetY默认为不转动
            float targetX = bufferWidth/2;
            //targetX默认为不转动
            float targetY = buferHeight / 2;
            if (!largestHoopRect.empty()) {
                //筛选出球网“下面”的人，即近侧的人
                auto filteredList = filterByBottomAndLeftRight(largestHoopRect[0], yoloResult);

                //筛选出最接近底线（镜头）的人
                auto personByBottom = sortByBottomDescending(filteredList);

                //targetY为球网下边线+120，保证球网显示在屏幕的中间偏上
                targetY = largestHoopRect[0].y + largestHoopRect[0].height + buferHeight * 0.17;
                if (personByBottom.size() >= 2) {
                    //获取第一个人的的中点坐标
                    float centerXFirst = personByBottom[0].x + personByBottom[0].width / 2;
                    //获取二个人的的中点坐标
                    float centerXSecond = personByBottom[1].x + personByBottom[1].width / 2;
                    //获取两个人中间的坐标
                    float centerX = (centerXFirst + centerXSecond) / 2;

                    //如果两个人中间点在屏幕中间区域左侧 (中间偏左150以外)，向左转动
                    if (centerX < bufferWidth / 2 - bufferWidth * 0.12) {
                        //向左转动,球网右边缘接近屏幕右测（右侧距离屏幕100像素以内），再想左转球网显示不全，不转动
                        if (largestHoopRect[0].x + largestHoopRect[0].width >= bufferWidth - bufferWidth * 0.08) {
                            targetX = bufferWidth / 2;
                        }
                            //正常转动
                        else {
                            targetX = centerX;
                        }
                    }
                        //如果两个人中间点在屏幕中间区域右侧   向右转动
                    else if (centerX > bufferWidth / 2 + bufferWidth * 0.12) {
                        //向右转动,球网左边缘接近屏幕左测（左侧距离屏幕100像素以内），再想右转球网显示不全，不转动
                        if (largestHoopRect[0].x <= bufferWidth * 0.08) {
                            targetX = bufferWidth / 2;
                        }
                            //正常转动
                        else {
                            targetX = centerX;
                        }
                    }
                        //如果果两个人中间点在屏幕中间区域   不转动
                    else {
                        targetX = bufferWidth / 2;
                    }
                }
            }
            TrackResult point = TrackResult(targetX, targetY);
            // targetX,targetY即跟踪目标点
            g_xbotgoTennisStrategyCallback->onTrackingPoint(point);
            return 0;
        };
    }
}