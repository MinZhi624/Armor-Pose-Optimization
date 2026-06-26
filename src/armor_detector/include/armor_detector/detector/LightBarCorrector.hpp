#pragma once

#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>
#include <vector>

namespace armor_detector {

    class LightBarCorrector {
    public:
        /// 调度函数：选择一种校正方法修正灯条端点
        void lightbarPointsCorrect(LightBar &light, const std::vector<cv::Point> &contour);

    private:
        /// 基于椭圆拟合的角度校正
        void correctByEllipse(LightBar &light, const std::vector<cv::Point> &contour);

        // 未来添加:
        // void correctByPCA(LightBar &light, const std::vector<cv::Point> &contour);
    };

} // namespace armor_detector
