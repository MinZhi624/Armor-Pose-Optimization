#pragma once

#include <cstddef>
#include <string>

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/pose/PoseRefineData.hpp"

namespace armor_detector::pose {

    struct PoseLandscapeParams {
        bool enabled = false;
        double physical_min_distance_m = 1.0;
        double physical_max_distance_m = 10.0;
        // 已确认首版使用 d0 ± 3 m；物理边界仍会裁剪该窗口。
        double half_window_m = 3.0;
        double distance_step_m = 0.05;
        double pose_yaw_min_deg = -70.0;
        double pose_yaw_max_deg = 70.0;
        double pose_yaw_step_deg = 2.0;
    };

    struct PoseLandscapeSampleInfo {
        std::size_t armor_index = 0;
        int armor_name = 0;
        std::string armor_type;
        double confidence = 0.0;
    };

    /**
     * @brief 对同一 PnP 初值进行固定视线 d-pose_yaw objective 扫描。
     *
     * 该类不依赖 ROS 或文件 I/O；每一个 grid 点只直接计算共享 robust
     * objective，不创建 Ceres Problem，也不执行迭代。
     */
    class PoseLandscapeAnalyzer {
    public:
        explicit PoseLandscapeAnalyzer(PoseLandscapeParams params = {});

        const PoseLandscapeParams &params() const;

        debug::PoseLandscapeSample analyze(const PoseRefineInput &input,
                                           const PoseLandscapeSampleInfo &sample_info) const;

    private:
        PoseLandscapeParams params_;
    };

} // namespace armor_detector::pose
