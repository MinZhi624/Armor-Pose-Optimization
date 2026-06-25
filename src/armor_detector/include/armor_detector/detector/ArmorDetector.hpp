#pragma once

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/tools/angle.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace armor_detector {

    class ArmorDetector {
    public:
        struct Params {
            double max_angle_diff = 15.0;
            double min_length_ratio = 0.7;
            double min_x_diff_ratio = 0.75;
            double max_y_diff_ratio = 1.0;
            double max_distance_ratio = 0.8;
            double min_distance_ratio = 0.1;
            LightBarColor target_color = LightBarColor::BLUE;
        };

        ArmorDetector() = default;
        explicit ArmorDetector(const Params &params);

        std::vector<ArmorCandidate> match(const std::vector<LightBar> &lights);

        const debug::ArmorMatchDebugData &getArmorMatchDebugData() const {
            return armor_debug_;
        }

    private:
        ArmorGeometry buildGeometry(const LightBar &left, const LightBar &right) const;
        bool checkLightColor(const LightBar &left, const LightBar &right) const;
        bool checkArmorGeometry(const ArmorGeometry &geometry, std::string *detail) const;

        Params params_;
        debug::ArmorMatchDebugData armor_debug_;

        static constexpr double kLargeArmorDistanceRatio = 2.8;
    };

} // namespace armor_detector
