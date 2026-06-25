#include "armor_detector/detector/ArmorDetector.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace armor_detector {

    ArmorDetector::ArmorDetector(const Params &params) : params_(params) {
    }

    bool ArmorDetector::checkLightColor(const LightBar &left, const LightBar &right) const {
        if (left.color != right.color) {
            return false;
        }
        if (left.color != params_.target_color) {
            return false;
        }
        return true;
    }

    ArmorGeometry ArmorDetector::buildGeometry(const LightBar &left, const LightBar &right) const {
        ArmorGeometry geo;
        geo.paired_lights = {left, right};
        geo.corners = {left.top, right.top, right.bottom, left.bottom};

        // angle_diff_deg: smallest angular difference in degrees
        double diff = std::abs(tools::radToDeg(left.angle - right.angle));
        diff = std::min(diff, 180.0 - diff);
        geo.angle_diff_deg = diff;

        // length_ratio: min/max of the two light bar lengths
        double max_len = std::max(left.length, right.length);
        double min_len = std::min(left.length, right.length);
        geo.length_diff = (max_len > 1e-6) ? (min_len / max_len) : 0.0;

        // Local coordinate differences
        double global_x_diff = right.center.x - left.center.x;
        double global_y_diff = right.center.y - left.center.y;
        double sinx = std::sin(left.angle);
        double cosx = std::cos(left.angle);
        double local_x = std::abs(-global_x_diff * sinx + global_y_diff * cosx);
        double local_y = std::abs(global_x_diff * cosx + global_y_diff * sinx);
        double mean_length = (left.length + right.length) / 2.0;

        geo.x_diff_ratio = (mean_length > 1e-6) ? (local_x / mean_length) : 0.0;
        geo.y_diff_ratio = (mean_length > 1e-6) ? (local_y / mean_length) : 0.0;

        // Distance ratio and armor type
        double distance = cv::norm(left.center - right.center);
        if (distance > 1e-6 && mean_length > 1e-6) {
            double distance_by_length = distance / mean_length;
            geo.distance_ratio = mean_length / distance;
            geo.type = (distance_by_length > kLargeArmorDistanceRatio) ? ArmorType::LARGE : ArmorType::SMALL;
        }
        else {
            geo.distance_ratio = 0.0;
            geo.type = ArmorType::NONE;
        }

        return geo;
    }

    bool ArmorDetector::checkArmorGeometry(const ArmorGeometry &geometry, std::string *detail) const {
        std::ostringstream oss;

        if (geometry.type == ArmorType::NONE) {
            oss << "type=NONE";
            *detail = oss.str();
            return false;
        }

        if (geometry.angle_diff_deg > params_.max_angle_diff) {
            oss << "angle=" << geometry.angle_diff_deg << " > " << params_.max_angle_diff;
            *detail = oss.str();
            return false;
        }

        if (geometry.length_diff < params_.min_length_ratio) {
            oss << "len_ratio=" << geometry.length_diff << " < " << params_.min_length_ratio;
            *detail = oss.str();
            return false;
        }

        if (geometry.x_diff_ratio < params_.min_x_diff_ratio) {
            oss << "x_diff=" << geometry.x_diff_ratio << " < " << params_.min_x_diff_ratio;
            *detail = oss.str();
            return false;
        }

        if (geometry.y_diff_ratio > params_.max_y_diff_ratio) {
            oss << "y_diff=" << geometry.y_diff_ratio << " > " << params_.max_y_diff_ratio;
            *detail = oss.str();
            return false;
        }

        if (geometry.distance_ratio < params_.min_distance_ratio ||
            geometry.distance_ratio > params_.max_distance_ratio) {
            oss << "dist_ratio=" << geometry.distance_ratio << " not in [" << params_.min_distance_ratio << ", "
                << params_.max_distance_ratio << "]";
            *detail = oss.str();
            return false;
        }

        return true;
    }

    std::vector<ArmorCandidate> ArmorDetector::match(const std::vector<LightBar> &lights) {
        armor_debug_.candidates.clear();
        armor_debug_.rejected_armors.clear();

        std::vector<ArmorCandidate> candidates;

        // Sort by center.x to ensure left-to-right ordering
        std::vector<LightBar> sorted_lights = lights;
        std::sort(sorted_lights.begin(), sorted_lights.end(), [](const LightBar &a, const LightBar &b) {
            return a.center.x < b.center.x;
        });

        for (std::size_t i = 0; i < sorted_lights.size(); ++i) {
            for (std::size_t j = i + 1; j < sorted_lights.size(); ++j) {
                const auto &left = sorted_lights[i];
                const auto &right = sorted_lights[j];

                // Check color
                if (!checkLightColor(left, right)) {
                    debug::RejectedArmor rejected;
                    rejected.geometry.paired_lights = {left, right};
                    rejected.reason = debug::DebugRejectReason::BAD_COLOR;
                    rejected.detail = "color mismatch or != target";
                    armor_debug_.rejected_armors.push_back(rejected);
                    continue;
                }

                // Build geometry
                ArmorGeometry geometry = buildGeometry(left, right);

                // Check geometry thresholds
                std::string detail;
                if (!checkArmorGeometry(geometry, &detail)) {
                    debug::RejectedArmor rejected;
                    rejected.geometry = geometry;
                    if (detail.find("angle") != std::string::npos) {
                        rejected.reason = debug::DebugRejectReason::BAD_ANGLE;
                    }
                    else if (detail.find("len_ratio") != std::string::npos) {
                        rejected.reason = debug::DebugRejectReason::BAD_LENGTH_RATIO;
                    }
                    else if (detail.find("x_diff") != std::string::npos) {
                        rejected.reason = debug::DebugRejectReason::BAD_X_DIFF;
                    }
                    else if (detail.find("y_diff") != std::string::npos) {
                        rejected.reason = debug::DebugRejectReason::BAD_Y_DIFF;
                    }
                    else if (detail.find("dist_ratio") != std::string::npos) {
                        rejected.reason = debug::DebugRejectReason::BAD_DISTANCE;
                    }
                    else {
                        rejected.reason = debug::DebugRejectReason::UNKNOWN;
                    }
                    rejected.detail = detail;
                    armor_debug_.rejected_armors.push_back(rejected);
                    continue;
                }

                // Passed all checks
                ArmorCandidate candidate;
                candidate.geometry = geometry;
                candidates.push_back(candidate);
                armor_debug_.candidates.push_back(candidate);
            }
        }

        return candidates;
    }

} // namespace armor_detector
