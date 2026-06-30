#include "armor_detector/pose/PoseRefineData.hpp"

namespace armor_detector::pose {

    std::string toString(PoseRefineMethod method) {
        switch (method) {
            case PoseRefineMethod::NONE:
                return "none";
            case PoseRefineMethod::YAW_SEARCH:
                return "yaw_search";
        }
        return "yaw_search";
    }

    PoseRefineMethod poseRefineMethodFromString(std::string_view name) {
        if (name == "none") {
            return PoseRefineMethod::NONE;
        }
        if (name == "yaw_search") {
            return PoseRefineMethod::YAW_SEARCH;
        }
        return PoseRefineMethod::YAW_SEARCH;
    }

} // namespace armor_detector::pose