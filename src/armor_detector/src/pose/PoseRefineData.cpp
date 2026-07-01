#include "armor_detector/pose/PoseRefineData.hpp"

namespace armor_detector::pose {

    std::string toString(PoseRefineMethod method) {
        switch (method) {
            case PoseRefineMethod::NONE:
                return "none";
            case PoseRefineMethod::YAW_SEARCH:
                return "yaw_search";
            case PoseRefineMethod::POSE_ONLY_BA_6DOF:
                return "pose_only_ba_6dof";
            case PoseRefineMethod::POSE_ONLY_BA_4DOF:
                return "pose_only_ba_4dof";
        }
        return "pose_only_ba_4dof";
    }

    PoseRefineMethod poseRefineMethodFromString(std::string_view name) {
        if (name == "none") {
            return PoseRefineMethod::NONE;
        }
        if (name == "yaw_search") {
            return PoseRefineMethod::YAW_SEARCH;
        }
        if (name == "pose_only_ba_6dof" || name == "ba_6dof") {
            return PoseRefineMethod::POSE_ONLY_BA_6DOF;
        }
        if (name == "pose_only_ba_4dof" || name == "ba_4dof") {
            return PoseRefineMethod::POSE_ONLY_BA_4DOF;
        }
        return PoseRefineMethod::POSE_ONLY_BA_4DOF;
    }

} // namespace armor_detector::pose
