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
            case PoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ:
                return "pose_only_ba_4dof_xyz";
            case PoseRefineMethod::POSE_ONLY_BA_4DOF_YPD:
                return "pose_only_ba_4dof_ypd";
        }
        return "pose_only_ba_4dof_xyz";
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
        if (name == "pose_only_ba_4dof_xyz" || name == "ba_4dof_xyz") {
            return PoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ;
        }
        if (name == "pose_only_ba_4dof_ypd" || name == "ba_4dof_ypd") {
            return PoseRefineMethod::POSE_ONLY_BA_4DOF_YPD;
        }
        return PoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ;
    }

} // namespace armor_detector::pose
