#include "armor_detector/pose/PoseRefineData.hpp"

namespace armor_detector::pose {

    std::string toString(SinglePoseRefineMethod method) {
        switch (method) {
            case SinglePoseRefineMethod::NONE:
                return "none";
            case SinglePoseRefineMethod::YAW_SEARCH:
                return "yaw_search";
            case SinglePoseRefineMethod::YAW_SEARCH_THEN_DISTANCE:
                return "yaw_search_then_distance";
            case SinglePoseRefineMethod::POSE_ONLY_BA_6DOF:
                return "pose_only_ba_6dof";
            case SinglePoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ:
                return "pose_only_ba_4dof_xyz";
            case SinglePoseRefineMethod::POSE_ONLY_BA_4DOF_YPD:
                return "pose_only_ba_4dof_ypd";
        }
        return "pose_only_ba_4dof_xyz";
    }

    SinglePoseRefineMethod singlePoseRefineMethodFromString(std::string_view name) {
        if (name == "none") {
            return SinglePoseRefineMethod::NONE;
        }
        if (name == "yaw_search") {
            return SinglePoseRefineMethod::YAW_SEARCH;
        }
        if (name == "yaw_search_then_distance") {
            return SinglePoseRefineMethod::YAW_SEARCH_THEN_DISTANCE;
        }
        if (name == "pose_only_ba_6dof" || name == "ba_6dof") {
            return SinglePoseRefineMethod::POSE_ONLY_BA_6DOF;
        }
        if (name == "pose_only_ba_4dof_xyz" || name == "ba_4dof_xyz") {
            return SinglePoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ;
        }
        if (name == "pose_only_ba_4dof_ypd" || name == "ba_4dof_ypd") {
            return SinglePoseRefineMethod::POSE_ONLY_BA_4DOF_YPD;
        }
        return SinglePoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ;
    }

    std::string toString(DualPoseRefineMethod method) {
        switch (method) {
            case DualPoseRefineMethod::NONE:
                return "none";
            case DualPoseRefineMethod::DUAL_ARMOR_BA_3DOF_YPD:
                return "dual_armor_ba_3dof_ypd";
            case DualPoseRefineMethod::DUAL_ARMOR_BA_7DOF_XYZ:
                return "dual_armor_ba_7dof_xyz";
        }
        return "none";
    }

    DualPoseRefineMethod dualPoseRefineMethodFromString(std::string_view name) {
        if (name == "dual_armor_ba_3dof_ypd") {
            return DualPoseRefineMethod::DUAL_ARMOR_BA_3DOF_YPD;
        }
        if (name == "dual_armor_ba_7dof_xyz") {
            return DualPoseRefineMethod::DUAL_ARMOR_BA_7DOF_XYZ;
        }
        return DualPoseRefineMethod::NONE;
    }

} // namespace armor_detector::pose
