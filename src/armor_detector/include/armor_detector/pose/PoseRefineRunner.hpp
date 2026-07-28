#pragma once

#include "armor_detector/pose/DualArmorBa3DofYPDRefiner.hpp"
#include "armor_detector/pose/IPoseRefiner.hpp"
#include "armor_detector/pose/NoneRefiner.hpp"
#include "armor_detector/pose/PoseOnlyBa4DofXYZRefiner.hpp"
#include "armor_detector/pose/PoseOnlyBa4DofYPDRefiner.hpp"
#include "armor_detector/pose/PoseOnlyBa6DofRefiner.hpp"
#include "armor_detector/pose/PoseRefineData.hpp"
#include "armor_detector/pose/YawSearchRefiner.hpp"
#include "armor_detector/pose/YawSearchThenDistanceRefiner.hpp"

#include <memory>
#include <string>

namespace armor_detector::pose {

    class PoseRefineRunner : public IPoseRefiner {
    public:
        void setSingleMethod(SinglePoseRefineMethod method);
        SinglePoseRefineMethod singleMethod() const;
        void setDualMethod(DualPoseRefineMethod method);
        DualPoseRefineMethod dualMethod() const;
        std::string methodName() const;

        PoseRefineBatchOutput refine(const std::vector<PoseRefineInput> &inputs) const override;

        double calculateInitialError(const PoseRefineInput &input) const;

    private:
        const IPoseRefiner &singleRefiner() const;

        SinglePoseRefineMethod single_method_ = SinglePoseRefineMethod::YAW_SEARCH_THEN_DISTANCE;
        DualPoseRefineMethod dual_method_ = DualPoseRefineMethod::NONE;
        NoneRefiner none_refiner_;
        YawSearchRefiner yaw_search_refiner_;
        YawSearchThenDistanceRefiner yaw_search_then_distance_refiner_;
        PoseOnlyBa6DofRefiner pose_only_ba_6dof_refiner_;
        PoseOnlyBa4DofXYZRefiner pose_only_ba_4dof_xyz_refiner_;
        PoseOnlyBa4DofYPDRefiner pose_only_ba_4dof_ypd_refiner_;
        DualArmorBa3DofYPDRefiner dual_armor_ba_3dof_ypd_refiner_;
    };

} // namespace armor_detector::pose
