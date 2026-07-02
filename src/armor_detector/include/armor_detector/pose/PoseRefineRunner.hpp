#pragma once

#include "armor_detector/pose/IPoseRefiner.hpp"
#include "armor_detector/pose/NoneRefiner.hpp"
#include "armor_detector/pose/PoseOnlyBa4DofXYZRefiner.hpp"
#include "armor_detector/pose/PoseOnlyBa4DofYPDRefiner.hpp"
#include "armor_detector/pose/PoseOnlyBa6DofRefiner.hpp"
#include "armor_detector/pose/PoseRefineData.hpp"
#include "armor_detector/pose/YawSearchRefiner.hpp"

#include <memory>
#include <string>

namespace armor_detector::pose {

    class PoseRefineRunner {
    public:
        void setMethod(PoseRefineMethod method);
        PoseRefineMethod method() const;
        std::string methodName() const;

        PoseRefineOutput refine(const PoseRefineInput &input) const;

        double calculateInitialError(const PoseRefineInput &input) const;

    private:
        PoseRefineMethod method_ = PoseRefineMethod::POSE_ONLY_BA_4DOF_XYZ;
        NoneRefiner none_refiner_;
        YawSearchRefiner yaw_search_refiner_;
        PoseOnlyBa6DofRefiner pose_only_ba_6dof_refiner_;
        PoseOnlyBa4DofXYZRefiner pose_only_ba_4dof_xyz_refiner_;
        PoseOnlyBa4DofYPDRefiner pose_only_ba_4dof_ypd_refiner_;
    };

} // namespace armor_detector::pose
