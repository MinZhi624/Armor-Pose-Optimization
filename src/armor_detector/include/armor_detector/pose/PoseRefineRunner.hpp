#pragma once

#include "armor_detector/pose/IPoseRefiner.hpp"
#include "armor_detector/pose/PoseRefineData.hpp"
#include "armor_detector/pose/NoneRefiner.hpp"
#include "armor_detector/pose/YawSearchRefiner.hpp"

#include <memory>
#include <string>

namespace armor_detector::pose {

    class PoseRefineRunner {
    public:
        void setMethod(PoseRefineMethod method);
        PoseRefineMethod method() const;
        std::string methodName() const;

        PoseRefineOutput refine(const PoseRefineInput &input,
                                const PoseErrorFunction &calculate_error) const;

        double calculateInitialError(const PoseRefineInput &input,
                                    const PoseErrorFunction &calculate_error) const;

    private:
        PoseRefineMethod method_ = PoseRefineMethod::YAW_SEARCH;
        NoneRefiner none_refiner_;
        YawSearchRefiner yaw_search_refiner_;
    };

} // namespace armor_detector::pose
