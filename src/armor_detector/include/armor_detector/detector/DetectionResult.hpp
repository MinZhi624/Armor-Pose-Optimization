#pragma once

#include "armor_detector/debug/DetectionDebugData.hpp"
#include "armor_detector/types/ArmorData.hpp"

#include <vector>

namespace armor_detector {

    struct DetectionResult {
        std::vector<ClassifiedArmor> armors;
        debug::DetectionDebugData debug;
    };

} // namespace armor_detector
