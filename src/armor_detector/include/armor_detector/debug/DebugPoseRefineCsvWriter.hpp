#pragma once

#include "armor_detector/debug/IDebugObserver.hpp"

#include <filesystem>
#include <fstream>
#include <string>

namespace armor_detector::debug {

    class DebugPoseRefineCsvWriter : public IDebugObserver {
    public:
        DebugPoseRefineCsvWriter(const std::string &root_dir,
                                 const std::string &video,
                                 const std::string &method);
        ~DebugPoseRefineCsvWriter() override = default;

        void onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) override;

    private:
        void writeHeader();
        void ensureDir();

        std::filesystem::path csv_path_;
        std::ofstream file_;
    };

} // namespace armor_detector::debug
