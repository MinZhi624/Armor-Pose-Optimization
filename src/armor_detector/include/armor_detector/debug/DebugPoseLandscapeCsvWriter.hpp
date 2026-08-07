#pragma once

#include <filesystem>
#include <fstream>
#include <string>

#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug {

    /**
     * @brief 将每个 pose landscape 样本写入独立、可复现的实验目录。
     */
    class DebugPoseLandscapeCsvWriter : public IDebugObserver {
    public:
        DebugPoseLandscapeCsvWriter(const std::string &root_dir, const std::string &video);
        ~DebugPoseLandscapeCsvWriter() override = default;

        void onPoseSolved(DebugFrameContext &context, const PoseDebugData &data) override;

    private:
        void writeIndexHeader();
        bool writeSample(const DebugFrameContext &context, const PoseLandscapeSample &sample);

        std::filesystem::path run_root_;
        std::ofstream index_file_;
    };

} // namespace armor_detector::debug
