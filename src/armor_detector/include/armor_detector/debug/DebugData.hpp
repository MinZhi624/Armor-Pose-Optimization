#pragma once

#include "armor_detector/types/ArmorData.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <opencv2/core.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace armor_detector::debug
{

/**
 * @brief 单帧 debug 上下文。
 *
 * source_bgr 只作为观察输入使用，Observer 不应反向修改主流程图像。
 * cv::Mat 生命周期与 clone 规则统一遵守 docs/Conventions.md。
 */
struct DebugFrameContext
{
    std::size_t frame_index = 0;
    builtin_interfaces::msg::Time stamp;
    cv::Mat source_bgr;
};

/**
 * @brief 按键事件。DebugKeyHandler 负责从 cv::waitKey 的 raw key 转换得到。
 */
enum class DebugKeyAction {
    NONE,
    EXIT,
    PAUSE_TOGGLE,
    SAVE_ROI,
    STEP_FRAME,
};

struct DebugKeyEvent
{
    DebugKeyAction action = DebugKeyAction::NONE;
    int raw_key = -1;
};

/**
 * @brief 阶段耗时记录。
 */
struct StageTiming
{
    std::string name;
    double elapsed_ms = 0.0;
};

/**
 * @brief 预处理阶段中间图像。
 */
struct PreprocessDebugData
{
    cv::Mat gray;
    cv::Mat binary;
    cv::Mat color_mask;
};

enum class DebugRejectReason {
    UNKNOWN,
    TOO_SMALL,
    TOO_LARGE,
    BAD_RATIO,
    BAD_ANGLE,
    BAD_COLOR,
    LOW_CONFIDENCE,
    PNP_FAILED,
};

struct RejectedLight
{
    LightBar light;
    DebugRejectReason reason = DebugRejectReason::UNKNOWN;
    std::string detail;
};

struct LightDebugData
{
    std::vector<LightBar> accepted_lights;
    std::vector<RejectedLight> rejected_lights;
};

struct RejectedArmor
{
    ArmorGeometry geometry;
    DebugRejectReason reason = DebugRejectReason::UNKNOWN;
    std::string detail;
};

struct ArmorMatchDebugData
{
    std::vector<ArmorCandidate> candidates;
    std::vector<RejectedArmor> rejected_armors;
};

struct ClassificationDebugData
{
    std::vector<ClassifiedArmor> classified_armors;
    std::vector<cv::Mat> number_rois;
};

struct PoseDebugData
{
    std::vector<SolvedArmor> solved_armors;
};

} // namespace armor_detector::debug
