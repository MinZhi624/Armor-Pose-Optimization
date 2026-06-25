# Debug Layer System Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a runtime-switchable debug layer system (number keys 1–5) with always-on timing overlay, thread-safe layer state, and DetectorNode refactor into .hpp/.cpp.

**Architecture:** Observer-based debug pipeline where `DebugLayerState` (thread-safe map) gates which views render. `DebugTiming` always draws timing text on `display_bgr` before it is submitted to `DebugGUI`. `DebugLayerController` handles number-key toggles. DetectorNode is split into header + implementation with extracted init methods.

**Tech Stack:** C++17, ROS 2 (rclcpp), OpenCV 4, ament_cmake

---

## File Structure

### Files to Create

| File | Responsibility |
|---|---|
| `include/armor_detector/debug/DebugLayerState.hpp` | Thread-safe layer enable/disable state table (header-only) |
| `include/armor_detector/debug/DebugLayerController.hpp` | Layer toggle key handler observer — header |
| `src/debug/DebugLayerController.cpp` | Layer toggle key handler observer — implementation |
| `src/debug/DebugTiming.cpp` | Frame/stage timing overlay on display_bgr + periodic stats log |

### Files to Modify

| File | Changes |
|---|---|
| `include/armor_detector/debug/DebugData.hpp` | Add `DebugLayer` enum, `TOGGLE_LAYER` action, `layer` field in `DebugKeyEvent` |
| `src/debug/DebugKeyHandler.cpp` | Map keys `'1'`–`'5'` to `TOGGLE_LAYER` with corresponding `DebugLayer` |
| `include/armor_detector/debug/DebugGUI.hpp` | Add `clearFrame()`, `pending_destroy_` queue, `destroy_mutex_` |
| `src/debug/DebugGUI.cpp` | Implement `clearFrame()`, process pending window destroys in `loop()` |
| `include/armor_detector/debug/DebugPreprocess.hpp` | Accept `DebugLayerState&` in constructor |
| `src/debug/DebugPreprocessView.cpp` | Gate rendering on `PREPROCESS` layer, clear stale frame on disable |
| `include/armor_detector/debug/DebugLight.hpp` | Accept `DebugLayerState&` in constructor |
| `src/debug/DebugLightView.cpp` | Gate rendering on `LIGHTS` layer |
| `include/armor_detector/DetectorNode.hpp` | Full class declaration with `DebugConfig`, init method signatures |
| `src/DetectorNode.cpp` | Refactored: include header, init methods, updated `run()`/`pollDebugKeys()` |
| `config/config.yaml` | Add `debug:` config section |
| `CMakeLists.txt` | Add `DebugTiming.cpp`, `DebugLayerController.cpp` |
| `docs/Debug.md` | Document `debug.*` config, number key mapping, timing always-on |
| `docs/DebugGUI.md` | Register timing text lines, lights draw elements |

---

### Task 1: Extend DebugData.hpp with DebugLayer and TOGGLE_LAYER

**Files:**
- Modify: `src/armor_detector/include/armor_detector/debug/DebugData.hpp`

- [ ] **Step 1: Add DebugLayer enum**

Add the following enum **before** the existing `DebugKeyAction` enum (after the `DebugFrameContext` struct, around line 27):

```cpp
/**
 * @brief 可切换的 debug 图层标识。
 *
 * 用于 DebugLayerState 和数字键 (1-5) 切换。
 * timing 不是图层——它在 debug.show=true 时始终启用。
 */
enum class DebugLayer {
    UNKNOWN,
    PREPROCESS,
    LIGHTS,
    ARMOR_MATCH,
    CLASSIFICATION,
    POSE,
};
```

- [ ] **Step 2: Add TOGGLE_LAYER to DebugKeyAction**

In the existing `DebugKeyAction` enum, add `TOGGLE_LAYER` after `PLAYBACK_RATE_DOWN`:

```cpp
enum class DebugKeyAction {
    NONE,
    EXIT,
    PAUSE_TOGGLE,
    SAVE_ROI,
    STEP_FRAME,
    PLAYBACK_RATE_UP,
    PLAYBACK_RATE_DOWN,
    TOGGLE_LAYER,   // 新增：数字键切换图层
};
```

- [ ] **Step 3: Add layer field to DebugKeyEvent**

Modify the existing `DebugKeyEvent` struct to include a `layer` field:

```cpp
struct DebugKeyEvent
{
    DebugKeyAction action = DebugKeyAction::NONE;
    int raw_key = -1;
    DebugLayer layer = DebugLayer::UNKNOWN;  // 新增：TOGGLE_LAYER 时的目标图层
};
```

- [ ] **Step 4: Build to verify**

Run: `colcon build --packages-select armor_detector`
Expected: Build succeeds. No existing code references `TOGGLE_LAYER` yet, so no errors.

- [ ] **Step 5: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugData.hpp
git commit -m "feat: add DebugLayer enum and TOGGLE_LAYER action"
```

---

### Task 2: Update DebugKeyHandler for Number Key Mapping

**Files:**
- Modify: `src/armor_detector/src/debug/DebugKeyHandler.cpp`

- [ ] **Step 1: Add number key mappings**

In `DebugKeyHandler::translate()`, add an `else if` block **before** the `return` statement (after the `'-'/'_'` block, around line 34). The full method becomes:

```cpp
DebugKeyEvent DebugKeyHandler::translate(int raw_key) const
{
    DebugKeyAction action = DebugKeyAction::NONE;
    DebugLayer layer = DebugLayer::UNKNOWN;

    if (raw_key == 27 || raw_key == 'q' || raw_key == 'Q')
    {
        action = DebugKeyAction::EXIT;
    }
    else if (raw_key == ' ' || raw_key == 'p' || raw_key == 'P')
    {
        action = DebugKeyAction::PAUSE_TOGGLE;
    }
    else if (raw_key == 'n' || raw_key == 'N' || raw_key == 65363 || raw_key == 2555904 ||
             raw_key == 0x270000)
    {
        action = DebugKeyAction::STEP_FRAME;
    }
    else if (raw_key == 's' || raw_key == 'S')
    {
        action = DebugKeyAction::SAVE_ROI;
    }
    else if (raw_key == '+' || raw_key == '=')
    {
        action = DebugKeyAction::PLAYBACK_RATE_UP;
    }
    else if (raw_key == '-' || raw_key == '_')
    {
        action = DebugKeyAction::PLAYBACK_RATE_DOWN;
    }
    else if (raw_key == '1')
    {
        action = DebugKeyAction::TOGGLE_LAYER;
        layer = DebugLayer::PREPROCESS;
    }
    else if (raw_key == '2')
    {
        action = DebugKeyAction::TOGGLE_LAYER;
        layer = DebugLayer::LIGHTS;
    }
    else if (raw_key == '3')
    {
        action = DebugKeyAction::TOGGLE_LAYER;
        layer = DebugLayer::ARMOR_MATCH;
    }
    else if (raw_key == '4')
    {
        action = DebugKeyAction::TOGGLE_LAYER;
        layer = DebugLayer::CLASSIFICATION;
    }
    else if (raw_key == '5')
    {
        action = DebugKeyAction::TOGGLE_LAYER;
        layer = DebugLayer::POSE;
    }

    return {action, raw_key, layer};
}
```

**Note:** The return statement changes from `{action, raw_key}` to `{action, raw_key, layer}` to include the new `layer` field in `DebugKeyEvent`. The aggregate initialization order matches the struct field order: `action`, `raw_key`, `layer`.

- [ ] **Step 2: Build to verify**

Run: `colcon build --packages-select armor_detector`
Expected: Build succeeds. `DebugKeyEvent` aggregate init with 3 fields compiles because we added `layer` in Task 1.

- [ ] **Step 3: Commit**

```bash
git add src/armor_detector/src/debug/DebugKeyHandler.cpp
git commit -m "feat: map number keys 1-5 to TOGGLE_LAYER"
```

---

### Task 3: Create DebugLayerState (Header-Only)

**Files:**
- Create: `src/armor_detector/include/armor_detector/debug/DebugLayerState.hpp`

- [ ] **Step 1: Write the complete header**

```cpp
#pragma once

#include "armor_detector/debug/DebugData.hpp"

#include <mutex>
#include <string>
#include <unordered_map>

namespace armor_detector::debug
{

/**
 * @brief 线程安全的 debug 图层开关状态表。
 *
 * DebugGUI 在独立线程运行，数字键切换和 Observer 绘制可能并发读写。
 * 所有公开方法均通过 std::mutex 保护内部 map。
 *
 * 使用方式：
 *   1. DetectorNode 初始化时从 config 设置各图层初始状态。
 *   2. DebugLayerController 在 onKey 中 toggle 状态。
 *   3. DebugPreprocessView / DebugLightView 在 onXxx 中查询状态。
 */
class DebugLayerState
{
public:
    void setEnabled(DebugLayer layer, bool enabled)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        states_[layer] = enabled;
    }

    bool enabled(DebugLayer layer) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = states_.find(layer);
        return it != states_.end() && it->second;
    }

    /**
     * @brief 切换指定图层的开关状态，返回切换后的新值。
     */
    bool toggle(DebugLayer layer)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bool new_state = !states_[layer];
        states_[layer] = new_state;
        return new_state;
    }

    /**
     * @brief 返回图层的可读名称，用于日志输出。
     */
    static std::string name(DebugLayer layer)
    {
        switch (layer) {
        case DebugLayer::PREPROCESS:
            return "preprocess";
        case DebugLayer::LIGHTS:
            return "lights";
        case DebugLayer::ARMOR_MATCH:
            return "armor_match";
        case DebugLayer::CLASSIFICATION:
            return "classification";
        case DebugLayer::POSE:
            return "pose";
        default:
            return "unknown";
        }
    }

private:
    mutable std::mutex mutex_;
    std::unordered_map<DebugLayer, bool> states_;
};

} // namespace armor_detector::debug
```

**Design notes:**
- `mutex_` is `mutable` so `enabled()` (const) can lock it.
- `name()` is `static` — it only depends on the enum value, no state.
- `toggle()` uses `operator[]` which default-initializes missing keys to `false`, then negates. So toggling an uninitialized layer goes `false → true`, which is the correct "first press enables" behavior.

- [ ] **Step 2: Build to verify**

Run: `colcon build --packages-select armor_detector`
Expected: Build succeeds. Header-only, not included by anything yet.

- [ ] **Step 3: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugLayerState.hpp
git commit -m "feat: add thread-safe DebugLayerState"
```

---

### Task 4: Add clearFrame() to DebugGUI

**Files:**
- Modify: `src/armor_detector/include/armor_detector/debug/DebugGUI.hpp`
- Modify: `src/armor_detector/src/debug/DebugGUI.cpp`

- [ ] **Step 1: Add declarations to DebugGUI.hpp**

Add a `#include <vector>` if not already present (it is — `std::vector` is in the return type of `takeKeyEvents`). Add two new private members and one new public method.

**Public section** — add after `takeKeyEvents()` declaration (after line 42):

```cpp
    // 线程安全: 从 frames_ 移除同名窗口帧，并请求 GUI 线程销毁窗口
    void clearFrame(const std::string & window_name);
```

**Private section** — add after `key_handler_` (after line 60):

```cpp
    // 待销毁窗口列表，由 clearFrame() 写入，loop() 中 GUI 线程消费
    std::mutex destroy_mutex_;
    std::vector<std::string> pending_destroy_;
```

- [ ] **Step 2: Implement clearFrame() and update loop() in DebugGUI.cpp**

**Add `clearFrame()` implementation** after `takeKeyEvents()` (after line 88):

```cpp
void DebugGUI::clearFrame(const std::string & window_name)
{
    {
        std::lock_guard<std::mutex> lock(frames_mutex_);
        frames_.erase(window_name);
    }
    {
        std::lock_guard<std::mutex> lock(destroy_mutex_);
        pending_destroy_.push_back(window_name);
    }
}
```

**Modify `loop()`** — add window destruction at the **beginning** of the while loop body (before the snapshot, before line 58):

```cpp
void DebugGUI::loop()
{
    while (running_.load()) {
        // 销毁被 clearFrame 标记的窗口
        {
            std::lock_guard<std::mutex> lock(destroy_mutex_);
            for (const auto & name : pending_destroy_) {
                cv::destroyWindow(name);
            }
            pending_destroy_.clear();
        }

        std::unordered_map<std::string, DebugWindowFrame> snapshot;
        // ... rest of existing loop body unchanged
```

**Why GUI thread destroys windows:** OpenCV `destroyWindow` should be called from the thread that created the window (the GUI thread) to avoid platform-specific issues.

- [ ] **Step 3: Build to verify**

Run: `colcon build --packages-select armor_detector`
Expected: Build succeeds.

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugGUI.hpp \
        src/armor_detector/src/debug/DebugGUI.cpp
git commit -m "feat: add DebugGUI::clearFrame for stale window cleanup"
```

---

### Task 5: Create DebugLayerController Observer

**Files:**
- Create: `src/armor_detector/include/armor_detector/debug/DebugLayerController.hpp`
- Create: `src/armor_detector/src/debug/DebugLayerController.cpp`

- [ ] **Step 1: Write the header**

```cpp
#pragma once

#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 处理数字键图层切换的 Observer。
 *
 * 只关心 onKey 事件。收到 TOGGLE_LAYER 时切换 DebugLayerState 并打印日志。
 * 不负责图层绘制——绘制由各 View Observer 根据 layer state 自行判断。
 */
class DebugLayerController : public IDebugObserver
{
public:
    explicit DebugLayerController(DebugLayerState & layer_state);

    void onKey(const DebugKeyEvent & event) override;

private:
    DebugLayerState & layer_state_;
};

} // namespace armor_detector::debug
```

- [ ] **Step 2: Write the implementation**

```cpp
#include "armor_detector/debug/DebugLayerController.hpp"

#include <rclcpp/rclcpp.hpp>

namespace armor_detector::debug
{

DebugLayerController::DebugLayerController(DebugLayerState & layer_state)
    : layer_state_(layer_state)
{
}

void DebugLayerController::onKey(const DebugKeyEvent & event)
{
    if (event.action != DebugKeyAction::TOGGLE_LAYER) {
        return;
    }
    if (event.layer == DebugLayer::UNKNOWN) {
        return;
    }

    bool new_state = layer_state_.toggle(event.layer);
    RCLCPP_INFO(rclcpp::get_logger("DEBUG"),
                "layer %s: %s",
                DebugLayerState::name(event.layer).c_str(),
                new_state ? "ON" : "OFF");
}

} // namespace armor_detector::debug
```

**Note:** `layer_state_.toggle()` returns the new state (true = ON, false = OFF). We use `rclcpp::get_logger("DEBUG")` since this observer doesn't inherit from `rclcpp::Node`.

- [ ] **Step 3: Build to verify**

Run: `colcon build --packages-select armor_detector`
Expected: Build succeeds. The new files aren't in CMakeLists.txt yet, but the headers compile when included.

**Note:** The `.cpp` won't be compiled until Task 11 (CMakeLists.txt update). This is expected. The header is standalone and can be verified by inclusion.

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugLayerController.hpp \
        src/armor_detector/src/debug/DebugLayerController.cpp
git commit -m "feat: add DebugLayerController for number key layer toggling"
```

---

### Task 6: Implement DebugTiming Observer

**Files:**
- Create: `src/armor_detector/src/debug/DebugTiming.cpp`

The header already exists at `include/armor_detector/debug/DebugTiming.hpp` (see existing code). It declares:
- Constructor: `explicit DebugTiming(std::size_t report_interval = 50)`
- Overrides: `onFrameStart`, `onPreprocess`, `onLights`, `onArmorMatch`, `onClassification`, `onPoseSolved`, `onFrameEnd`
- Private: `mark(stage_name)`, `report_interval_`, `frame_count_`, `frame_start_`, `last_mark_`, `stage_time_sums_`, `frame_time_sum_ms_`

We need to add **current-frame stage times** for drawing. Modify the header first.

- [ ] **Step 1: Add current-frame storage to DebugTiming.hpp**

In `include/armor_detector/debug/DebugTiming.hpp`, add these private members after `frame_time_sum_ms_` (after line 39):

```cpp
    // 当前帧的阶段耗时（用于 onFrameEnd 绘制）
    double last_frame_ms_ = 0.0;
    std::vector<std::pair<std::string, double>> last_stage_ms_;
```

Add `#include <utility>` and `#include <vector>` to the includes (after `#include <map>`).

The full header after edits:

```cpp
#pragma once

#include "armor_detector/debug/IDebugObserver.hpp"

#include <chrono>
#include <cstddef>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace armor_detector::debug
{

/**
 * @brief 阶段耗时统计 Observer。
 *
 * 每帧统计总耗时和各阶段耗时，在 display_bgr 左上角绘制。
 * 每 report_interval 帧打印一次平均耗时日志。
 * timing 不参与图层切换——debug.show=true 时始终启用。
 */
class DebugTiming : public IDebugObserver
{
public:
    explicit DebugTiming(std::size_t report_interval = 50);

    void onFrameStart(DebugFrameContext & context) override;
    void onPreprocess(DebugFrameContext & context, const PreprocessDebugData & data) override;
    void onLights(DebugFrameContext & context, const LightDebugData & data) override;
    void onArmorMatch(DebugFrameContext & context, const ArmorMatchDebugData & data) override;
    void onClassification(DebugFrameContext & context, const ClassificationDebugData & data) override;
    void onPoseSolved(DebugFrameContext & context, const PoseDebugData & data) override;
    void onFrameEnd(DebugFrameContext & context) override;

private:
    void mark(const std::string & stage_name);

    std::size_t report_interval_ = 50;
    std::size_t frame_count_ = 0;
    std::chrono::steady_clock::time_point frame_start_;
    std::chrono::steady_clock::time_point last_mark_;
    std::map<std::string, double> stage_time_sums_;
    double frame_time_sum_ms_ = 0.0;

    // 当前帧的阶段耗时（用于 onFrameEnd 绘制文字）
    double last_frame_ms_ = 0.0;
    std::vector<std::pair<std::string, double>> last_stage_ms_;
};

} // namespace armor_detector::debug
```

- [ ] **Step 2: Write DebugTiming.cpp**

```cpp
#include "armor_detector/debug/DebugTiming.hpp"

#include <rclcpp/rclcpp.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>

namespace armor_detector::debug
{

DebugTiming::DebugTiming(std::size_t report_interval)
    : report_interval_(report_interval > 0 ? report_interval : 50)
{
}

void DebugTiming::onFrameStart(DebugFrameContext & /*context*/)
{
    ++frame_count_;
    frame_start_ = std::chrono::steady_clock::now();
    last_mark_ = frame_start_;
    last_stage_ms_.clear();
}

void DebugTiming::mark(const std::string & stage_name)
{
    auto now = std::chrono::steady_clock::now();
    double dt_ms = std::chrono::duration<double, std::milli>(now - last_mark_).count();
    stage_time_sums_[stage_name] += dt_ms;
    last_stage_ms_.emplace_back(stage_name, dt_ms);
    last_mark_ = now;
}

void DebugTiming::onPreprocess(DebugFrameContext & /*context*/,
                                const PreprocessDebugData & /*data*/)
{
    mark("preprocess");
}

void DebugTiming::onLights(DebugFrameContext & /*context*/,
                            const LightDebugData & /*data*/)
{
    mark("lights");
}

void DebugTiming::onArmorMatch(DebugFrameContext & /*context*/,
                                const ArmorMatchDebugData & /*data*/)
{
    mark("armor_match");
}

void DebugTiming::onClassification(DebugFrameContext & /*context*/,
                                    const ClassificationDebugData & /*data*/)
{
    mark("classification");
}

void DebugTiming::onPoseSolved(DebugFrameContext & /*context*/,
                                const PoseDebugData & /*data*/)
{
    mark("pose");
}

void DebugTiming::onFrameEnd(DebugFrameContext & context)
{
    auto now = std::chrono::steady_clock::now();
    last_frame_ms_ = std::chrono::duration<double, std::milli>(now - frame_start_).count();
    frame_time_sum_ms_ += last_frame_ms_;

    // ---- 绘制 timing 文字到 display_bgr ----
    if (context.display_bgr.empty()) {
        return;
    }

    auto drawText = [&](const std::string & text, int line_index) {
        int y = 24 * line_index;  // line 1 = y:24, line 2 = y:48, ...
        cv::putText(context.display_bgr, text, cv::Point(10, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    };

    // 格式化: "Process: xx.xx ms"
    std::ostringstream total_ss;
    total_ss << std::fixed << std::setprecision(2) << last_frame_ms_;
    drawText("Process: " + total_ss.str() + " ms", 1);

    // 各阶段
    int line = 2;
    for (const auto & [name, ms] : last_stage_ms_) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << ms;
        drawText(name + ": " + ss.str() + " ms", line);
        ++line;
    }

    // ---- 每 report_interval 帧打印平均耗时 ----
    if (frame_count_ > 0 && frame_count_ % report_interval_ == 0) {
        std::ostringstream marks_ss;
        for (const auto & [name, sum] : stage_time_sums_) {
            double avg = sum / static_cast<double>(report_interval_);
            marks_ss << name << ":" << std::fixed << std::setprecision(1) << avg << "ms ";
        }
        std::string marks_str = marks_ss.str();
        if (!marks_str.empty()) {
            marks_str.pop_back();  // 移除末尾空格
        }

        double avg_total = frame_time_sum_ms_ / static_cast<double>(report_interval_);
        RCLCPP_INFO(rclcpp::get_logger("DEBUG_TIMING"),
                    "【平均用时】总:%.1fms [%s]",
                    avg_total, marks_str.c_str());

        // 重置累计
        frame_time_sum_ms_ = 0.0;
        for (auto & [k, v] : stage_time_sums_) {
            v = 0.0;
        }
    }
}

} // namespace armor_detector::debug
```

**Key design decisions:**
- Text is drawn at TOP_LEFT following DebugGUI.md conventions: `FONT_HERSHEY_SIMPLEX`, scale `0.60`, thickness `2`, `LINE_AA`, 24px line height starting at y=24.
- `mark()` records inter-mark interval (not cumulative). Stage times don't sum to total — the gaps represent overhead between stages.
- Stats print uses `rclcpp::get_logger("DEBUG_TIMING")` for filtering.
- `report_interval <= 0` is guarded in the constructor to avoid division by zero.

- [ ] **Step 3: Build to verify (header-only check)**

Run: `colcon build --packages-select armor_detector`
Expected: Build succeeds. The `.cpp` won't compile into the binary until Task 11, but no syntax errors in the header changes.

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugTiming.hpp \
        src/armor_detector/src/debug/DebugTiming.cpp
git commit -m "feat: implement DebugTiming with on-screen overlay and stats logging"
```

---

### Task 7: Gate DebugPreprocessView with Layer State

**Files:**
- Modify: `src/armor_detector/include/armor_detector/debug/DebugPreprocess.hpp`
- Modify: `src/armor_detector/src/debug/DebugPreprocessView.cpp`

- [ ] **Step 1: Update the header**

Replace the full content of `include/armor_detector/debug/DebugPreprocess.hpp`:

```cpp
#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 预处理中间结果显示 Observer。
 *
 * 把 gray / binary / color_mask 等中间图拼成窗口图像，提交给 DebugGUI。
 * 通过 DebugLayerState 控制是否渲染。关闭时清理旧的 preprocess_debug 窗口。
 */
class DebugPreprocessView : public IDebugObserver
{
public:
    DebugPreprocessView(DebugGUI & gui, DebugLayerState & layer_state);

    void onPreprocess(DebugFrameContext & context, const PreprocessDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
    DebugLayerState & layer_state_;
    bool was_shown_ = false;  // 用于在关闭时只清理一次窗口
};

} // namespace armor_detector::debug
```

- [ ] **Step 2: Update the implementation**

Replace the full content of `src/armor_detector/src/debug/DebugPreprocessView.cpp`:

```cpp
#include "armor_detector/debug/DebugPreprocess.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug
{

DebugPreprocessView::DebugPreprocessView(DebugGUI & gui, DebugLayerState & layer_state)
    : gui_(&gui), layer_state_(layer_state)
{
}

void DebugPreprocessView::onPreprocess(
    DebugFrameContext & context,
    const PreprocessDebugData & data)
{
    if (!gui_ || !gui_->enabled()) return;

    // 图层关闭：清理旧窗口后返回
    if (!layer_state_.enabled(DebugLayer::PREPROCESS)) {
        if (was_shown_) {
            gui_->clearFrame("preprocess_debug");
            was_shown_ = false;
        }
        return;
    }

    if (context.source_bgr.empty()) return;

    was_shown_ = true;

    // 原图 resize 0.5
    cv::Mat src_half;
    cv::resize(context.source_bgr, src_half, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

    // 三张单通道图转 BGR 后 resize 0.5
    auto toHalfBgr = [](const cv::Mat & src) -> cv::Mat {
        if (src.empty()) return {};
        cv::Mat bgr, half;
        cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
        cv::resize(bgr, half, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
        return half;
    };

    cv::Mat gray_half = toHalfBgr(data.gray);
    cv::Mat binary_half = toHalfBgr(data.img_thre);
    cv::Mat color_half = toHalfBgr(data.color_mask);

    // 空图用黑图替代，保证 2x2 网格尺寸一致
    cv::Mat black = cv::Mat::zeros(src_half.size(), CV_8UC3);
    if (gray_half.empty()) gray_half = black.clone();
    if (binary_half.empty()) binary_half = black.clone();
    if (color_half.empty()) color_half = black.clone();

    // 2x2 网格: [source | gray] [binary | color_mask]
    cv::Mat top, bottom, combined;
    cv::hconcat(src_half, gray_half, top);
    cv::hconcat(binary_half, color_half, bottom);
    cv::vconcat(top, bottom, combined);

    // 文字标注（遵循 DebugGUI.md: FONT_HERSHEY_SIMPLEX, scale 0.60, thickness 2）
    int w = src_half.cols;
    int h = src_half.rows;
    cv::putText(combined, "source", cv::Point(10, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    cv::putText(combined, "gray", cv::Point(w + 10, 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    cv::putText(combined, "binary", cv::Point(10, h + 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
    cv::putText(combined, "color_mask", cv::Point(w + 10, h + 24),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 165, 255), 2, cv::LINE_AA);

    gui_->setFrame("preprocess_debug", combined);
}

} // namespace armor_detector::debug
```

**Key changes from original:**
1. Constructor takes `DebugLayerState &` in addition to `DebugGUI &`.
2. After the `gui_->enabled()` check, a layer-state gate returns early if `PREPROCESS` is disabled.
3. When transitioning from shown → hidden (`was_shown_` true → layer off), calls `gui_->clearFrame("preprocess_debug")` to destroy the stale OpenCV window.
4. `was_shown_` prevents calling `clearFrame` every frame when the layer is off.

- [ ] **Step 3: Build to verify**

Run: `colcon build --packages-select armor_detector`
Expected: Build may fail because DetectorNode.cpp still constructs `DebugPreprocessView` with the old signature `(debug_gui_)`. This will be fixed in Task 10. For now, verify that the new header/impl files are syntactically correct.

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugPreprocess.hpp \
        src/armor_detector/src/debug/DebugPreprocessView.cpp
git commit -m "feat: gate DebugPreprocessView with DebugLayerState"
```

---

### Task 8: Gate DebugLightView with Layer State

**Files:**
- Modify: `src/armor_detector/include/armor_detector/debug/DebugLight.hpp`
- Modify: `src/armor_detector/src/debug/DebugLightView.cpp`

- [ ] **Step 1: Update the header**

Replace the full content of `include/armor_detector/debug/DebugLight.hpp`:

```cpp
#pragma once

#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"
#include "armor_detector/debug/IDebugObserver.hpp"

namespace armor_detector::debug
{

/**
 * @brief 灯条检测可视化 Observer。
 *
 * 绘制 accepted lights、rejected lights 和拒绝原因。
 * 通过 DebugLayerState 控制是否渲染。lights 绘制在 display_bgr 上（叠加到主窗口），
 * 关闭时不生成叠加内容即可，无需清理窗口。
 */
class DebugLightView : public IDebugObserver
{
public:
    DebugLightView(DebugGUI & gui, DebugLayerState & layer_state);

    void onLights(DebugFrameContext & context, const LightDebugData & data) override;

private:
    DebugGUI * gui_ = nullptr;
    DebugLayerState & layer_state_;
};

} // namespace armor_detector::debug
```

- [ ] **Step 2: Update the implementation**

Replace the full content of `src/armor_detector/src/debug/DebugLightView.cpp`:

```cpp
#include "armor_detector/debug/DebugLight.hpp"

#include <opencv2/imgproc.hpp>

namespace armor_detector::debug
{

DebugLightView::DebugLightView(DebugGUI & gui, DebugLayerState & layer_state)
    : gui_(&gui), layer_state_(layer_state)
{
}

void DebugLightView::onLights(
    DebugFrameContext & context,
    const LightDebugData & data)
{
    if (!gui_ || !gui_->enabled()) return;
    if (!layer_state_.enabled(DebugLayer::LIGHTS)) return;
    if (context.display_bgr.empty()) return;

    cv::Mat & display = context.display_bgr;

    // 绘制 accepted lights：绿色 rotatedRect + 端点
    for (const auto & light : data.accepted_lights) {
        cv::Point2f vertices[4];
        light.rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(display, vertices[i], vertices[(i + 1) % 4],
                     cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        }
        cv::circle(display, light.top, 1, cv::Scalar(255, 0, 255), -1);
        cv::circle(display, light.bottom, 1, cv::Scalar(255, 0, 255), -1);
    }

    // 绘制 rejected lights：红色 + 原因
    for (const auto & rejected : data.rejected_lights) {
        cv::Point2f vertices[4];
        rejected.light.rect.points(vertices);
        for (int i = 0; i < 4; i++) {
            cv::line(display, vertices[i], vertices[(i + 1) % 4],
                     cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        cv::putText(display, rejected.detail,
                    cv::Point(rejected.light.center.x + 5, rejected.light.center.y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.60, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
}

} // namespace armor_detector::debug
```

**Key changes from original:**
1. Constructor takes `DebugLayerState &`.
2. One new early-return check: `if (!layer_state_.enabled(DebugLayer::LIGHTS)) return;`
3. No `was_shown_` tracking needed — lights draws on `display_bgr` which is re-created each frame. When lights is disabled, the overlay simply doesn't happen.

- [ ] **Step 3: Build to verify**

Same as Task 7 — build will temporarily fail due to constructor signature mismatch in DetectorNode.cpp. Fixed in Task 10.

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/include/armor_detector/debug/DebugLight.hpp \
        src/armor_detector/src/debug/DebugLightView.cpp
git commit -m "feat: gate DebugLightView with DebugLayerState"
```

---

### Task 9: Update config.yaml with Debug Section

**Files:**
- Modify: `src/armor_detector/config/config.yaml`

- [ ] **Step 1: Replace config.yaml content**

Write the full new content:

```yaml
armor_detector_node_cpp:
  ros__parameters:
    # 目标颜色
    target_color: "BLUE"

    # 预处理
    gray_threshold: 100
    color_threshold: 100

    # 灯条
    min_contours_area: 30
    min_contours_ratio: 0.06
    max_contours_ratio: 0.5

    # Debug
    debug:
      show: true
      rosbag_control: true
      rosbag_player_node: "/rosbag2_player"
      preprocess: false
      lights: true
      armor_match: false
      classification: false
      pose: false
      stats_interval: 50
```

**Note:** ROS 2 YAML uses nested `debug:` block which maps to dot-separated parameter names (`debug.show`, `debug.lights`, etc.) via `declare_parameter<bool>("debug.show", true)`.

- [ ] **Step 2: Commit**

```bash
git add src/armor_detector/config/config.yaml
git commit -m "config: add debug section with layer toggles and stats_interval"
```

---

### Task 10: Split and Refactor DetectorNode

This is the largest task. It splits the class into `.hpp`/`.cpp`, extracts init methods, wires all new debug components, updates `run()` with timing events, and updates `pollDebugKeys()` for layer toggling.

**Files:**
- Modify: `src/armor_detector/include/armor_detector/DetectorNode.hpp` (replace stub)
- Modify: `src/armor_detector/src/DetectorNode.cpp` (full rewrite)

- [ ] **Step 1: Write DetectorNode.hpp**

Replace the full content of `include/armor_detector/DetectorNode.hpp`:

```cpp
#pragma once

#include "armor_detector/CameraProvider.hpp"
#include "armor_detector/detector/Detector.hpp"
#include "armor_detector/detector/LightDetector.hpp"
#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/debug/DebugGUI.hpp"
#include "armor_detector/debug/DebugHub.hpp"
#include "armor_detector/debug/DebugLayerState.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_interfaces/srv/play_next.hpp>
#include <rosbag2_interfaces/srv/set_rate.hpp>
#include <rosbag2_interfaces/srv/toggle_paused.hpp>

#include <cstddef>
#include <string>

namespace armor_detector
{

/**
 * @brief Debug 配置参数，从 ROS 参数 debug.* 读取。
 */
struct DebugConfig
{
    bool show = true;
    bool rosbag_control = true;
    std::string rosbag_player_node = "/rosbag2_player";
    bool preprocess = false;
    bool lights = true;
    bool armor_match = false;
    bool classification = false;
    bool pose = false;
    std::size_t stats_interval = 50;
};

/**
 * @brief 装甲板检测 ROS 2 节点。
 *
 * 订阅 /image_raw，执行预处理 → 灯条检测流水线，
 * 通过 debug observer 体系输出可视化与统计。
 */
class DetectorNode : public rclcpp::Node
{
public:
    DetectorNode();
    ~DetectorNode() override;

private:
    void initParameters();
    void initDetectors();
    void initDebug();
    void initRosbagClients();

    void run(const sensor_msgs::msg::Image::SharedPtr & msg);
    void pollDebugKeys();

    // 检测组件
    CameraProvider camera_provider_;
    Detector detector_;
    LightDetector light_detector_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Debug
    DebugConfig debug_config_;
    debug::DebugGUI debug_gui_;
    debug::DebugHub debug_hub_;
    debug::DebugLayerState layer_state_;
    rclcpp::TimerBase::SharedPtr debug_key_timer_;

    // Rosbag 控制
    rclcpp::Client<rosbag2_interfaces::srv::TogglePaused>::SharedPtr toggle_paused_client_;
    rclcpp::Client<rosbag2_interfaces::srv::PlayNext>::SharedPtr play_next_client_;
    rclcpp::Client<rosbag2_interfaces::srv::SetRate>::SharedPtr set_rate_client_;

    double playback_rate_ = 1.0;
    std::size_t frame_index_ = 0;
};

} // namespace armor_detector
```

**Key differences from old code:**
1. Class declaration in `.hpp` (was entirely in `.cpp`).
2. `DebugConfig` struct replaces scattered member variables.
3. `debug_gui_window_name_` removed — window name is hardcoded as `"debug_show"`.
4. `layer_state_` added as new member.
5. Four `init*()` methods declared.
6. No detector parameter members needed — they're read and used locally in `initDetectors()`.

- [ ] **Step 2: Rewrite DetectorNode.cpp**

Replace the full content of `src/DetectorNode.cpp`:

```cpp
#include "armor_detector/DetectorNode.hpp"
#include "armor_detector/debug/DebugLayerController.hpp"
#include "armor_detector/debug/DebugLight.hpp"
#include "armor_detector/debug/DebugPreprocess.hpp"
#include "armor_detector/debug/DebugTiming.hpp"

#include <algorithm>
#include <memory>

namespace armor_detector
{

DetectorNode::DetectorNode()
    : Node("armor_detector_node_cpp"),
      detector_(Detector::Params{}),
      light_detector_(LightDetector::Params{})
{
    initParameters();
    initDetectors();
    camera_provider_.init();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::QoS(10),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { run(msg); });

    initDebug();
    initRosbagClients();

    RCLCPP_INFO(this->get_logger(), "armor_detector节点已启动.");
}

DetectorNode::~DetectorNode()
{
    debug_gui_.stop();
}

// ===================== init 方法 =====================

void DetectorNode::initParameters()
{
    // Debug 配置
    debug_config_.show = this->declare_parameter<bool>("debug.show", true);
    debug_config_.rosbag_control = this->declare_parameter<bool>("debug.rosbag_control", true);
    debug_config_.rosbag_player_node =
        this->declare_parameter<std::string>("debug.rosbag_player_node", "/rosbag2_player");
    debug_config_.preprocess = this->declare_parameter<bool>("debug.preprocess", false);
    debug_config_.lights = this->declare_parameter<bool>("debug.lights", true);
    debug_config_.armor_match = this->declare_parameter<bool>("debug.armor_match", false);
    debug_config_.classification = this->declare_parameter<bool>("debug.classification", false);
    debug_config_.pose = this->declare_parameter<bool>("debug.pose", false);
    debug_config_.stats_interval =
        static_cast<std::size_t>(this->declare_parameter<int>("debug.stats_interval", 50));
}

void DetectorNode::initDetectors()
{
    std::string target_color_str = this->declare_parameter<std::string>("target_color", "BLUE");
    LightBarColor target_color =
        (target_color_str == "RED") ? LightBarColor::RED : LightBarColor::BLUE;

    Detector::Params det_params;
    det_params.gray_threshold = this->declare_parameter<int>("gray_threshold", 100);
    det_params.color_threshold = this->declare_parameter<int>("color_threshold", 100);
    det_params.target_color = target_color;
    detector_ = Detector(det_params);

    LightDetector::Params light_params;
    light_params.min_contours_area = this->declare_parameter<int>("min_contours_area", 30);
    light_params.min_contours_ratio =
        static_cast<float>(this->declare_parameter<double>("min_contours_ratio", 0.06));
    light_params.max_contours_ratio =
        static_cast<float>(this->declare_parameter<double>("max_contours_ratio", 0.5));
    light_detector_ = LightDetector(light_params);
}

void DetectorNode::initDebug()
{
    if (!debug_config_.show) {
        RCLCPP_INFO(this->get_logger(), "Debug GUI 已禁用 (debug.show=false)");
        return;
    }

    debug_gui_.setEnabled(true);
    debug_gui_.start();

    // 初始化图层状态（从 config 读取初始值）
    layer_state_.setEnabled(debug::DebugLayer::PREPROCESS, debug_config_.preprocess);
    layer_state_.setEnabled(debug::DebugLayer::LIGHTS, debug_config_.lights);
    layer_state_.setEnabled(debug::DebugLayer::ARMOR_MATCH, debug_config_.armor_match);
    layer_state_.setEnabled(debug::DebugLayer::CLASSIFICATION, debug_config_.classification);
    layer_state_.setEnabled(debug::DebugLayer::POSE, debug_config_.pose);

    // 始终注册 timing observer
    debug_hub_.addObserver(
        std::make_shared<debug::DebugTiming>(debug_config_.stats_interval));

    // 注册图层 view observer（内部根据 layer state 决定是否绘制）
    debug_hub_.addObserver(
        std::make_shared<debug::DebugPreprocessView>(debug_gui_, layer_state_));
    debug_hub_.addObserver(
        std::make_shared<debug::DebugLightView>(debug_gui_, layer_state_));

    // 注册图层控制器（处理数字键）
    debug_hub_.addObserver(
        std::make_shared<debug::DebugLayerController>(layer_state_));

    // 按键轮询 timer
    debug_key_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(15),
        [this]() { pollDebugKeys(); });

    RCLCPP_INFO(this->get_logger(),
                "按键操作: [q/ESC]退出  [Space/p]暂停/继续  [n/→]单步  "
                "[s]保存ROI  [+/-]加速/减速  [1-5]切换图层");
}

void DetectorNode::initRosbagClients()
{
    if (!debug_config_.rosbag_control) {
        return;
    }
    const auto & node = debug_config_.rosbag_player_node;
    toggle_paused_client_ = this->create_client<rosbag2_interfaces::srv::TogglePaused>(
        node + "/toggle_paused");
    play_next_client_ = this->create_client<rosbag2_interfaces::srv::PlayNext>(
        node + "/play_next");
    set_rate_client_ = this->create_client<rosbag2_interfaces::srv::SetRate>(
        node + "/set_rate");
}

// ===================== 主流程 =====================

void DetectorNode::run(const sensor_msgs::msg::Image::SharedPtr & msg)
{
    Frame frame = camera_provider_.receiveImage(msg);

    debug::DebugFrameContext ctx;
    ctx.frame_index = frame_index_++;
    ctx.stamp = frame.timestamp;
    ctx.source_bgr = frame.image;
    ctx.display_bgr = frame.image.clone();

    debug_hub_.onFrameStart(ctx);

    cv::Mat img_thre = detector_.preprocess(frame.image);
    debug_hub_.onPreprocess(ctx, detector_.getPreprocessDebugData());

    auto lights = light_detector_.findLights(img_thre, frame.image);
    debug_hub_.onLights(ctx, light_detector_.getLightDebugData());
    (void)lights;

    debug_hub_.onFrameEnd(ctx);

    if (debug_config_.show) {
        debug_gui_.setFrame("debug_show", ctx.display_bgr);
    }
}

// ===================== 按键处理 =====================

void DetectorNode::pollDebugKeys()
{
    auto events = debug_gui_.takeKeyEvents();
    for (const auto & event : events) {
        switch (event.action) {
        case debug::DebugKeyAction::EXIT:
            debug_gui_.stop();
            rclcpp::shutdown();
            break;

        case debug::DebugKeyAction::PAUSE_TOGGLE:
            if (!toggle_paused_client_ || !toggle_paused_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "rosbag toggle_paused service not ready");
                continue;
            }
            (void)toggle_paused_client_->async_send_request(
                std::make_shared<rosbag2_interfaces::srv::TogglePaused::Request>());
            break;

        case debug::DebugKeyAction::STEP_FRAME:
            if (!play_next_client_ || !play_next_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "rosbag play_next service not ready");
                continue;
            }
            (void)play_next_client_->async_send_request(
                std::make_shared<rosbag2_interfaces::srv::PlayNext::Request>());
            break;

        case debug::DebugKeyAction::SAVE_ROI:
            debug_hub_.onKey(event);
            break;

        case debug::DebugKeyAction::TOGGLE_LAYER:
            debug_hub_.onKey(event);
            break;

        case debug::DebugKeyAction::PLAYBACK_RATE_UP:
        case debug::DebugKeyAction::PLAYBACK_RATE_DOWN: {
            if (!set_rate_client_ || !set_rate_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "rosbag set_rate service not ready");
                continue;
            }
            double new_rate = (event.action == debug::DebugKeyAction::PLAYBACK_RATE_UP)
                                  ? playback_rate_ * 1.1
                                  : playback_rate_ / 1.1;
            new_rate = std::clamp(new_rate, 0.1, 10.0);
            auto req = std::make_shared<rosbag2_interfaces::srv::SetRate::Request>();
            req->rate = new_rate;
            (void)set_rate_client_->async_send_request(req);
            playback_rate_ = new_rate;
            RCLCPP_INFO(this->get_logger(), "设置播放倍率: %.2fx", playback_rate_);
            break;
        }

        default:
            break;
        }
    }
}

} // namespace armor_detector

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<armor_detector::DetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

**Key changes from original DetectorNode.cpp:**

1. **Header inclusion:** `#include "armor_detector/DetectorNode.hpp"` replaces all individual includes for types that are now in the header. New includes for `DebugTiming.hpp`, `DebugLayerController.hpp`.
2. **Constructor:** Calls `initParameters()`, `initDetectors()`, `initDebug()`, `initRosbagClients()` instead of inline initialization. Member initializer uses `Detector::Params{}` / `LightDetector::Params{}` (default-constructed Params structs).
3. **`initParameters()`:** Reads all `debug.*` parameters into `debug_config_` using direct assignment style. Uses `declare_parameter<T>("debug.show", default)` which maps to YAML `debug: show: default`.
4. **`initDetectors()`:** Reads detector params and constructs `Detector` / `LightDetector`. Uses local `Params` structs populated by `declare_parameter`.
5. **`initDebug()`:** Guards on `debug_config_.show`. Initializes `layer_state_` from config. Registers `DebugTiming`, `DebugPreprocessView`, `DebugLightView`, `DebugLayerController` observers. Creates key polling timer. All only when `debug.show=true`.
6. **`initRosbagClients()`:** Guards on `debug_config_.rosbag_control`.
7. **`run()`:** Adds `debug_hub_.onFrameStart(ctx)` at the start and `debug_hub_.onFrameEnd(ctx)` after lights. `setFrame` is conditional on `debug_config_.show`. Window name hardcoded as `"debug_show"`.
8. **`pollDebugKeys()`:** New `TOGGLE_LAYER` case forwards to `debug_hub_.onKey(event)`.
9. **No `debug_gui_window_name_` member** — removed per spec.

- [ ] **Step 3: Build to verify**

Run: `colcon build --packages-select armor_detector`
Expected: Build succeeds. All new files are compiled, constructor signatures match, observer registrations are correct.

- [ ] **Step 4: Run lint tests**

Run: `colcon test --packages-select armor_detector`
Expected: All ament lint tests pass (except cpplint/copyright which are skipped).

- [ ] **Step 5: Commit**

```bash
git add src/armor_detector/include/armor_detector/DetectorNode.hpp \
        src/armor_detector/src/DetectorNode.cpp
git commit -m "refactor: split DetectorNode into hpp/cpp, wire debug layer system"
```

---

### Task 11: Update CMakeLists.txt

**Files:**
- Modify: `src/armor_detector/CMakeLists.txt`

- [ ] **Step 1: Add new source files to the executable**

In the `add_executable` block, add two new source files after the existing debug sources (after line 31, `src/debug/DebugLightView.cpp`):

```cmake
add_executable(armor_detector_node
  src/DetectorNode.cpp
  src/detector/ArmorDetector.cpp
  src/detector/Detector.cpp
  src/CameraProvider.cpp
  src/PoseSolver.cpp
  src/detector/LightDetector.cpp
  src/debug/DebugGUI.cpp
  src/debug/DebugMsg.cpp
  src/debug/DebugKeyHandler.cpp
  src/debug/DebugPreprocessView.cpp
  src/debug/DebugLightView.cpp
  src/debug/DebugTiming.cpp
  src/debug/DebugLayerController.cpp
)
```

- [ ] **Step 2: Full build and test**

Run: `colcon build --packages-select armor_detector`
Expected: `[Finished] armor_detector` with 0 errors, 0 warnings (except expected ones).

Run: `colcon test --packages-select armor_detector`
Expected: All tests pass.

- [ ] **Step 3: Whitespace check**

Run: `git diff --check`
Expected: No whitespace errors.

- [ ] **Step 4: Commit**

```bash
git add src/armor_detector/CMakeLists.txt
git commit -m "build: add DebugTiming.cpp and DebugLayerController.cpp to CMake"
```

---

### Task 12: Update Documentation

**Files:**
- Modify: `docs/Debug.md`
- Modify: `docs/DebugGUI.md`

- [ ] **Step 1: Update docs/Debug.md — add debug config section**

In `docs/Debug.md`, replace the "开关规范" section's YAML example (the `debug:` block around line 134–144) with the actual config structure:

```yaml
debug:
  show: true
  rosbag_control: true
  rosbag_player_node: "/rosbag2_player"
  preprocess: false
  lights: true
  armor_match: false
  classification: false
  pose: false
  stats_interval: 50
```

In the "按键规范" section, add the new key mappings **after** the existing `SAVE_ROI` entry (around line 167):

```text
EXIT          退出节点 / 关闭窗口
PAUSE_TOGGLE  rosbag 暂停/继续
STEP_FRAME    rosbag 单步
SAVE_ROI      保存 ROI（预留）
TOGGLE_LAYER  切换图层显示（数字键 1-5）
```

Add a new subsection after "按键规范":

```markdown
## 图层切换

数字键 1–5 切换对应 debug 图层的显示状态：

| 按键 | 图层 | 说明 |
|---|---|---|
| 1 | preprocess | 预处理中间图（独立窗口 preprocess_debug） |
| 2 | lights | 灯条检测结果（叠加到 debug_show） |
| 3 | armor_match | 装甲板匹配（预留，当前只切状态） |
| 4 | classification | 数字分类（预留，当前只切状态） |
| 5 | pose | 位姿解算（预留，当前只切状态） |

切换后输出日志，例如：`DEBUG layer lights: ON`

## Timing 行为

- `timing` 不是图层——`debug.show=true` 时始终启用，无法通过数字键关闭。
- 每帧在 `debug_show` 左上角绘制 `Process` 总耗时和各阶段耗时。
- 每 `debug.stats_interval` 帧（默认 50）打印一次平均耗时日志。
```

- [ ] **Step 2: Update docs/DebugGUI.md — register timing text and lights draw**

In the "debug_show - Text" registration table (around line 136), replace with:

```markdown
## debug_show - Text

| 位置 | 行号 | 内容 | 字体颜色(BGR) | 说明 |
|---|---:|---|---|---|
| TOP_LEFT | 1 | frame processing time (total) | (0, 165, 255) | 当前帧总处理耗时，由 DebugTiming 绘制 |
| TOP_LEFT | 2 | preprocess stage time | (0, 165, 255) | 预处理阶段耗时，由 DebugTiming 绘制 |
| TOP_LEFT | 3 | lights stage time | (0, 165, 255) | 灯条检测阶段耗时，由 DebugTiming 绘制 |
```

**Note:** Line numbers are relative — DebugTiming draws at `y = 24 * line_index`, so line 1 = y:24, line 2 = y:48, line 3 = y:72. Additional stages (armor_match, classification, pose) will add more lines when implemented.

In the "debug_show - Draw" registration table (around line 141), replace with:

```markdown
## debug_show - Draw

| 图层 | 元素 | 内容 | 颜色(BGR) | 样式 | 显示条件 | 说明 |
|---|---|---|---|---|---|---|
| lights | rotated_rect | accepted light bar | (0, 255, 0) | thickness=2, LINE_AA | layer lights ON | 已通过筛选的灯条 |
| lights | point | light endpoints (top/bottom) | (255, 0, 255) | radius=1, filled | layer lights ON | 灯条端点标记 |
| lights | rotated_rect | rejected light bar | (0, 0, 255) | thickness=2, LINE_AA | layer lights ON | 被拒绝的灯条 |
| lights | text | reject reason detail | (0, 0, 255) | scale=0.60, thickness=2 | layer lights ON | 拒绝原因文字，偏移 +5px |
```

- [ ] **Step 3: Commit**

```bash
git add docs/Debug.md docs/DebugGUI.md
git commit -m "docs: update debug config, layer keys, timing, and GUI registrations"
```

---

## Verification Checklist

After all tasks are complete, run the full verification sequence:

- [ ] **Build:** `colcon build --packages-select armor_detector` — 0 errors
- [ ] **Test:** `colcon test --packages-select armor_detector` — all pass
- [ ] **Whitespace:** `git diff --check` — clean
- [ ] **Manual run:** `source install/setup.bash && ros2 launch armor_detector video1.launch.py`
- [ ] **debug_show** displays `Process`, `preprocess`, `lights` timing text at top-left
- [ ] Every 50 frames, terminal prints `【平均用时】` log
- [ ] Press **1**: toggles `preprocess_debug` window; pressing again removes it cleanly (no stale frame)
- [ ] Press **2**: toggles light bar overlay on `debug_show`
- [ ] Press **3/4/5**: toggles state and prints log; no visual change (layers not yet implemented)
- [ ] **q/ESC**, **Space/p**, **n**, **+/-** behave identically to before
- [ ] Set `debug.show: false` in config.yaml → node runs headless with no OpenCV windows
