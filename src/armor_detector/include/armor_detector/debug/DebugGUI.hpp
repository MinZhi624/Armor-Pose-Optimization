#pragma once

#include "armor_detector/debug/DebugData.hpp"
#include "armor_detector/debug/DebugKeyHandler.hpp"

#include <opencv2/core.hpp>

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace armor_detector::debug
{

struct DebugWindowFrame
{
    std::string window_name;   // 冗余字段，快照显示用；写入时必须与 map key 同步
    cv::Mat image;
    double display_scale = 1.0;
};

class DebugGUI
{
public:
    DebugGUI() = default;
    ~DebugGUI();

    void setEnabled(bool enabled) { enabled_.store(enabled); }
    bool enabled() const { return enabled_.load(); }

    void start();   // 启动 GUI 线程 (if enabled_ && !running_)
    void stop();    // 设 running_=false, join 线程, 清空队列

    // 线程安全: clone + resize 后覆盖同名窗口帧
    void setFrame(const std::string & window_name, const cv::Mat & image, double display_scale = 1.0);

    // ROS 线程调用: 取走并清空 pending key events
    std::vector<DebugKeyEvent> takeKeyEvents();

    // 线程安全: 从 frames_ 移除同名窗口帧，并请求 GUI 线程销毁窗口
    void clearFrame(const std::string & window_name);

private:
    void loop();    // GUI 线程主循环

    std::atomic<bool> enabled_{true};
    std::atomic<bool> running_{false};
    std::thread thread_;

    // frames_ 用 mutex 保护; 同名窗口覆盖
    // map key 是权威窗口名，DebugWindowFrame.window_name 冗余但快照时有用
    std::mutex frames_mutex_;
    std::unordered_map<std::string, DebugWindowFrame> frames_;

    // 有界按键队列, 容量 16, 满了丢最旧
    std::mutex key_mutex_;
    std::deque<DebugKeyEvent> key_queue_;

    DebugKeyHandler key_handler_;

    // 待销毁窗口列表，由 clearFrame() 写入，loop() 中 GUI 线程消费
    std::mutex destroy_mutex_;
    std::vector<std::string> pending_destroy_;
};

} // namespace armor_detector::debug
