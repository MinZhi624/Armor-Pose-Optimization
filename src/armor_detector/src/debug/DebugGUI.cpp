#include "armor_detector/debug/DebugGUI.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace armor_detector::debug
{

DebugGUI::~DebugGUI()
{
    stop();
}

void DebugGUI::start()
{
    if (!enabled_.load() || running_.load()) {
        return;
    }
    running_.store(true);
    thread_ = std::thread(&DebugGUI::loop, this);
}

void DebugGUI::stop()
{
    running_.store(false);
    if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id()) {
        thread_.join();
    }
    {
        std::lock_guard<std::mutex> lock(frames_mutex_);
        frames_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(key_mutex_);
        key_queue_.clear();
    }
}

void DebugGUI::setFrame(const std::string & window_name, const cv::Mat & image, double display_scale)
{
    if (image.empty()) {
        return;
    }
    DebugWindowFrame frame;
    frame.window_name = window_name;
    frame.image = image.clone();
    if (display_scale != 1.0) {
        cv::resize(frame.image, frame.image, cv::Size(), display_scale, display_scale, cv::INTER_LINEAR);
    }
    frame.display_scale = display_scale;
    std::lock_guard<std::mutex> lock(frames_mutex_);
    frames_[window_name] = std::move(frame);
}

void DebugGUI::loop()
{
    while (running_.load()) {
        std::unordered_map<std::string, DebugWindowFrame> snapshot;
        {
            std::lock_guard<std::mutex> lock(frames_mutex_);
            snapshot = frames_;
        }
        for (const auto & [name, frame] : snapshot) {
            cv::imshow(name, frame.image);
        }
        int raw_key = cv::waitKeyEx(1);
        if (raw_key != -1) {
            DebugKeyEvent event = key_handler_.translate(raw_key);
            if (event.action != DebugKeyAction::NONE) {
                std::lock_guard<std::mutex> lock(key_mutex_);
                if (key_queue_.size() >= 16) {
                    key_queue_.pop_front();
                }
                key_queue_.push_back(event);
            }
        }
    }
    cv::destroyAllWindows();
}

std::vector<DebugKeyEvent> DebugGUI::takeKeyEvents()
{
    std::vector<DebugKeyEvent> events;
    std::lock_guard<std::mutex> lock(key_mutex_);
    events.assign(key_queue_.begin(), key_queue_.end());
    key_queue_.clear();
    return events;
}

} // namespace armor_detector::debug
