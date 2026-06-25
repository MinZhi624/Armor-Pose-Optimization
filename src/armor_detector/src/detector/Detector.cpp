#include "armor_detector/detector/Detector.hpp"

#include <algorithm>
#include <opencv2/imgproc.hpp>

namespace armor_detector {

    Detector::Detector(const Params &params) : params_(params) {
    }

    cv::Mat Detector::preprocess(const cv::Mat &img_bgr) {
        // COLOR 阈值
        std::vector<cv::Mat> bgr;
        cv::split(img_bgr, bgr);
        cv::Mat color_ch = (params_.target_color == LightBarColor::BLUE) ? bgr[0] : bgr[2];
        cv::Mat target_color_dim_thre;
        cv::threshold(color_ch, target_color_dim_thre, params_.color_threshold, 255, cv::THRESH_BINARY);

        // GRAY 阈值
        cv::Mat gray;
        cv::cvtColor(img_bgr, gray, cv::COLOR_BGR2GRAY);
        cv::Mat gray_thre;
        cv::threshold(gray, gray_thre, params_.gray_threshold, 255, cv::THRESH_BINARY);

        // 对每个 color_dim 区域，检查 GRAY 碎片数，构建合并二值图
        std::vector<std::vector<cv::Point>> target_color_contours;
        cv::findContours(target_color_dim_thre, target_color_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // 过滤面积太小或点数不足的区域
        target_color_contours.erase(
            std::remove_if(target_color_contours.begin(),
                           target_color_contours.end(),
                           [](const auto &c) { return c.size() < 6 || cv::contourArea(c) < 30; }),
            target_color_contours.end());

        // 以空白为底图，GRAY 碎片==1 用 GRAY，>=3 为噪音跳过，其余用 COLOR_dim
        cv::Mat img_thre = cv::Mat::zeros(img_bgr.size(), CV_8UC1);
        cv::Mat roi_mask;
        for (const auto &target_color_contour : target_color_contours) {
            cv::Rect color_bb = cv::boundingRect(target_color_contour);
            cv::Rect safe_bb = color_bb & cv::Rect(0, 0, gray_thre.cols, gray_thre.rows);
            cv::Mat gray_roi = gray_thre(safe_bb);

            std::vector<std::vector<cv::Point>> gray_roi_contours;
            cv::findContours(gray_roi, gray_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            int valid_count = 0;
            for (const auto &c : gray_roi_contours) {
                if (cv::contourArea(c) > 5) {
                    valid_count++;
                }
            }

            if (valid_count == 1) {
                gray_thre(safe_bb).copyTo(img_thre(safe_bb));
            }
            else if (valid_count >= 3) {
                // 噪音，跳过
            }
            else {
                // 0 or 2：用 COLOR_dim 覆盖
                roi_mask = cv::Mat::zeros(safe_bb.size(), CV_8UC1);
                std::vector<cv::Point> local_contour;
                local_contour.reserve(target_color_contour.size());
                for (const auto &pt : target_color_contour) {
                    local_contour.push_back(pt - cv::Point(safe_bb.x, safe_bb.y));
                }
                cv::fillConvexPoly(roi_mask, local_contour, 255);
                target_color_dim_thre(safe_bb).copyTo(img_thre(safe_bb), roi_mask);
            }
        }

        // 闭运算
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
        cv::dilate(img_thre, img_thre, kernel);
        cv::erode(img_thre, img_thre, kernel);

        // 填充 debug 数据
        preprocess_debug_.gray = gray_thre;
        preprocess_debug_.img_thre = img_thre;
        preprocess_debug_.color_mask = target_color_dim_thre;

        return img_thre;
    }

} // namespace armor_detector
