#pragma once

#include <list>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "armor.hpp"
#include "classifier.hpp"

namespace xz_rm_vision
{
    // 装甲板检测类
    class Detector
    {
    public:
        Detector(const std::string &config_path, bool debug = true);

        std::list<Aromr> detect(const cv::Mat &bgr_img, int frame_count = -1);

    private:
        Classifier classifier_;

        double threshold_;
        double max_angle_error_;
        double min_lightbar_ratio_, max_lightbar_ratio_;
        double min_lightbar_length_;
        double min_armor_ratio_, max_armor_ratio_;
        double max_side_ratio_;
        double min_confidence_;
        double max_rectangular_error_;

        bool debug_;
        std::string save_path_;
    }
}