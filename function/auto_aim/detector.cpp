#include "detector.hpp"

namespace xz_vision
{
    Detector::Detector(const std::string &config_path, bool debug)::classifier_(config_path), debug_(debug)
    {
        auto yaml = YAML::LoadFile(config_path);

        threshold_ = yaml["threshold"].as<double>();
        max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3; // degree to rad
        min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
        max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();
        min_lightbar_length_ = yaml["min_lightbar_length"].as<double>();
        min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();
        max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();
        max_side_ratio_ = yaml["max_side_ratio"].as<double>();
        min_confidence_ = yaml["min_confidence"].as<double>();
        max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3; // degree to rad

        save_path_ = "patterns";
        std::filesystem::create_directory(save_path_);
    }

    std::list<Armor> Detector::detect(const cv::Mat &bgr_img, int frame_count)
    {
        // 彩色图转灰度图
        cv::Mat gray_img;
        cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);

        // 进行二值化
        cv::Mat binary_img;
        cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
        // cv::imshow("binary_img", binary_img);

        // 获取轮廓点
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        // 获取灯条
        std::size_t lightbar_id = 0;
        std::list<Lightbar> lightbars;
        for (const auto &contour : contours)
        {
            auto rotated_rect = cv::minAreaRect(contour);
            auto lightbar = Lightbar(rotated_rect, lightbar_id);

            if (!check_geometry(lightbar))
                continue;

            lightbar.color = get_color(bgr_img, contour);
            lightbars.emplace_back(lightbar);
            lightbar_id += 1;
        }

        // 将灯条从左到右排序
        lightbars.sort([](const Lightbar &a, const Lightbar &b)
                       { return a.center.x < b.center.x; });

        // 获取装甲板
        std::list<Armor> armors;
        for (auto left = lightbars.begin(); left != lightbars.end(); left++)
        {
            for (auto right = std::next(left); right != lightbars.end(); right++)
            {
                if (left->color != right->color)
                    continue;

                auto armor = Armor(*left, *right);
                if (!check_geometry(armor))
                    continue;

                armor.pattern = get_pattern(bgr_img, armor);
                classifier_.classify(armor);
                if (!check_name(armor))
                    continue;

                armor.type = get_type(armor);
                if (!check_type(armor))
                    continue;

                armor.center_norm = get_center_norm(bgr_img, armor.center);
                armors.emplace_back(armor);
            }
        }

        // 检查装甲板是否存在共用灯条的情况
        for (auto armor1 = armors.begin(); armor1 != armors.end(); armor1++)
        {
            for (auto armor2 = std::next(armor1); armor2 != armors.end(); armor2++)
            {
                if (
                    armor1->left.id != armor2->left.id && armor1->left.id != armor2->right.id &&
                    armor1->right.id != armor2->left.id && armor1->right.id != armor2->right.id)
                {
                    continue;
                }

                // 装甲板重叠, 保留roi小的
                if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id)
                {
                    auto area1 = armor1->pattern.cols * armor1->pattern.rows;
                    auto area2 = armor2->pattern.cols * armor2->pattern.rows;
                    if (area1 < area2)
                        armor2->duplicated = true;
                    else
                        armor1->duplicated = true;
                }

                // 装甲板相连，保留置信度大的
                if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id)
                {
                    if (armor1->confidence < armor2->confidence)
                        armor1->duplicated = true;
                    else
                        armor2->duplicated = true;
                }
            }
        }

        armors.remove_if([&](const Armor &a)
                         { return a.duplicated; });

        if (debug_)
            show_result(binary_img, bgr_img, lightbars, armors, frame_count);

        return armors;
    }

}
