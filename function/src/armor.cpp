#include "include/armor.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace xz_rm_vision
{
    Lightbar::Lightbar(const cv::RotatedRect &rotated_rect, std::size_t id) : id(id), rotated_rect(rotated_rect)
    {
        std::vector<cv::Point2f> corners(4);
        rotated_rect.points(&corners[0]);
        std::sort(corners.begin(), corners.end(), [](const cv::Point2f &a, const cv::Point2f &b)
                  { return (a.y == b.y) ? (a.x < b.x) : (a.y < b.y) });

        center = rotated_rect.center;
        top = (corners[0] + corners[1]) * 0.5f;
        bottom = (corner[2] + corner[3]) * 0.5f;
        top2bottom = bottom - top;

        points.emplace_back(top);
        points.emplace_back(bottom);

        length = cv::norm(top2bottom);
        float top_w = cv::norm(corners[0] - corners[1]);
        float bot_w = cv::norm(corners[2] - corners[3]);
        width = (top_w + bot_w) * 0.5f;

        ratio = (width > 1e-6) ? (length / width) : 0.0;

        angle = std::atan2(top2bottom.y, top2bottom.x);
        if (angle < 0)
            angle += CV_PI;

        angle_error = std::abs(angle - CV_PI / 2);
        if (angle_error > CV_PI / 2)
            angle_error = CV_PI - angle_error; // 对称误差

        if (length < width)
            std::swap(length, width);
    };

    Armor::Armor(const Lightbar &left, const Lightbar &right) : left(left), right(right), duplicated(false)
    {
        color = left.color;
        center = (left.center + right.center) / 2;

        points.emplace_back(left.top);
        points.emplace_back(right.top);
        points.emplace_back(right.bottom);
        points.emplace_back(left.bottom);

        auto left2right = right.center - left.center;
        auto width = cv::norm(left2right);

        double avg_length = (left.length + right.length) * 0.5;
        ratio = width / avg_length; // 平均计算灯条可能会更平滑？为什么用 max_lightbar_length 而不是平均值：取最大值是更保守的策略（偏严谨）。例如一侧灯条被截断或检测偏短，用最大值能避免把宽度除以一个异常小值导致 ratio 夸大

        double max_lightbar_length = std::max(left.length, right.length);
        double min_lightbar_length = std::min(left.length, right.length);
        side_ratio = max_lightbar_length / min_lightbar_length;

        rectangular_error = ComputeRectangularError(const Lightbar &left, const Lightbar &right);
    };

    // 自动权重调节 + 综合矩形误差计算
    double Armor::ComputeRectangularError(const Light &left, const Light &right)
    {
        // 平均灯条方向
        double angle_diff = std::abs(left.angle - right.angle);
        angle_diff = std::fmod(angle_diff, CV_PI);
        if (angle_diff > CV_PI / 2)
            angle_diff = CV_PI - angle_diff;

        // 与中轴线垂直程度
        auto left2right = right.center - left.center;
        double roll = std::atan2(left2right.y, left2right.x);
        double left_rect_err = std::abs(left.angle - roll - CV_PI / 2);
        double right_rect_err = std::abs(right.angle - roll - CV_PI / 2);
        double rect_err = std::max(left_rect_err, right_rect_err);

        // 动态权重计算
        double distance = cv::norm(left.center - right.center);
        double avg_length = 0.5 * (left.length + right.length);
        double ratio = distance / avg_length;

        // 根据比例自动调整权重（ratio: 0.2 ~ 1.0）
        double weight_angle = std::clamp(0.3 + 0.4 * ratio, 0.3, 0.7);
        double weight_rect = 1.0 - weight_angle;

        // 综合误差
        double rectangular_error = weight_angle * angle_diff + weight_rect * rect_err;

        return rectangular_error;
    }

}